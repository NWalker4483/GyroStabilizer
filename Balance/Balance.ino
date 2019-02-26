#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this code but required!

/////////////////////////
#define X_ServoPin      9
#define Y_ServoPin      8
////////////////////////
float P_GAIN =           0.002;
float I_GAIN =           0.0;
float D_GAIN =           100.5;
int   UPDATE_FREQUENCY = 300; //Hz
#define DEADZONE 15 // Degrees
/////////////////////////
#define MAX_ANGLE       180 // %
#define MIN_SPEED       85 // %
#define MAX_SPEED       105 // %

#define Y_MIN_SPEED       85 // %
#define Y_MAX_SPEED       95 // %


#define LoopTime        100   // PID loop time(ms)
//Globals
unsigned long lastMilli = 0; // time at the end of the last loop

int current_x_speed_setting = 0;
int current_y_speed_setting = 0;
int actual_x_angle = 0;
int actual_y_angle = 0;

float goal_x_angle = 90;
float goal_y_angle = 90;
float current_steering_angle = 0;

struct config {          
  bool  Clamped = false;
  float IntegralTerm = 0;
  float DerivativeTerm = 0;
  float PID_Output = 0;
  float AngleError = 0;
  float lastAngleError = 0;
}; 

struct config x_config;
struct config y_config;
Servo X_Servo;
Servo Y_Servo;
// Initialize the IMU Sensor
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

float RwEst[3];     //Rw estimated from combining RwAcc and RwGyro
unsigned long lastMicros;

//Variables below don't need to be global but we expose them for debug purposes
unsigned long interval; //interval since previous analog samples
float RwAcc[3];         //projection of normalized gravitation force vector on x/y/z axis, as measured by accelerometer
float RwGyro[3];        //Rw obtained from last estimated value and gyro movement
float GyroTemp[3];
float Awz[2];           //angles between projection of R on XZ/YZ plane and Z axis (deg)

// Bound the input value between x_min and x_max. Also works in anti-windup
int CheckClamp(int x, struct config beta) {
  int angle = constrain(x, MIN_SPEED, MAX_SPEED); // Angle Limit
  beta.Clamped = not (angle == x);
  return angle;
}

void ApplyKalmanFiltering() {
  static int w;
  static float tmpf, tmpf2;
  static unsigned long newMicros; //new timestamp
  static char signRzGyro;
  float wGyro = 20;
  ////BELOW is likely removable
  //compute interval since last sampling time
  interval = newMicros - lastMicros;    //please note that overflows are ok, since for example 0x0001 - 0x00FE will be equal to 2
  lastMicros = newMicros;               //save for next loop, please note interval will be invalid in first sample but we don't use it


  //normalize vector (convert to a vector with same direction and with length 1)
  normalize3DVector(RwAcc);

  //evaluate RwGyro vector
  if (abs(RwEst[2]) < 0.1) {
    //Rz is too small and because it is used as reference for computing Axz, Ayz it's error fluctuations will amplify leading to bad results
    //in this case skip the gyro data and just use previous estimate
    for (w = 0; w <= 2; w++) RwGyro[w] = RwEst[w];
  } else {
    //get angles between projection of R on ZX/ZY plane and Z axis, based on last RwEst
    for (w = 0; w <= 1; w++) {
      tmpf = GyroTemp[w];                             //get current gyro rate in deg/ms
      tmpf *= interval / 1000.0f;                     //get angle change in deg
      Awz[w] = atan2(RwEst[w], RwEst[2]) * 180 / PI;  //get angle and convert to degrees
      Awz[w] += tmpf;                                 //get updated angle according to gyro movement
    }

    //estimate sign of RzGyro by looking in what qudrant the angle Axz is,
    //RzGyro is pozitive if  Axz in range -90 ..90 => cos(Awz) >= 0
    signRzGyro = (cos(Awz[0] * PI / 180) >= 0 ) ? 1 : -1;

    //reverse calculation of RwGyro from Awz angles, for formulas deductions see  http://starlino.com/imu_guide.html
    for (w = 0; w <= 1; w++) {
      RwGyro[0] = sin(Awz[0] * PI / 180);
      RwGyro[0] /= sqrt( 1 + squared(cos(Awz[0] * PI / 180)) * squared(tan(Awz[1] * PI / 180)) );
      RwGyro[1] = sin(Awz[1] * PI / 180);
      RwGyro[1] /= sqrt( 1 + squared(cos(Awz[1] * PI / 180)) * squared(tan(Awz[0] * PI / 180)) );
    }
    RwGyro[2] = signRzGyro * sqrt(1 - squared(RwGyro[0]) - squared(RwGyro[1]));
  }

  //combine Accelerometer and gyro readings
  for (w = 0; w <= 2; w++) RwEst[w] = (RwAcc[w] + wGyro * RwGyro[w]) / (1 + wGyro);

  normalize3DVector(RwEst);
}

void UpdatePIDController_X() {
  // compute the error between the measurement and the desired value
  x_config.AngleError = -ShortestAngularPath(g2degree(RwEst[1]), goal_x_angle);
  if (abs(x_config.AngleError) <= DEADZONE) { // Deadzone // Stop if close enough to prevent oscillations
    current_x_speed_setting = 90;
    set_X_Speed(current_x_speed_setting);
  } else {
    x_config.DerivativeTerm = x_config.AngleError - x_config.lastAngleError;
    // If the actuator is saturating ignore the integral term
    // if the system is clamped and the sign of the integrator term and the sign of the PID output are the same
    if (x_config.Clamped and ((x_config.PID_Output / abs(x_config.PID_Output)) == (x_config.IntegralTerm / abs(x_config.IntegralTerm)))) {
      x_config.IntegralTerm += 0;
    } else {
      x_config.IntegralTerm += x_config.AngleError;
    }
    // compute the control effort by multiplying the error by Kp
    x_config.PID_Output = (x_config.AngleError * P_GAIN) + (x_config.IntegralTerm * I_GAIN) + (x_config.DerivativeTerm * D_GAIN);
    current_x_speed_setting += x_config.PID_Output;

    // make sure the output value is bounded to 0 to 100 using the bound function defined below
    current_x_speed_setting = CheckClamp(current_x_speed_setting,x_config);
    set_X_Speed(current_x_speed_setting); // then write it to the LED pin to change control voltage to LED
  }
}

void UpdatePIDController_Y() {
  // compute the error between the measurement and the desired value
  y_config.AngleError = ShortestAngularPath(g2degree(RwEst[0]), goal_y_angle); // Minimum degree shifts in order to reach goal
  if (abs(y_config.AngleError) <= DEADZONE) { // Deadzone
    current_y_speed_setting = 90;
    set_Y_Speed(current_y_speed_setting);
  } else {
    y_config.DerivativeTerm = y_config.AngleError - y_config.lastAngleError;
    // If the actuator is saturating ignore the integral term
    // if the system is clamped and the sign of the integrator term and the sign of the PID output are the same
    if (y_config.Clamped and ((y_config.PID_Output / abs(y_config.PID_Output)) == (y_config.IntegralTerm / abs(y_config.IntegralTerm)))) {
      y_config.IntegralTerm += 0; 
    } else {
      y_config.IntegralTerm += y_config.AngleError;
    }
    // compute the control effort by multiplying the error by Kp
    y_config.PID_Output = (y_config.AngleError * P_GAIN) + (y_config.IntegralTerm * I_GAIN) + (y_config.DerivativeTerm * D_GAIN);
    current_y_speed_setting += y_config.PID_Output;

    // make sure the output value is bounded to 0 to 100 using the bound function defined below
    current_y_speed_setting = CheckClamp(current_y_speed_setting,y_config);
    set_Y_Speed(current_y_speed_setting); // then write it to the LED pin to change control voltage to LED
  }
}

void getMachineState() {
  // Convert DOF data to angles
  lsm.read();
  sensors_event_t a, m, g, temp;
  bool isUpsideDown;
  lsm.getEvent(&a, &m, &g, &temp);

  RwAcc[0] = a.acceleration.x / 9.80665; // m/s^2 in g
  RwAcc[1] = a.acceleration.y / 9.80665; // m/s^2 in g
  RwAcc[2] = a.acceleration.z / 9.80665; // m/s^2 in g

  GyroTemp[0] = g.gyro.y / 1000;
  GyroTemp[1] = g.gyro.x / 1000;
  GyroTemp[2] = g.gyro.z / 1000;
}

void setup() {
  X_Servo.attach(X_ServoPin);
  Y_Servo.attach(Y_ServoPin);
  set_X_Speed(90);
  set_Y_Speed(90);
  Serial.begin(115200);
  while (!Serial) {
    delay(1); // will pause Zero, Leonardo, etc until serial console opens
  }

  Serial.println("LSM9DS1 Stabilized Stick Demo");

  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
  Serial.println("Found LSM9DS1 9DOF");

  // helper to just set the default scaling we want, see above!
  setupSensor();
  // Add Comment
  getMachineState();
  for (int i = 0; i <= 2; i++) RwEst[i] = RwAcc[i]; //initialize with accelerometer readings
}

void loop() {
  getMachineState();
  ApplyKalmanFiltering();
  UpdatePIDController_X();
  UpdatePIDController_Y();
  delay(1000 / UPDATE_FREQUENCY); 
}

// Minimum degree shifts in order to reach goal
float ShortestAngularPath(int angle, int goal) {
  double raw_diff = angle > goal ? angle - goal : goal - angle;
  double dist = raw_diff > 180.0 ? 360.0 - raw_diff : raw_diff;
  if (goal <= 180) { // Theoretically fails for goal angles above 180 degrees
    if ((angle < goal) or (angle > (goal + 180))) {
      dist = dist;
    }
    else {
      dist = dist * -1;
    }
  }
  return dist;
}

void set_X_Speed(int angle) {
  angle = constrain(angle, MIN_SPEED, MAX_SPEED);
  X_Servo.write(angle);
}

void set_Y_Speed(int angle) {
  angle = constrain(angle, Y_MIN_SPEED, Y_MAX_SPEED);
  Y_Servo.write(angle);
}

void set_X_Goal(int angle) { // -180 :-: 180
  angle = constrain(angle, -MAX_ANGLE, MAX_ANGLE); // Speed Limit
  goal_x_angle = angle;
}

void set_Y_Goal(int angle) { // -18 0 :-: 180
  angle = constrain(angle, -MAX_ANGLE, MAX_ANGLE); // Speed Limit
  goal_y_angle = angle;
}

void normalize3DVector(float* vector) {
  static float R;
  R = sqrt(squared(vector[0]) + squared(vector[1]) + squared(vector[2]));
  vector[0] /= R;
  vector[1] /= R;
  vector[2] /= R;
}

void setupSensor() {
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
}

float squared(float x) {return x * x;}

int g2degree(float g) {return constrain(((g + 1) / 2) * 180, 0, 180);}