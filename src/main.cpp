#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <AccelStepper.h>
//****** pinout define ******//

//          ||D13    typeC    D12||
//          ||3V3             D11||
//          ||REF             D10||
//          ||A0               D9||
//          ||A1               D8||DIR_R
//          ||A2               D7||DIR_L
//          ||A3               D6||STEP_R
//   IMU_SDA||A4               D5||STEP_L
//   IMU_SCL||A5               D4||EN
//          ||A6               D3||
//          ||A7               D2||
//          ||5V              GND||
//          ||RST             RST||
//          ||GND              RX||BT_TX
//          ||VIN              TX||BT_RX

// define some variables
String command;
Adafruit_MPU6050 mpu;
volatile float accAngle, currentAngle, prevAngle = 0, error, prevError = 0, errorSum = 0;
float pitch = 0;
float roll = 0;
float yaw = 0;
float pitch_prev = 0;
float roll_prev = 0;
float accelOffset[3] = {0, 0, 0}; // X, Y, Z offsets for accelerometer
float gyroOffset[3] = {0, 0, 0};  // X, Y, Z offsets for gyroscope
unsigned long previousTime = 0;
float elapsedTime = 0;
#define stepPin1 5
#define dirPin1 7
#define stepPin2 6
#define dirPin2 8

float setPoint = 0;      // Desired angle (upright position)
float kp_balance = 5.0;  // Proportional gain
float ki_balance = 0.0;  // Integral gain
float kd_balance = 0.01; // Derivative gain

float pid_pitch = 0 ;

float error_prev = 0;       // Store the previous error for derivative calculation
float integral = 0;         // Store the cumulative error for integral calculation
unsigned long lastTime = 0; // Store the last time for derivative calculation

AccelStepper stepper1(AccelStepper::DRIVER, stepPin1, dirPin1);
AccelStepper stepper2(AccelStepper::DRIVER, stepPin2, dirPin2);

void initmotor()
{
  stepper1.setMaxSpeed(1000);    // Set max speed for motor 1
  stepper2.setMaxSpeed(1000);    // Set max speed for motor 2
  stepper1.setAcceleration(500); // Set acceleration for motor 1
  stepper2.setAcceleration(500); // Set acceleration for motor 2
}

void calibrateMPU6050()
{
  const int sampleSize = 1000;
  float accelSum[3] = {0, 0, 0}; // X, Y, Z sum for accelerometer
  float gyroSum[3] = {0, 0, 0};  // X, Y, Z sum for gyroscope

  for (int i = 0; i < sampleSize; i++)
  {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    accelSum[0] += a.acceleration.x;
    accelSum[1] += a.acceleration.y;
    accelSum[2] += a.acceleration.z;
    gyroSum[0] += g.gyro.x;
    gyroSum[1] += g.gyro.y;
    gyroSum[2] += g.gyro.z;

    delay(3);
  }

  for (int i = 0; i < 3; i++)
  {
    accelOffset[i] = accelSum[i] / sampleSize;
    gyroOffset[i] = gyroSum[i] / sampleSize;
  }

  // Adjust the accelerometer Z offset to account for gravity (1g)
  accelOffset[2] -= 9.81;
}

bool initIMU()
{
  Serial.println("Adafruit MPU6050 test!");

  // Initialize the MPU6050
  if (!mpu.begin())
  {
    Serial.println("Failed to find MPU6050 chip");
    return false;
  }
  Serial.println("MPU6050 Found!");

  // Configure the accelerometer range
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange())
  {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }

  // Configure the gyroscope range
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange())
  {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  // Configure the digital low-pass filter
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth())
  {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  // Calibrate the MPU6050
  Serial.println("Calibrating MPU6050...");
  calibrateMPU6050();
  Serial.println("Calibration done!");

  return true;
}

bool initBT()
{
  Serial.begin(115200); // Default communication rate of the Bluetooth module
  Serial.println("intialize BlueTooth sucessful");
}

void updateIMU()
{
  // Get new sensor events
  float alpha = 0.4;
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  // Apply the offsets
  accel.acceleration.x -= accelOffset[0];
  accel.acceleration.y -= accelOffset[1];
  accel.acceleration.z -= accelOffset[2];
  gyro.gyro.x -= gyroOffset[0];
  gyro.gyro.y -= gyroOffset[1];
  gyro.gyro.z -= gyroOffset[2];

  // Calculate pitch and roll using accelerometer data
  float pitch_IMU = atan2(accel.acceleration.x, sqrt(sq(accel.acceleration.y) + sq(accel.acceleration.z))) * 180.0 / PI;
  float roll_IMU = atan2(-accel.acceleration.y, sqrt(sq(accel.acceleration.x) + sq(accel.acceleration.z))) * 180.0 / PI;

  pitch = alpha * pitch_IMU + (1 - alpha) * pitch_prev;
  roll = alpha * roll_IMU + (1 - alpha) * roll_prev;
  pitch_prev = pitch_IMU;
  roll_prev = roll_IMU;
  // Update yaw using gyroscope data
  yaw += gyro.gyro.z * elapsedTime;
}

void pid_balance()
{
  // Calculate error
  float error = pitch - setPoint;

  // Calculate integral (I term)
  integral += error * elapsedTime;

  // Calculate derivative (D term)
  float derivative = (error - error_prev) / elapsedTime;

  error_prev = error;

  // Calculate motor speed using PID controller
  pid_pitch = kp_balance * error + ki_balance * integral + kd_balance * derivative;

  pid_pitch = constrain(pid_pitch , -1000 , 1000);
}

void printMPUdata()
{
  // print_counter = micros();
  Serial.print(F(">Pitch:"));
  Serial.println(pitch);
  Serial.print(F(">Row:"));
  Serial.println(roll);
  Serial.print(F(">Yaw:"));
  Serial.println(yaw);
}
void printPIDgain()
{
  Serial.print(F(">pid:"));
  Serial.println(pid_pitch);
}

void setup()
{
  initBT();
  initIMU();
  initmotor();
}

void loop()
{

  unsigned long currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000.0; // Convert to seconds
  previousTime = currentTime;

  updateIMU();
  pid_balance();

  // actuator

  stepper1.setSpeed(stepper1.maxSpeed());
  stepper2.setSpeed(-pid_pitch);
  stepper1.runSpeed();
  stepper2.runSpeed();

  printMPUdata();
  printPIDgain();
  // if(Serial.available() > 0){ // Checks whether data is comming from the serial port
  //   command = Serial.read(); // Reads the data from the serial port
  //   //Serial.println(command);
  // }
}