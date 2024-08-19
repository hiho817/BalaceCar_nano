//****** pinout define ******//

//          ||D13    typeC    D12||
//          ||3V3             D11||
//          ||REF             D10||
//          ||A0               D9||
//          ||A1               D8||DIR_R
//          ||A2               D7||DIR_L
//          ||A3               D6||STEP_R
//   IMU_SDA||A4               D5||STEP_L
//   IMU_SCL||A5               D4||STEP_EN
//          ||A6               D3||
//          ||A7               D2||
//          ||5V              GND||
//          ||RST             RST||
//          ||GND              RX||BT_TX
//          ||VIN              TX||BT_RX

#define STEP_R 6
#define STEP_L 5
#define DIR_R 8
#define DIR_L 7
#define STEP_EN 4 //enable pin for stepper motor driver
#define motorInterfaceType 1
#define Debug_LED 13 // Debug LED on nano board

// #define DEBUG // Enable debug mode for serial output command it out to disable

// define some variables
float pitch = 0;
float roll = 0;
float yaw = 0;

float pitch_prev = 0;
float roll_prev = 0;
float yaw_prev = 0;

float accelOffset[3] = {0, 0, 0}; // X, Y, Z offsets for accelerometer
float gyroOffset[3] = {0, 0, 0};  // X, Y, Z offsets for gyroscope

float elapsedTime_settling = 0.05;
float maxspeed = 5000.0;

unsigned long previousTime = 0;
float elapsedTime = 0;
unsigned long currentTime = 0;

float setPoint = 0;      // Desired angle (upright position)
float kp_balance = 0.03;  // Proportional gain
float ki_balance = 0.00;  // Integral gain
float kd_balance = 0.00; // Derivative gain

float pid_pitch = 0 ;

float error_prev = 0;       // Store the previous error for derivative calculation
float integral = 0;         // Store the cumulative error for integral calculation
unsigned long lastTime = 0; // Store the last time for derivative calculation
