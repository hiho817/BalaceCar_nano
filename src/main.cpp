#include <Arduino.h>

//****** pinout define ******//

//          ||D13    typeC    D12||
//          ||3V3             D11||
//          ||REF             D10||
//          ||A0               D9||
//          ||A1               D8||DIR_R
//          ||A2               D7||DIR_L
//          ||A3               D6||STEP_R
//   IMU_SDA||A4               D5||STEP_L
//   IMU_SCL||A5               D4||
//          ||A6               D3||
//          ||A7               D2||
//          ||5V              GND||
//          ||RST             RST||
//          ||GND              RX||BT_TX
//          ||VIN              TX||BT_RX


// define some variables
String command;

bool initIMU(){
}

bool initBT(){
  Serial.begin(38400); // Default communication rate of the Bluetooth module
  Serial.println("intialize BlueTooth sucessful");
}



void setup() {
  initBT();
  initIMU();
}

void loop() {
  if(Serial.available() > 0){ // Checks whether data is comming from the serial port
    command = Serial.read(); // Reads the data from the serial port
    //Serial.println(command);
  }
  
}