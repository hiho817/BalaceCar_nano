#include <Arduino.h>
bool initIMU(){

}
bool initBT(){

}



void setup() {
  Serial.begin(115200);
  if(!initIMU()){
    Serial.println("intialize IMU sucessful");
  }
  if(!initBT()){
    Serial.println("intialize BlueTooth sucessful");
  }

}

void loop() {
}