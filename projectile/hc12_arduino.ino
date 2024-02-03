/* 
  Recieves data sent from instrumented projectile

  References: 
  Arduino Long Range Wireless Communication using HC-12
  Example 01
  by Dejan Nedelkovski, www.HowToMechatronics.com

  SerialTransfer
  uart_rx_datum Example
  https://github.com/PowerBroker2/SerialTransfer/blob/master/examples/uart_rx_datum/uart_rx_datum.ino
*/

#include <SoftwareSerial.h>
#include "SerialTransfer.h"

SoftwareSerial HC12(10, 11);  // HC-12 TX Pin, HC-12 RX Pin
SerialTransfer myTransfer;

struct __attribute__((packed)) STRUCT {
  float tempBME;
  float altitude;
  float distance;
  float tempMPU;
  float accel_x;
  float accel_y;
  float accel_z;
} testStruct;


void setup() {
  Serial.begin(9600);  // Serial port to computer
  HC12.begin(9600);    // Serial port to HC12
  myTransfer.begin(HC12);
}

void loop() {
  if (myTransfer.available()) {
    myTransfer.rxObj(testStruct);
    Serial.print("temp BME: ");
    Serial.println(testStruct.tempBME);
    Serial.print("altitude: ");
    Serial.println(testStruct.altitude);
    Serial.print("distance: ");
    Serial.println(testStruct.distance);
    Serial.print("temp MPU: ");
    Serial.println(testStruct.tempMPU);
    Serial.print("accel x: ");
    Serial.println(testStruct.accel_x);
    Serial.print("accel y: ");
    Serial.println(testStruct.accel_y);
    Serial.print("accel z: ");
    Serial.println(testStruct.accel_z);
    Serial.println();
  }
}