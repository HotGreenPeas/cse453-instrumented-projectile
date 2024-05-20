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
  float altitude;
  float distance;
  float vel_y;
  float a_y;
} testStruct;

void setup() {
  Serial.begin(9600);  // Serial port to computer
  HC12.begin(9600);    // Serial port to HC12
  myTransfer.begin(HC12);
}

void loop() {
  if (myTransfer.available()) {
    myTransfer.rxObj(testStruct);
    Serial.print("vertical velo:");
    Serial.print(testStruct.vel_y);
    Serial.print(",");
    Serial.print("altitude:");
    Serial.print(testStruct.altitude);
    Serial.print(",");
    Serial.print("distance:");
    Serial.println(testStruct.distance);
    Serial.print(",");
    Serial.print("accel:");
    Serial.println(testStruct.a_y);
  }
}