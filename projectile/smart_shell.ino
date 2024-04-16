/*
captures and sends projectile data 
https://lastminuteengineers.com/mpu6050-accel-gyro-arduino-tutorial/
https://github.com/plerup/espsoftwareserial/
https://randomnerdtutorials.com/bme280-sensor-arduino-pressure-temperature-humidity/
https://github.com/PowerBroker2/SerialTransfer
https://github.com/Makerfabs/Makerfabs-ESP32-UWB/tree/main/example/tag/uwb_tag
*/
#include <Wire.h>
#include <SPI.h>

#include "DW1000Ranging.h"
#include "altitude.h"

#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_MPU6050.h>

#include <SoftwareSerial.h>
#include "SerialTransfer.h"

#define TAG_ADDR "7D:00:22:EA:82:60:3B:9B"

#define DW_CS 4
#define SPI_MOSI 23
#define SPI_SCK 18
#define SPI_MISO 19

#define I2C_SDA 22
#define I2C_SCL 21
#define UART_RX 32  //IO32 -> TX
#define UART_TX 33  //IO33 -> RX

#define SEALEVELPRESSURE_HPA (1013.25)

#define GYRO_X_OFFSET 0.10
#define ACCEL_STDDEV 0.5505
#define GYRO_STDDEV 0.0005
#define BARO_STDDEV 0.02
#define CA 1
#define ACCEL_THRESH 25

#define G 9.8

// 0.0005, 	// sigma Accel
//                                                0.0005, 	// sigma Gyro
//                                                0.018,   // sigma Baro
//                                                0.5, 	// ca
//                                                0.1)

// Altitude estimator
AltitudeEstimator altitude = AltitudeEstimator(ACCEL_STDDEV, 	GYRO_STDDEV, 	BARO_STDDEV, CA, ACCEL_THRESH);	

Adafruit_MPU6050 mpu;

Adafruit_BMP3XX bmp;

EspSoftwareSerial::UART HC12;

SerialTransfer myTransfer;

// connection pins
const uint8_t PIN_RST = 27;  // reset pin
const uint8_t PIN_IRQ = 34;  // irq pin
const uint8_t PIN_SS = 4;    // spi select pin

struct __attribute__((packed)) STRUCT {
  float altitude;
  float distance;
  float vel_y;
  float a_y;
} testStruct;

void setup(void) {

  testStruct.altitude = -1;
  testStruct.vel_y = -1;
  testStruct.distance = -1;

  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);

  HC12.begin(9600, SWSERIAL_8N1, UART_RX, UART_TX, false);
  if (HC12) {  // If the object did not initialize, then its configuration is invalid
    Serial.println("valid EspSoftwareSerial pin configuration");
  }
  myTransfer.begin(HC12);

  while (!Serial)
    delay(10);  // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
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
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
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

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
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

  bool status;

  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  status = bmp.begin_I2C();
  if (!status) {
    Serial.println("Could not find a valid BMP390 sensor, check wiring!");
    while (1)
      ;
  }
  Serial.println("BMP390 Found!");
 // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);


  //init the configuration
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ);  //Reset, CS, IRQ pin
  //define the sketch as anchor. It will be great to dynamically change the type of module
  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);
  //Enable the filter to smooth the distance
  DW1000Ranging.useRangeFilter(true);

  //we start the module as a tag
  DW1000Ranging.startAsTag(TAG_ADDR, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, DW1000.CHANNEL_2, false);
}

long int runtime = 0;
void loop() {
  if ((millis() - runtime) > 60) {
    sendData();
    runtime = millis();
    //printAcceleration();
    //printHeight();
  }
  // if tag within 0.5 m of the anchor
  // and is Vy = 0
  // for 1 second
  // set relative height to zero
  
  DW1000Ranging.loop();
}

void sendData() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  bmp.performReading();
  float accel[3] = {a.acceleration.x/G, a.acceleration.y/G, a.acceleration.z/G}; 
  float gyro[3] = {g.gyro.x + GYRO_X_OFFSET, g.gyro.y, g.gyro.z};
  float baroHeight = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  testStruct.distance = DW1000Ranging.getDistantDevice()->getRange();
  altitude.estimate(accel, gyro, baroHeight, micros());

  testStruct.vel_y = altitude.getVerticalVelocity();
  testStruct.a_y = altitude.getVerticalAcceleration();
  testStruct.altitude = altitude.getAltitude();
  myTransfer.sendDatum(testStruct);
}

void printAcceleration() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
}

void printHeight() {
  Serial.print("Temperature = ");
  Serial.print(bmp.temperature);
  Serial.println(" *C");

  // Convert temperature to Fahrenheit
  /*Serial.print("Temperature = ");
  Serial.print(1.8 * bme.readTemperature() + 32);
  Serial.println(" *F");*/

  Serial.print("Pressure = ");
  Serial.print(bmp.pressure / 100.0F);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.println();
}

void newRange() {
  Serial.print("from: ");
  Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
  Serial.print("\t Range: ");
  Serial.print(DW1000Ranging.getDistantDevice()->getRange());
  Serial.print(" m");
  Serial.print("\t RX power: ");
  Serial.print(DW1000Ranging.getDistantDevice()->getRXPower());
  Serial.println(" dBm");
}

void newDevice(DW1000Device *device) {
  Serial.print("ranging init; 1 device added ! -> ");
  Serial.print(" short:");
  Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device) {
  Serial.print("delete inactive device: ");
  Serial.println(device->getShortAddress(), HEX);
}