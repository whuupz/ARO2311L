#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_MPL115A2.h>
#include <SPI.h>
#include <RH_RF95.h>

Adafruit_MPL115A2 mpl115a2;
/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);


#define SENSOR1_ADDRESS 0xC0  //pressure sensor address   / to stop 0x12
#define SENSOR2_ADDRESS 0x28   // to stop 0x40    imu addresses

#define RFM95_CS    8
#define RFM95_INT   3
#define RFM95_RST   4

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);

void setup() {
  digitalWrite(13,HIGH);
  Wire.begin();
  
  Serial.begin(9600);
  bno.begin();
  mpl115a2.begin();
  delay(200);
  digitalWrite(13,LOW);

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  rf95.setTxPower(23, false);
}

int16_t packetnum = 0;

void loop() {

  sensors_event_t accelData;
  bno.getEvent(&accelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  double accelX = accelData.acceleration.x;
  double accelY = accelData.acceleration.y;
  double accelZ = accelData.acceleration.z;
  sensors_event_t magData;
  bno.getEvent(&magData, Adafruit_BNO055::VECTOR_MAGNETOMETER); 
  double magX = magData.magnetic.x;
  double magY = magData.magnetic.y;
  double magZ = magData.magnetic.z;
  sensors_event_t gyroData;
  bno.getEvent(&gyroData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  double gyroX = gyroData.gyro.x;
  double gyroY = gyroData.gyro.y;
  double gyroZ = gyroData.gyro.z;

  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  double orientationX = orientationData.orientation.x;
  double orientationY = orientationData.orientation.y;
  double orientationZ = orientationData.orientation.z;

  imu::Quaternion quat = bno.getQuat();

  //delay(100);
  //digitalWrite(13, HIGH);
  float pressureKPA = 0, temperatureC = 0;
  pressureKPA = mpl115a2.getPressure();
  temperatureC = mpl115a2.getTemperature();
  //digitalWrite(13, LOW);
  //delay(100);

  char radiopacket[200]; // Adjust the array size accordingly
  sprintf(radiopacket,
        "Q:%.2f,%.2f,%.2f,%.2f|A:%.2f,%.2f,%.2f|M:%.2f,%.2f,%.2f|G:%.2f,%.2f,%.2f|O:%.2f,%.2f,%.2f|T:%.2f|P:%.2f|",
        quat.x(), quat.y(), quat.z(), quat.w(),
        accelX, accelY, accelZ, // Corrected lines
        magX, magY, magZ,       // Corrected lines
        gyroX, gyroY, gyroZ,
        orientationY, orientationY, orientationZ,
        temperatureC,
        pressureKPA);

  Serial.print("Sending: ");
  Serial.println(radiopacket);

  rf95.send((uint8_t *)radiopacket, strlen(radiopacket));
  rf95.waitPacketSent();

  Serial.println("Waiting for reply...");
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (rf95.waitAvailableTimeout(1)) {
    if (rf95.recv(buf, &len)) {
      Serial.print("Got reply: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
    } else {
      Serial.println("Receive failed");
    }
  } else {
    Serial.println("No reply, is there a listener around?");
  }
  

  int8_t boardTemp = bno.getTemp();
  Serial.println();
  Serial.print(F("temperature: "));
  Serial.println(boardTemp);
  

  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.println();
  Serial.print("Calibration: Sys=");
  Serial.print(system);
  Serial.print(" Gyro=");
  Serial.print(gyro);
  Serial.print(" Accel=");
  Serial.print(accel);
  Serial.print(" Mag=");
  Serial.println(mag);

  Serial.println("--");
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
