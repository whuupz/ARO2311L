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
  readSensor1Data();
  readSensor2Data();
  // Add your main loop logic here
}

void readSensor2Data() {
  digitalWrite(13, HIGH);
  delay(10);
  
  float pressureKPA = 0, temperatureC = 0;    
  pressureKPA = mpl115a2.getPressure();  
  Serial.print("Pressure (kPa): "); Serial.print(pressureKPA, 4); Serial.println(" kPa");
  sendRadioTransmission("Pressure:", pressureKPA, pressureKPA, pressureKPA);

  temperatureC = mpl115a2.getTemperature();  
  Serial.print("Temp (*C): "); Serial.print(temperatureC, 1); Serial.println(" *C");
  sendRadioTransmission("Temp:", temperatureC, temperatureC, temperatureC);
  
  digitalWrite(13, LOW);
}

void readSensor1Data() {
  sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  printEvent(&orientationData);
  printEvent(&angVelocityData);
  printEvent(&linearAccelData);
  printEvent(&magnetometerData);
  printEvent(&accelerometerData);
  printEvent(&gravityData);

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

void printEvent(sensors_event_t* event) {
  float x = -1000000, y = -1000000, z = -1000000;

  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    sendRadioTransmission("Accl:", event->acceleration.x, event->acceleration.y, event->acceleration.z);
  } else if (event->type == SENSOR_TYPE_ORIENTATION) {
    sendRadioTransmission("Orient:", event->orientation.x, event->orientation.y, event->orientation.z);
  } else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    sendRadioTransmission("Mag:", event->magnetic.x, event->magnetic.y, event->magnetic.z);
  } else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    sendRadioTransmission("Gyro:", event->gyro.x, event->gyro.y, event->gyro.z);
  } else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    sendRadioTransmission("Rot:", event->gyro.x, event->gyro.y, event->gyro.z);
  } else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    sendRadioTransmission("Linear:", event->acceleration.x, event->acceleration.y, event->acceleration.z);
  } else if (event->type == SENSOR_TYPE_GRAVITY) {
    Serial.print("Gravity:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  } else {
    sendRadioTransmission("Unk:", x, y, z);
  }
}

void sendRadioTransmission(const char* sensorType, float x, float y, float z) {
  Serial.print(sensorType);
  Serial.print("\tx= "); Serial.print(x);
  Serial.print(" |\ty= "); Serial.print(y);
  Serial.print("\t|\tz= "); Serial.println(z);

  char radiopacket[30];
  sprintf(radiopacket, "%s\tx=%.2f |\ty=%.2f |\tz=%.2f", sensorType, x, y, z);
  
  Serial.print("Sending: ");
  Serial.println(radiopacket);

  rf95.send((uint8_t *)radiopacket, strlen(radiopacket));
  rf95.waitPacketSent();

  Serial.println("Waiting for reply...");
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (rf95.waitAvailableTimeout(1000)) {
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
}