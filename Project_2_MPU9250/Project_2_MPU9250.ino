#include "FastIMU.h"
#include <Wire.h>

#define IMU_ADDRESS 0x68

MPU6500 mpu;

calData calib = { 0 };  // Calibration data
AccelData accelData;    // Sensor data
GyroData gyroData;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // 400kHz clock
  delay(1000);

  int err = mpu.init(calib, IMU_ADDRESS);
  if (err != 0) {
    Serial.print("Error initializing IMU: ");
    Serial.println(err);
    while (true) {
      ;
    }
  } else {
    Serial.println("MPU9250 Initialized");
  }
}

void loop() {
  mpu.update();
  // Get Accelerometer Data
  mpu.getAccel(&accelData);
  Serial.print(accelData.accelX); Serial.print(",");
  Serial.print(accelData.accelY); Serial.print(",");
  Serial.print(accelData.accelZ); Serial.print(",");

  // Get Gyro Data
  mpu.getGyro(&gyroData);
  Serial.print(gyroData.gyroX); Serial.print(",");
  Serial.print(gyroData.gyroY); Serial.print(",");
  Serial.println(gyroData.gyroZ);
}
