#include "FastIMU.h"
#include <Wire.h>

#define IMU_ADDRESS 0x68

MPU6500 mpu;

calData calib = { 0 };  // Calibration data
GyroData gyroData;
AccelData accelData;

float AccelXOffset = 0.03;
float AccelYOffset = 0.02;
float AccelZOffset = 1.09;

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
    Serial.println("MPU6500 Initialized");
  }
}

void loop() {
  float accelXSum = 0, accelYSum = 0, accelZSum = 0;
  float gyroXSum = 0, gyroYSum = 0, gyroZSum = 0;

  // Collect 10 readings and sum them up
  for (int i = 0; i < 10; i++) {
    // Get sensor data
    mpu.update();
    mpu.getAccel(&accelData);
    mpu.getGyro(&gyroData);

    // Accumulate Accelerometer readings
    accelXSum += (accelData.accelX - AccelXOffset);
    accelYSum += (accelData.accelY - AccelYOffset);
    accelZSum += (accelData.accelZ - AccelZOffset);

    // Accumulate Gyroscope readings
    gyroXSum += gyroData.gyroX;
    gyroYSum += gyroData.gyroY;
    gyroZSum += gyroData.gyroZ;

    delay(100); // Delay between readings to control sampling rate
  }

  // Calculate the average after 10 readings
  float avgAccelX = accelXSum / 10.0;
  float avgAccelY = accelYSum / 10.0;
  float avgAccelZ = accelZSum / 10.0;

  float avgGyroX = gyroXSum / 10.0;
  float avgGyroY = gyroYSum / 10.0;
  float avgGyroZ = gyroZSum / 10.0;

  // Print averaged values
  Serial.print("Average Accel : "); Serial.print(avgAccelX); Serial.print(", ");
  Serial.print(avgAccelY); Serial.print(", ");
  Serial.println(avgAccelZ);

  Serial.print("Average Gyro  : "); Serial.print(avgGyroX); Serial.print(", ");
  Serial.print(avgGyroY); Serial.print(", ");
  Serial.println(avgGyroZ);
}
