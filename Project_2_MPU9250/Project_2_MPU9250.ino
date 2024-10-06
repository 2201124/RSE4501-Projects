#include "FastIMU.h"

#define IMU_ADDRESS 0x68

MPU6500 mpu;

calData calib = { 0 };  // Calibration data
GyroData gyroData;
AccelData accelData;

//offsets
float AccelXOffset = 0.03;
float AccelYOffset = 0.02;
float AccelZOffset = 1.09;
float gyroXOffset = 0, gyroYOffset = 0, gyroZOffset = 0;

float accelXSum = 0, accelYSum = 0, accelZSum = 0;
float gyroXSum = 0, gyroYSum = 0, gyroZSum = 0;
bool calibrated = false;
unsigned long prevTime = 0;

float angleX = 0, angleY = 0, angleZ = 0;

float alpha = 0.80;  // Complementary filter coefficient
float angleX_cf = 0, angleY_cf = 0;  // Angles from complementary filter

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
    calibrateGyro();
  }
}

void loop() {
  if (!calibrated) return;

  static int count = 0;

  // Get current time so that can convert gyro data to degrees
  unsigned long currTime = micros();
  float deltaTime = (currTime - prevTime) / 1000000.0;
  prevTime = currTime;

  mpu.update();
  mpu.getAccel(&accelData);
  mpu.getGyro(&gyroData);

  // Apply gyro offsets
  float gyroX = gyroData.gyroX - gyroXOffset;
  float gyroY = gyroData.gyroY - gyroYOffset;
  float gyroZ = gyroData.gyroZ - gyroZOffset;

  accelXSum += (accelData.accelX - AccelXOffset);
  accelYSum += (accelData.accelY - AccelYOffset);
  accelZSum += (accelData.accelZ - AccelZOffset);

  gyroXSum += gyroX;
  gyroYSum += gyroY;
  gyroZSum += gyroZ;

  angleX += gyroX * deltaTime;
  angleY += gyroY * deltaTime;
  angleZ += gyroZ * deltaTime;
  
  count++;

  

    float accelAngleX = atan2(accelData.accelY, sqrt(accelData.accelX * accelData.accelX + accelData.accelZ * accelData.accelZ)) * 180 / PI;
    float accelAngleY = atan2(-accelData.accelX, sqrt(accelData.accelY * accelData.accelY + accelData.accelZ * accelData.accelZ)) * 180 / PI;

    // Integrate gyroscope data to get change in angles
    float deltaAngleX = gyroX * deltaTime;
    float deltaAngleY = gyroY * deltaTime;
    // Complementary filter to combine accelerometer and gyroscope data
    angleX_cf = alpha * (angleX_cf + deltaAngleX) + (1.0 - alpha) * accelAngleX;
    angleY_cf = alpha * (angleY_cf + deltaAngleY) + (1.0 - alpha) * accelAngleY;

    Serial.print("Complementary Filter Angle X: ");
    Serial.print(angleX_cf);
    Serial.print(", Angle Y: ");
    Serial.println(angleY_cf);


  if (count == 10) {
    float avgAccelX = accelXSum / 10.0;
    float avgAccelY = accelYSum / 10.0;
    float avgAccelZ = accelZSum / 10.0;

    float avgGyroX = (gyroXSum / 10.0);
    float avgGyroY = (gyroYSum / 10.0);
    float avgGyroZ = (gyroZSum / 10.0);

    float TotalAccel = sqrt(sq(avgAccelX) + sq(avgAccelY) + sq(avgAccelZ));
    // Serial.print("Total Accel: "); Serial.println(TotalAccel);
    // Serial.print("Angle X (Roll): "); Serial.print(angleX); Serial.print(", ");
    // Serial.print("Angle Y(Pitch): "); Serial.print(angleY); Serial.print(", ");
    // Serial.print("Angle Z  (Yaw): "); Serial.println(angleZ);

    // Serial.print(avgAccelX); Serial.print(", ");
    // Serial.print(avgAccelY); Serial.print(", ");
    // Serial.print(avgAccelZ);Serial.print(", ");

    // Serial.print(avgGyroX); Serial.print(", ");
    // Serial.print(avgGyroY); Serial.print(", ");
    // Serial.println(avgGyroZ);

    accelXSum = 0;
    accelYSum = 0;
    accelZSum = 0;
    gyroXSum = 0;
    gyroYSum = 0;
    gyroZSum = 0;
    count = 0;
  }
  delay(10);
}



void calibrateGyro() {
  Serial.println("Calibrating Gyroscope");
  float gyroXSum = 0, gyroYSum = 0, gyroZSum = 0;
  const int numSamples = 500;

  for (int i = 0; i < numSamples; i++) {
    mpu.update();
    mpu.getGyro(&gyroData);
    gyroXSum += gyroData.gyroX;
    gyroYSum += gyroData.gyroY;
    gyroZSum += gyroData.gyroZ;

    delay(10); // Delay between readings to ensure stable averaging
  }

  gyroXOffset = gyroXSum / numSamples;
  gyroYOffset = gyroYSum / numSamples;
  gyroZOffset = gyroZSum / numSamples;

  Serial.print("Gyro Calibration Complete. Offsets Values are: ");
  Serial.print("X: "); Serial.print(gyroXOffset); Serial.print(", ");
  Serial.print("Y: "); Serial.print(gyroYOffset); Serial.print(", ");
  Serial.print("Z: "); Serial.println(gyroZOffset);
  calibrated = true;
}