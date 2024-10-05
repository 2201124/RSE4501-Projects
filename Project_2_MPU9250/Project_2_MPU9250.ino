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

static float accelXSum = 0, accelYSum = 0, accelZSum = 0;
static float gyroXSum = 0, gyroYSum = 0, gyroZSum = 0;
bool calibrated = false;

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
  count++;

  if (count == 10) {
    float avgAccelX = accelXSum / 10.0;
    float avgAccelY = accelYSum / 10.0;
    float avgAccelZ = accelZSum / 10.0;

    float avgGyroX = gyroXSum / 10.0;
    float avgGyroY = gyroYSum / 10.0;
    float avgGyroZ = gyroZSum / 10.0;

    Serial.print("Average Accel : "); Serial.print(avgAccelX); Serial.print(", ");
    Serial.print(avgAccelY); Serial.print(", ");
    Serial.println(avgAccelZ);

    Serial.print("Average Gyro  : "); Serial.print(avgGyroX); Serial.print(", ");
    Serial.print(avgGyroY); Serial.print(", ");
    Serial.println(avgGyroZ);

    accelXSum = 0;
    accelYSum = 0;
    accelZSum = 0;
    gyroXSum = 0;
    gyroYSum = 0;
    gyroZSum = 0;
    count = 0;
  }
  delay(100);
}



void calibrateGyro() {
  Serial.println("Calibrating Gyroscope");
  float gyroXSum = 0, gyroYSum = 0, gyroZSum = 0;
  const int numSamples = 300;

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
