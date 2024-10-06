#include "FastIMU.h"

#define IMU_ADDRESS 0x68

MPU6500 mpu;

calData calib = {0};  // Calibration data
GyroData gyroData;
AccelData accelData;

// offsets
float AccelXOffset = 0.03;
float AccelYOffset = 0.02;
float AccelZOffset = 1.09;
float gyroXOffset = 0, gyroYOffset = 0, gyroZOffset = 0;

static float accelXSum = 0, accelYSum = 0, accelZSum = 0;
static float gyroXSum = 0, gyroYSum = 0, gyroZSum = 0;
bool calibrated = false;
unsigned long prevTime = 0;

static float angleX = 0, angleY = 0, angleZ = 0;

// Complementary Filter Variables
float compfilterangle = 0;
float alpha = 0.98;  // Adjusted alpha value to balance gyro and accelerometer

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);  // 400kHz clock
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

  // Get current time for delta calculation
  unsigned long currTime = millis();
  float deltaTime = (currTime - prevTime) / 1000.0;
  prevTime = currTime;
  static float accumulatedDeltaTime = 0;
  accumulatedDeltaTime += deltaTime;

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

  if (count == 10) {
    float avgAccelX = accelXSum / 10.0;
    float avgAccelY = accelYSum / 10.0;
    float avgAccelZ = accelZSum / 10.0;

    float avgGyroX = (gyroXSum / 10.0);
    float avgGyroY = (gyroYSum / 10.0);
    float avgGyroZ = (gyroZSum / 10.0);
    
    // Calculate accelerometer angle for roll
    float accelAngleX = atan2(avgAccelY, avgAccelZ) * RAD_TO_DEG;

    // Debugging: Print raw accelerometer angle
    Serial.print("Accel Roll Angle: ");
    Serial.println(accelAngleX);

    // Complementary filter: combine accelerometer and gyroscope data
    compfilterangle = alpha * (compfilterangle + avgGyroX * accumulatedDeltaTime) + (1 - alpha) * accelAngleX;

    // Print filtered roll angle
    Serial.print("Filtered Roll Angle: ");
    Serial.println(compfilterangle);

    // Calculate total acceleration (for fall detection)
    float TotalAccel = sqrt(sq(avgAccelX) + sq(avgAccelY) + sq(avgAccelZ));
    Serial.print("Total Acceleration: ");
    Serial.println(TotalAccel);

    // Debugging: print other angles
    // Serial.print("Angle X: (Roll)"); Serial.print(angleX); Serial.print(", ");
    // Serial.print("Pitch : "); Serial.print(angleY); Serial.print(", ");
    // Serial.print("Angle Z (Yaw): "); Serial.println(angleZ);

    // Serial.print(avgAccelX); Serial.print(", ");
    // Serial.print(avgAccelY); Serial.print(", ");
    // Serial.print(avgAccelZ);Serial.print(", ");

    // Serial.print(avgGyroX); Serial.print(", ");
    // Serial.print(avgGyroY); Serial.print(", ");
    // Serial.println(avgGyroZ);

    // Reset accumulated values
    accelXSum = 0;
    accelYSum = 0;
    accelZSum = 0;
    gyroXSum = 0;
    gyroYSum = 0;
    gyroZSum = 0;
    count = 0;
    accumulatedDeltaTime = 0;  // Reset after averaging
  }

  delay(10);
}

void calibrateGyro() {
  Serial.println("Calibrating Gyroscope");
  float gyroXSum = 0, gyroYSum = 0, gyroZSum = 0;
  const int numSamples = 1000;  // Increased number of samples for better accuracy

  for (int i = 0; i < numSamples; i++) {
    mpu.update();
    mpu.getGyro(&gyroData);
    gyroXSum += gyroData.gyroX;
    gyroYSum += gyroData.gyroY;
    gyroZSum += gyroData.gyroZ;

    delay(10);  // Delay between readings to ensure stable averaging
  }

  gyroXOffset = gyroXSum / numSamples;
  gyroYOffset = gyroYSum / numSamples;
  gyroZOffset = gyroZSum / numSamples;

  Serial.print("Gyro Calibration Complete. Offset Values are: ");
  Serial.print("X: ");
  Serial.print(gyroXOffset);
  Serial.print(", Y: ");
  Serial.print(gyroYOffset);
  Serial.print(", Z: ");
  Serial.println(gyroZOffset);
  calibrated = true;
}
