/*
  Arduino BMI270 - Simple Gyroscope

  This example reads the gyroscope values from the BMI270
  sensor and continuously prints them to the Serial Monitor
  or Serial Plotter.

  The circuit:
  - Arduino Nano 33 BLE Sense Rev2

  created 10 Jul 2019
  by Riccardo Rizzo

  This example code is in the public domain.
*/

#include "BoschSensorClass_PSoC.hpp"



void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Started");

    
  if (!IMU_PSoC.begin(BOSCH_ACCELEROMETER_ONLY)) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU_PSoC.gyroscopeSampleRate(BMI2_GYR_ODR_100HZ));
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Gyroscope in degrees/second");
  Serial.println("X\tY\tZ");
}

void loop() {
  float x, y, z;

  if (IMU_PSoC.gyroscopeAvailable()) {
    IMU_PSoC.readGyroscope(x, y, z);

    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.println(z);
  }
}
