/**
 * @file PSoC_SimpleMagnetometer.ino
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-06-03
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "BoschSensorClass_PSoC.hpp"

// Switch to 1 if you also want to collect the sensor temperature
#define WITH_TEMPERATURE  0


void setup() {
    Serial.begin(115200);
    while (!Serial);
    Serial.println("Started");

    // To configure the BMM3500
    if (!IMU_PSoC.begin(BOSCH_MAGNETOMETER_ONLY)) {
        Serial.println("Failed to initialize IMU_PSoC!");
        while (1);
    }

    /** the sample rate can be set to
     * 400Hz                  |  BMM350_DATA_RATE_400HZ (default)
     * 200Hz                  |  BMM350_DATA_RATE_200HZ
     * 100Hz                  |  BMM350_DATA_RATE_100HZ
     * 50Hz                   |  BMM350_DATA_RATE_50HZ
     * 25Hz                   |  BMM350_DATA_RATE_25HZ
     * 12.5Hz                 |  BMM350_DATA_RATE_12_5HZ 
     * 6.25Hz                 |  BMM350_DATA_RATE_6_25HZ
     * 3.125Hz                |  BMM350_DATA_RATE_3_125HZ
     * 1.5625Hz               |  BMM350_DATA_RATE_1_5625HZ
     *
     * low power/highest noise    |  BMM350_NO_AVERAGING  BMM350_LOWPOWER
     * lesser noise               |  BMM350_AVERAGING_2   BMM350_REGULARPOWER
     * even lesser noise          |  BMM350_AVERAGING_4   BMM350_LOWNOISE
     * lowest noise/highest power |  BMM350_AVERAGING_8   BMM350_ULTRALOWNOISE
     */
    if (IMU_PSoC.magneticSensorPreset(BMM350_DATA_RATE_12_5HZ, BMM350_AVERAGING_2) != BMM350_OK )
    {
        Serial.println("Failed to set data rate and nois level!");
        while (1);
    }

    /**
     *                powermode |   Power mode
     * -------------------------|-----------------------
     *                          |  BMM350_SUSPEND_MODE
     *                          |  BMM350_NORMAL_MODE
     *                          |  BMM350_FORCED_MODE
     *                          |  BMM350_FORCED_MODE_FAST
     */
    if (IMU_PSoC.magneticPowerMode(BMM350_NORMAL_MODE) != BMM350_OK )
    {
        Serial.println("Failed to set power mode!");
        while (1);
    }

    /**
     * Enable or disable x,y,z axes, be default all three are enabled
     */
    if (IMU_PSoC.magneticEnableAxes() != BMM350_OK )
    {
        Serial.println("Failed to enable/disable axes!");
        while (1);
    }

    Serial.println("Everything set, we can start");

    Serial.println();
    Serial.println("Magnetic Field in uT");
    Serial.println("X\tY\tZ");
}

void loop() {
    float x, y, z, t;

    IMU_PSoC.readMagneticField(x, y, z, t);

    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.print(z);
    if (WITH_TEMPERATURE)
    {
      Serial.print('\t');
      Serial.print(t);
    }
    Serial.println("");

    delay(10);
}
