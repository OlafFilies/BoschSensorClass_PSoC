/**
 * @file PSoC_MagnetometerDrift.ino
 * @author Dr Olaf Filies
 * @brief Tests the BMM350 magnetometer stability and drift
 * @version 0.1
 * @date 2025-06-05
 * 
 * @copyright Copyright (c) 2025
 * 
 */


#include "BoschSensorClass_PSoC.hpp"

// Switch to 1 if you also want to collect the sensor temperature
#define WITH_TEMPERATURE  1

bmm350_mag_temp_data drift;

void setup()
{
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
    if (IMU_PSoC.magneticSensorPreset(BMM350_DATA_RATE_12_5HZ, BMM350_AVERAGING_8) != BMM350_OK )
    {
        Serial.println("Failed to set data rate and noise level!");
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
    if (IMU_PSoC.magneticPowerMode(BMM350_FORCED_MODE) != BMM350_OK )
    {
        Serial.println("Failed to set power mode!");
        while (1);
    }


    float x, y, z, t;
    for (int i = 0; i < 10; i++)
    {
        IMU_PSoC.readMagneticField(x, y, z, t);
        drift.x += x;
        drift.y += y;
        drift.z += z;
        drift.temperature += t;
        delay(100);
    }
    drift.x /= 10.0f;
    drift.y /= 10.0f;
    drift.z /= 10.0f;
    drift.temperature /= 10.0f;

    Serial.println("Drift values:");
    Serial.println("X\tY\tZ\tTemperature");
    Serial.println("µT\tµT\tµT\t°C");
       
    Serial.print(drift.x);  Serial.print('\t');
    Serial.print(drift.y);  Serial.print('\t');
    Serial.print(drift.z);  Serial.print('\t');
    Serial.println(drift.temperature);

    delay(1000);
    Serial.println("Interrupt on threshold started");
}


void loop()
{
    float x, y, z, t;

    IMU_PSoC.readMagneticField(x, y, z, t);

    Serial.print(drift.x - x);  Serial.print('\t');
    Serial.print(drift.y - y);  Serial.print('\t');
    Serial.print(drift.z - z);

    drift.x += x;
    drift.y += y;
    drift.z += z;

    drift.x /= 2.0f;
    drift.y /= 2.0f;
    drift.z /= 2.0f;

    if (WITH_TEMPERATURE)
    {
        Serial.print('\t');
        Serial.print(drift.temperature - t);
        drift.temperature += t;
        drift.temperature /= 2.0f;
    }
    Serial.println("");

    delay(100);

}