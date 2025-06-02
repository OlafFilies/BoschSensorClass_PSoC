/**
 * @file PSoC_InterruptMagnetometer.ino
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-06-02
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "BoschSensorClass_PSoC.hpp"


volatile uint8_t interruptFlag = 0;
void BMM350_Interrupt(){
    interruptFlag = 1;
    detachInterrupt(0);
}

void setup() {
    Serial.begin(115200);
    while (!Serial);
    Serial.println("Started");

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
    IMU_PSoC.magneticSensorPreset(BMM350_DATA_RATE_25HZ, BMM350_ULTRALOWNOISE);

    /**
     *                powermode |   Power mode
     * -------------------------|-----------------------
     *                          |  BMM350_SUSPEND_MODE
     *                          |  BMM350_NORMAL_MODE
     *                          |  BMM350_FORCED_MODE
     *                          |  BMM350_FORCED_MODE_FAST
     */
    IMU_PSoC.magneticPowerMode(BMM350_NORMAL_MODE);

    /**
     * Set threshold interrupt, an interrupt is triggered when the geomagnetic value of a channel is beyond/below the threshold
     * High polarity: active on high level, the default is low level, which turns to high level when the interrupt is triggered.
     * Low polarity: active on low level, the default is high level, which turns to low level when the interrupt is triggered.
     * threshold  //Threshold range, default to expand 16 times, for example: under low threshold mode, if the threshold is set to be 1, actually the geomagnetic data below 16 will trigger an interrupt
     * polarity:
     *   BMM350_ACTIVE_HIGH
     *   BMM350_ACTIVE_LOW
     */
    IMU_PSoC.magneticSetThreshold(0,BMM350_ACTIVE_LOW);
    pinMode(P1_0 ,INPUT_PULLUP);
    attachInterrupt(0, BMM350_Interrupt, LOW);

    // To configure the BMM3500
    if (!IMU_PSoC.begin(BOSCH_MAGNETOMETER_ONLY)) {
        Serial.println("Failed to initialize IMU_PSoC!");
        while (1);
    }

    Serial.println("Interrupt on threshold started");
}

void loop() {
    if(interruptFlag == 1){
        bmm350_threshold_data_t thresholdData = IMU_PSoC.magneticGetThreshold();

        if(thresholdData.m_x != 0){
            Serial.print("Threshold x = ");
            Serial.print(thresholdData.m_x);
            Serial.print("µT on Interrupt ");
            if (thresholdData.m_x < 0)
                Serial.println("LOW");
            if (thresholdData.m_x > 0)
                Serial.println("HIGH");
        }
        if(thresholdData.m_y != 0){
            Serial.print("Threshold y = ");
            Serial.print(thresholdData.m_y);
            Serial.print("µT on Interrupt "); 
            if (thresholdData.m_y < 0)
                Serial.println("LOW");
            if (thresholdData.m_y > 0)
                Serial.println("HIGH");
        }
        if(thresholdData.m_z != 0){
            Serial.print("Threshold z = ");
            Serial.print(thresholdData.m_z);
            Serial.print("µT on Interrupt "); 
            if (thresholdData.m_z < 0)
                Serial.println("LOW");
            if (thresholdData.m_z > 0)
                Serial.println("HIGH");
        }
        Serial.println();

        interruptFlag = 0;
        attachInterrupt(0, BMM350_Interrupt, LOW);

    }
    delay(1000);

}