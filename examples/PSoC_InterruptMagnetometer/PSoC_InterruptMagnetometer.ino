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
    if (IMU_PSoC.magneticSensorPreset(BMM350_DATA_RATE_50HZ, BMM350_ULTRALOWNOISE) != BMM350_OK )
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
     * Set threshold interrupt, an interrupt is triggered when the geomagnetic value of a channel is beyond/below the threshold
     * High polarity: active on high level, the default is low level, which turns to high level when the interrupt is triggered.
     * Low polarity: active on low level, the default is high level, which turns to low level when the interrupt is triggered.
     * threshold  //Threshold range, default to expand 16 times, for example: under low threshold mode, if the threshold is set to be 1, actually the geomagnetic data below 16 will trigger an interrupt
     * polarity:
     *   BMM350_ACTIVE_HIGH
     *   BMM350_ACTIVE_LOW
     */
    if (IMU_PSoC.magneticSetThreshold(-10,BMM350_ACTIVE_LOW) != BMM350_OK )
    {
        Serial.println("Failed to set interrupt!");
        while (1);
    }

    pinMode(35 ,INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(35), BMM350_Interrupt, CHANGE);

    Serial.println("Interrupt on threshold started");
}

void loop() {

    if(interruptFlag == 1){
    // Without interruptFlag we only test the DRDY register if new data is available
        bmm350_threshold_data_t thresholdData = IMU_PSoC.magneticGetThreshold();

        // Serial.print("Threshold x = ");
        // if(thresholdData.i_x != 0){
        //     Serial.print(thresholdData.m_x);
        //     Serial.print(" µT on Interrupt\t");
        //     if (thresholdData.m_x < 0)
        //         Serial.println("LOW");
        //     if (thresholdData.m_x > 0)
        //         Serial.println("HIGH");
        // }else{
        //     Serial.println("none");
        // }

        // Serial.print("Threshold y = ");
        // if(thresholdData.i_y != 0){
        //     Serial.print(thresholdData.m_y);
        //     Serial.print(" µT on Interrupt\t"); 
        //     if (thresholdData.m_y < 0)
        //         Serial.println("LOW");
        //     if (thresholdData.m_y > 0)
        //         Serial.println("HIGH");
        // }else{And 
        //     Serial.println("none");
        // }

        // Serial.print("Threshold z = ");
        // if(thresholdData.i_z != 0){
        //     Serial.print(thresholdData.m_z);
        //     Serial.print(" µT on Interrupt\t"); 
        //     if (thresholdData.m_z < 0)
        //         Serial.println("LOW");
        //     if (thresholdData.m_z > 0)
        //         Serial.println("HIGH");
        // }else{
        //     Serial.println("none");
        // }
        Serial.println();

        interruptFlag = 0;
        attachInterrupt(digitalPinToInterrupt(35), BMM350_Interrupt, CHANGE);

    }
    delay(1000);

}