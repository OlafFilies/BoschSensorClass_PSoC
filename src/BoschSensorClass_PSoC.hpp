/**
 * @file BoschSensorClassPSoC.h
 * @author 
 * @brief 
 * @version 0.0.1
 * @date 2025-05-21
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <Arduino.h>
#include <Wire.h>
#include "utilities/BMI270-Sensor-API/bmi270.h"
#include "utilities/BMM350-Sensor-API/bmm350.h"

typedef enum {
  BOSCH_ACCELEROMETER_ONLY,
  BOSCH_MAGNETOMETER_ONLY,
  BOSCH_ACCEL_AND_MAGN
} CfgBoshSensor_t;

struct dev_info {
  TwoWire* _wire;
  uint8_t dev_addr;
};



class BoschSensorClassPSoC {
    
    public:
        BoschSensorClassPSoC(TwoWire& wire = Wire);
        ~BoschSensorClassPSoC() {}

        int begin(CfgBoshSensor_t cfg = BOSCH_ACCEL_AND_MAGN);
        void end();
        void debug(Stream&);
        void setContinuousMode();
        void oneShotMode();

        // Accelerometer
        virtual int readAcceleration(float& x, float& y, float& z); // Results are in G (earth gravity).
        virtual int accelerationAvailable(); // Number of samples in the FIFO.
        virtual float accelerationSampleRate(); // Sampling rate of the sensor.

        // Gyroscope
        virtual int readGyroscope(float& x, float& y, float& z); // Results are in degrees/second.
        virtual int gyroscopeAvailable(); // Number of samples in the FIFO.
        virtual float gyroscopeSampleRate(); // Sampling rate of the sensor.

        // Magnetometer
        virtual int readMagneticField(float& x, float& y, float& z);
        virtual int readMagneticField(float& x, float& y, float& z, float& t);

        virtual int magneticSensorPreset(enum bmm350_data_rates rate = BMM350_DATA_RATE_400HZ, enum bmm350_performance_parameters performance = BMM350_REGULARPOWER);
        virtual int magneticPowerMode(enum bmm350_power_modes power = BMM350_NORMAL_MODE);                          //
        virtual int magneticInterruptMode(enum bmm350_interrupt_enable_disable interrupt = BMM350_DISABLE_INTERRUPT);                      //
        virtual int magneticSetThreshold(int8_t threshold, enum bmm350_intr_polarity polarity);


    protected:
        // can be modified by subclassing for finer configuration
        virtual int8_t configure_sensor(struct bmm350_dev *dev);
        virtual int8_t configure_sensor(struct bmi2_dev *dev) ;

    private:

        TwoWire* _wire;
        Stream* _debug = nullptr;
        bool _initialized = false;
        bool continuousMode;
        int _interrupts = 0;
        struct dev_info accel_gyro_dev_info;
        struct dev_info mag_dev_info;
        struct bmi2_dev bmi270;
        struct bmm350_dev bmm350;
        uint16_t _int_status;

        static int8_t bosch_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
        static int8_t bosch_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
        static void delay_us(uint32_t period, void *intf_ptr);
        void bmi270_print_rslt(int8_t rslt);
        void bmm350_print_rslt(int8_t rslt);
        void panic_led_trap(const char* errhead, const char* errtext,int8_t rslt);

        struct bmm350_mag_temp_data magnetic_threshold;
        enum bmm350_performance_parameters bmm350_performance = BMM350_REGULARPOWER;           /*! Power mode of sensor */
        enum bmm350_data_rates bmm350_data_rate               = BMM350_DATA_RATE_400HZ;        /*! Data rate value (ODR) */
        enum bmm350_power_modes bmm350_pwr_mode               = BMM350_NORMAL_MODE;            /*! Preset mode of sensor */
        enum bmm350_interrupt_enable_disable bmm350_interrupt = BMM350_DISABLE_INTERRUPT;
        int8_t bmm350_threshold = 0;

};


extern BoschSensorClassPSoC IMU_BMI270_BMM350;
#undef IMU_PSoC
#define IMU_PSoC IMU_BMI270_BMM350

