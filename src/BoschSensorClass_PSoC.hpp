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
#include <math.h>

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

typedef struct{
  int32_t m_x;      /* magnetic x data */
  int32_t m_y;      /* magnetic y data */
  int32_t m_z;      /* magnetic z data */
  int8_t i_x;       /* x interrupt -1=low, 0 none, 1=high */ 
  int8_t i_y;       /* y interrupt -1=low, 0 none, 1=high */ 
  int8_t i_z;       /* z interrupt -1=low, 0 none, 1=high */ 
} bmm350_threshold_data_t;

class BoschSensorClassPSoC {
    
    public:
        BoschSensorClassPSoC(TwoWire& wire = Wire);
        ~BoschSensorClassPSoC() {}

        int begin(CfgBoshSensor_t cfg = BOSCH_ACCEL_AND_MAGN);
        void end();
        void debug(Stream&);
        void setContinuousMode();
        void oneShotMode();

        // IMU Features
        virtual int imu_FeatureConfig(const uint8_t *sens_list, uint8_t n_sens, bmi2_int_pin_config int_pin_cfg, bmi2_sens_config sens_cfg);
        virtual int imu_SetDataRate(uint8_t sensor, uint8_t rate);


        // Accelerometer
        virtual int readAcceleration(float& x, float& y, float& z); // Results are in G (earth gravity).
        virtual int accelerationAvailable(); // Number of samples in the FIFO.
        virtual float accelerationSampleRate(uint8_t rate = 0); // Sampling rate of the sensor.

        // Gyroscope
        virtual int readGyroscope(float& x, float& y, float& z); // Results are in degrees/second.
        virtual int gyroscopeAvailable(); // Number of samples in the FIFO.
        virtual float gyroscopeSampleRate(uint8_t rate = 0);

        // Magnetometer
        virtual int readMagneticField(float& x, float& y, float& z);
        virtual int readMagneticField(float& x, float& y, float& z, float& t);
        virtual bmm350_mag_temp_data readMagneticData(void);
        virtual int readCompass(float& compass);

        virtual int magneticSensorPreset(enum bmm350_data_rates rate = BMM350_DATA_RATE_400HZ, enum bmm350_performance_parameters performance = BMM350_REGULARPOWER);
        virtual int magneticPowerMode(enum bmm350_power_modes power = BMM350_NORMAL_MODE);
        virtual int magneticInterruptMode(enum bmm350_interrupt_enable_disable interrupt = BMM350_DISABLE_INTERRUPT);
        virtual int magneticEnableAxes(enum bmm350_x_axis_en_dis en_x = BMM350_X_EN, enum bmm350_y_axis_en_dis en_y = BMM350_Y_EN, enum bmm350_z_axis_en_dis en_z = BMM350_Z_EN);
        virtual int magneticSetThreshold(int8_t threshold, enum bmm350_intr_polarity polarity);
        virtual bmm350_threshold_data_t magneticGetThreshold();

    protected:
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

        uint8_t bmi270_gyr_odr                                = BMI2_GYR_ODR_100HZ;
        uint8_t bmi270_acc_odr                                = BMI2_ACC_ODR_100HZ;
        uint8_t bmi270_aux_odr                                = BMI2_AUX_ODR_100HZ;

       
        struct bmm350_mag_temp_data magnetic_threshold;
        enum bmm350_performance_parameters bmm350_performance = BMM350_REGULARPOWER;           /*! Power mode of sensor */
        enum bmm350_data_rates bmm350_data_rate               = BMM350_DATA_RATE_400HZ;        /*! Data rate value (ODR) */
        enum bmm350_power_modes bmm350_pwr_mode               = BMM350_NORMAL_MODE;            /*! Preset mode of sensor */
        enum bmm350_interrupt_enable_disable bmm350_interrupt = BMM350_DISABLE_INTERRUPT;
        int8_t bmm350_threshold = 0;
        bmm350_threshold_data_t bmm350_threshold_tmp;
};


extern BoschSensorClassPSoC IMU_BMI270_BMM350;
#undef IMU_PSoC
#define IMU_PSoC IMU_BMI270_BMM350

