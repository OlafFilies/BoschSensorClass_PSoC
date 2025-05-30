/**
 * @file BoschSensorClassPSoC.cpp
 * @brief
 * @date 2025-05-21
 *
 *
 */


#include "Arduino.h"
#include "Wire.h"
#include "BoschSensorClass_PSoC.hpp"


// default range is +-2000dps, so conversion factor is (((1 << 15)/4.0f))
#define INT16_to_G   (8192.0f)

// default range is +-2000dps, so conversion factor is (((1 << 15)/4.0f))
#define INT16_to_DPS   (16.384f)


void BoschSensorClassPSoC::debug(Stream& stream)
{
  _debug = &stream;
}


/**
 * @brief Construct a new Bosch Sensor Class P So C:: Bosch Sensor Class P So C object
 * 
 * @param wire 
 */
BoschSensorClassPSoC::BoschSensorClassPSoC(TwoWire& wire)
{
  _wire = &wire;
}

/**
 * @brief Begin function configures one or both sensors with default settings
 * and tries to call them on the I2C line
 * 
 * @verbatim
      cfg                   |
 ---------------------------|-----------------------------------------
   BOSCH_ACCELEROMETER_ONLY | only the BMI270 accelerometer/girometer 
   BOSCH_MAGNETOMETER_ONLY  | only the BMM350 magnetometer
   BOSCH_ACCEL_AND_MAGN     | BMI270 and BMM350
 * @endverbatim
 * @return int
 * @param cfg which sensor should be connected
 * @return int 
 */
int BoschSensorClassPSoC::begin(CfgBoshSensor_t cfg)
{
    _wire->begin();

    bmi270.chip_id = BMI2_I2C_PRIM_ADDR;
    bmi270.read = bosch_i2c_read;
    bmi270.write = bosch_i2c_write;
    bmi270.delay_us = delay_us;
    bmi270.intf = BMI2_I2C_INTF;
    bmi270.intf_ptr = &accel_gyro_dev_info;
    bmi270.read_write_len = 30; // Limitation of the Wire library
    bmi270.config_file_ptr = NULL; // Use the default BMI270 config file

    accel_gyro_dev_info._wire = _wire;
    accel_gyro_dev_info.dev_addr = bmi270.chip_id;


    bmm350.chip_id = BMM350_I2C_ADSEL_SET_HIGH;
    bmm350.read = bosch_i2c_read;
    bmm350.write = bosch_i2c_write;
    bmm350.delay_us = delay_us;
    bmm350.intf_ptr = &mag_dev_info;
    bmm350.axis_en = BMM350_ENABLE;
    // bmm350.intf = BMM350_INTF_RET_TYPE;
    // mag_comp
    // otp_data
    // var_idm
    // raw_override


    mag_dev_info._wire = _wire;
    mag_dev_info.dev_addr = bmm350.chip_id;

    int8_t result = 0;

    if(cfg != BOSCH_MAGNETOMETER_ONLY) {

        result  |= bmi270_init(&bmi270);
        bmi270_print_rslt(result);

        result  |= configure_sensor(&bmi270);
        bmi270_print_rslt(result);
    }

    if(cfg != BOSCH_ACCELEROMETER_ONLY) {

Serial.println("BMM INI IN");
        result |= bmm350_init(&bmm350);
        bmm350_print_rslt(result);
Serial.println(result);

Serial.println("BMM CONF IN");
        result = configure_sensor(&bmm350);
        bmm350_print_rslt(result);
Serial.println("BMM CONF OUT");
    }

    result = 0;
    _initialized = (result == 0);

    return _initialized;
}

/**
 * @brief 
 * 
 */
void BoschSensorClassPSoC::setContinuousMode() {
  bmi2_set_fifo_config(BMI2_FIFO_GYR_EN | BMI2_FIFO_ACC_EN, 1, &bmi270);
  continuousMode = true;
}

/**
 * @brief 
 * 
 */
void BoschSensorClassPSoC::oneShotMode() {
  bmi2_set_fifo_config(BMI2_FIFO_GYR_EN | BMI2_FIFO_ACC_EN, 0, &bmi270);
  continuousMode = false;
}

/**
 * @brief 
 * 
 * @param dev 
 * @return int8_t 
 */
int8_t BoschSensorClassPSoC::configure_sensor(struct bmi2_dev *dev)
{
  int8_t rslt;
  uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_GYRO };

  struct bmi2_int_pin_config int_pin_cfg;
  int_pin_cfg.pin_type = BMI2_INT1;
  int_pin_cfg.int_latch = BMI2_INT_NON_LATCH;
  int_pin_cfg.pin_cfg[0].lvl = BMI2_INT_ACTIVE_HIGH;
  int_pin_cfg.pin_cfg[0].od = BMI2_INT_PUSH_PULL;
  int_pin_cfg.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
  int_pin_cfg.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE;

  struct bmi2_sens_config sens_cfg[2];
  sens_cfg[0].type = BMI2_ACCEL;
  sens_cfg[0].cfg.acc.bwp = BMI2_ACC_OSR2_AVG2;
  sens_cfg[0].cfg.acc.odr = BMI2_ACC_ODR_100HZ;
  sens_cfg[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
  sens_cfg[0].cfg.acc.range = BMI2_ACC_RANGE_4G;
  sens_cfg[1].type = BMI2_GYRO;
  sens_cfg[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
  sens_cfg[1].cfg.gyr.bwp = BMI2_GYR_OSR2_MODE;
  sens_cfg[1].cfg.gyr.odr = BMI2_GYR_ODR_100HZ;
  sens_cfg[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
  sens_cfg[1].cfg.gyr.ois_range = BMI2_GYR_OIS_2000;

  rslt = bmi2_set_int_pin_config(&int_pin_cfg, dev);
  if (rslt != BMI2_OK)
    return rslt;

  rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, dev);
  if (rslt != BMI2_OK)
    return rslt;

  rslt = bmi2_set_sensor_config(sens_cfg, 2, dev);
  if (rslt != BMI2_OK)
    return rslt;

  rslt = bmi2_sensor_enable(sens_list, 2, dev);
  if (rslt != BMI2_OK)
    return rslt;

  return rslt;
}

/**
 * @brief 
 * As this lib is for PSoC for Arduino, we do not need a setup for any other mcu
 * 
 * @param x 
 * @param y 
 * @param z 
 * @return int 
 */
int BoschSensorClassPSoC::readAcceleration(float& x, float& y, float& z) {
    struct bmi2_sens_data sensor_data;
    auto ret = bmi2_get_sensor_data(&sensor_data, &bmi270);
    x = sensor_data.acc.x / INT16_to_G;
    y = sensor_data.acc.y / INT16_to_G;
    z = sensor_data.acc.z / INT16_to_G;
    return (ret == 0);
}

/**
 * @brief 
 * 
 * @return int 
 */
int BoschSensorClassPSoC::accelerationAvailable() {
    uint16_t status;
    bmi2_get_int_status(&status, &bmi270);
    int ret = ((status | _int_status) & BMI2_ACC_DRDY_INT_MASK);
    _int_status = status;
    _int_status &= ~BMI2_ACC_DRDY_INT_MASK;
    return ret;
}

/**
 * @brief 
 * 
 * @return float 
 */
float BoschSensorClassPSoC::accelerationSampleRate() {
    struct bmi2_sens_config sens_cfg;
    sens_cfg.type = BMI2_ACCEL;
    bmi2_get_sensor_config(&sens_cfg, 1, &bmi270);
    return (1 << sens_cfg.cfg.acc.odr) * 0.39;
}



/**
 * @brief 
 * As this lib is for PSoC for Arduino, we do not need a setup for any other mcu
 * 
 * @param x 
 * @param y 
 * @param z 
 * @return int 
 */
int BoschSensorClassPSoC::readGyroscope(float& x, float& y, float& z) {
    struct bmi2_sens_data sensor_data;
    auto ret = bmi2_get_sensor_data(&sensor_data, &bmi270);
    x = sensor_data.gyr.x / INT16_to_DPS;
    y = sensor_data.gyr.y / INT16_to_DPS;
    z = sensor_data.gyr.z / INT16_to_DPS;
    return (ret == 0);
}

/**
 * @brief 
 * 
 * @return int 
 */
int BoschSensorClassPSoC::gyroscopeAvailable() {
    uint16_t status;
    bmi2_get_int_status(&status, &bmi270);
    int ret = ((status | _int_status) & BMI2_GYR_DRDY_INT_MASK);
    _int_status = status;
    _int_status &= ~BMI2_GYR_DRDY_INT_MASK;
    return ret;
}

/**
 * @brief 
 * 
 * @return float 
 */
float BoschSensorClassPSoC::gyroscopeSampleRate() {
    struct bmi2_sens_config sens_cfg;
    sens_cfg.type = BMI2_GYRO;
    bmi2_get_sensor_config(&sens_cfg, 1, &bmi270);
    return (1 << sens_cfg.cfg.gyr.odr) * 0.39;
}




/**
 * @brief 
 * 
 * @return int 
 */
int BoschSensorClassPSoC::magneticFieldAvailable() {
    uint8_t status;
    int8_t rslt;
    //bmm350_get_interrupt_status(&status, &bmm350);
    rslt = bmm350_get_regs(BMM350_REG_INT_STATUS, &status, 1, &bmm350);

    Serial.println(status);
    Serial.println(rslt);

    return status;
}

/**
 * @brief 
 * 
 * @param x 
 * @param y 
 * @param z 
 * @return int 
 */
int BoschSensorClassPSoC::readMagneticField(float& x, float& y, float& z) {
  struct bmm350_mag_temp_data mag_data;
  int const rc = bmm350_get_compensated_mag_xyz_temp_data(&mag_data, &bmm350);
  x = mag_data.x;
  y = mag_data.y;
  z = mag_data.z;

  if (rc == BMM350_OK)
    return 1;
  else
    return 0;
}

/**
 * @brief 
 * 
 * @param rate 
 * @verbatim
      Data rate (ODR)     |        odr
 -------------------------|-----------------------
   400Hz                  |  BMM350_DATA_RATE_400HZ
   200Hz                  |  BMM350_DATA_RATE_200HZ
   100Hz                  |  BMM350_DATA_RATE_100HZ
   50Hz                   |  BMM350_DATA_RATE_50HZ
   25Hz                   |  BMM350_DATA_RATE_25HZ
   12.5Hz                 |  BMM350_DATA_RATE_12_5HZ
   6.25Hz                 |  BMM350_DATA_RATE_6_25HZ
   3.125Hz                |  BMM350_DATA_RATE_3_125HZ
   1.5625Hz               |  BMM350_DATA_RATE_1_5625HZ
 * @endverbatim
 * @return int
 */
float BoschSensorClassPSoC::magneticFieldSampleRate(enum bmm350_data_rates rate) {
    bmm350_data_rate = rate;
    switch (bmm350_data_rate){
        case BMM350_DATA_RATE_400HZ:
            return 400;
        case BMM350_DATA_RATE_200HZ:
            return 200;
        case BMM350_DATA_RATE_100HZ:
            return 100;
        case BMM350_DATA_RATE_50HZ:
            return 50;
        case BMM350_DATA_RATE_25HZ:
            return 25;
        case BMM350_DATA_RATE_12_5HZ:
            return 12.5;
        case BMM350_DATA_RATE_6_25HZ:
            return 6.25;
        case BMM350_DATA_RATE_3_125HZ:
            return 3.125;
        case BMM350_DATA_RATE_1_5625HZ:
            return 1.5625;
            break;
        default:
            return 0;
    }
}

/**
 * @brief 
 * 
 * @param noise 
 * @verbatim
     avg                    |   averaging factor            alias
 ---------------------------|------------------------------------------
   low power/highest noise  |  BMM350_NO_AVERAGING  BMM350_LOWPOWER
   lesser noise             |  BMM350_AVERAGING_2   BMM350_REGULARPOWER
   even lesser noise        |  BMM350_AVERAGING_4   BMM350_LOWNOISE
 lowest noise/highest power |  BMM350_AVERAGING_8   BMM350_ULTRALOWNOISE
 * @endverbatim
 
 * @return int 
 */
int BoschSensorClassPSoC::magneticPerformanceMode(enum bmm350_performance_parameters performance) {

    bmm350_performance = performance;
    return bmm350_performance;
}

/**
 * @brief 
 * 
 * @param power
 * @verbatim
                powermode |   Power mode
 -------------------------|-----------------------
                          |  BMM350_SUSPEND_MODE
                          |  BMM350_NORMAL_MODE
                          |  BMM350_FORCED_MODE
                          |  BMM350_FORCED_MODE_FAST
 * @endverbatim
 * @return int 
 */
int BoschSensorClassPSoC::magneticPowerMode(enum bmm350_power_modes power){
    bmm350_pwr_mode = power;
    return bmm350_set_powermode(bmm350_pwr_mode, &bmm350);
}

/**
 * @brief 
 * 
 * @param interrupt 
 * @return int 
 */
int BoschSensorClassPSoC::magneticInterruptMode(enum bmm350_interrupt_enable_disable interrupt){
    bmm350_interrupt = interrupt;
    return bmm350_enable_interrupt(bmm350_interrupt, &bmm350);
}


/** 
 * @brief 
 * 
 * @param dev 
 * @return int8_t 
 */
int8_t BoschSensorClassPSoC::configure_sensor(struct bmm350_dev *dev)
{
    int8_t rslt;

    magneticFieldSampleRate(bmm350_data_rate);
    magneticPerformanceMode(bmm350_performance);

    //  set performance
    rslt = bmm350_set_odr_performance(
        bmm350_data_rate,
        bmm350_performance,
        dev
    );
    if (rslt == BMM350_OK)
    {
        rslt = magneticPowerMode(bmm350_pwr_mode);
        rslt = magneticInterruptMode(bmm350_interrupt);
    }



    return rslt;
}




/**
 * @brief 
 * 
 * @param reg_addr 
 * @param reg_data 
 * @param len 
 * @param intf_ptr 
 * @return int8_t 
 */
int8_t BoschSensorClassPSoC::bosch_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    if ((reg_data == NULL) || (len == 0) || (len > 32))
    {
        return -1;
    }
    uint8_t bytes_received;

    struct dev_info *dev_info = (struct dev_info *)intf_ptr;
    uint8_t dev_id = dev_info->dev_addr;

    dev_info->_wire->beginTransmission(dev_id);
    dev_info->_wire->write(reg_addr);
    if (dev_info->_wire->endTransmission() == 0)
    {
        bytes_received = dev_info->_wire->requestFrom(dev_id, len);
        // Optionally, throw an error if bytes_received != len
        for (uint16_t i = 0; i < bytes_received; i++)
        {
            reg_data[i] = dev_info->_wire->read();
        }
    }
    else
    {
        return -1;
    }

    return 0;
}

/**
 * @brief 
 * 
 * @param reg_addr 
 * @param reg_data 
 * @param len 
 * @param intf_ptr 
 * @return int8_t 
 */
int8_t BoschSensorClassPSoC::bosch_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    if ((reg_data == NULL) || (len == 0) || (len > 32))
    {
        return -1;
    }

    struct dev_info *dev_info = (struct dev_info *)intf_ptr;
    uint8_t dev_id = dev_info->dev_addr;
    dev_info->_wire->beginTransmission(dev_id);
    dev_info->_wire->write(reg_addr);
    for (uint16_t i = 0; i < len; i++)
    {
        dev_info->_wire->write(reg_data[i]);
    }
    if (dev_info->_wire->endTransmission() != 0)
    {
        return -1;
    }

    return 0;
}

/**
 * @brief 
 * 
 * @param period 
 * @param intf_ptr 
 */
void BoschSensorClassPSoC::delay_us(uint32_t period, void *intf_ptr)
{
    delayMicroseconds(period);
}

/**
 * @brief 
 * 
 */
void BoschSensorClassPSoC::panic_led_trap(const char* errhead, const char* errtext,int8_t rslt)
{
    Serial.print(errhead);
    Serial.print(rslt);
    Serial.print("] : ");
    Serial.println(errtext);

    int panicStop = 0;
    pinMode(LED_BUILTIN, OUTPUT);
    while (1 && panicStop<10)
    {
        digitalWrite(LED_BUILTIN, LOW);
        delay(100);
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
        digitalWrite(LED_BUILTIN, LOW);
        panicStop++;
    }
}

/**
 * @brief 
 * 
 * @param rslt 
 */
void BoschSensorClassPSoC::bmi270_print_rslt(int8_t rslt)
{
    if (!_debug)
    {
        return;
    } else {
        switch (rslt)
        {
            case BMI2_OK:
                return;
            case BMI2_E_NULL_PTR:
                panic_led_trap(" BMI270 Error ["," : Null pointer",rslt);
                break;
            case BMI2_E_COM_FAIL:
                panic_led_trap(" BMI270 Error ["," : Communication fail",rslt);
                break;
            case BMI2_E_DEV_NOT_FOUND:
                panic_led_trap(" BMI270 Error ["," : Device not found",rslt);
                break;
            case BMI2_E_OUT_OF_RANGE:
                panic_led_trap(" BMI270 Error ["," : Out of range",rslt);
                break;
            case BMI2_E_ACC_INVALID_CFG:
                panic_led_trap(" BMI270 Error ["," : Invalid accel configuration",rslt);
                break;
            case BMI2_E_GYRO_INVALID_CFG:
                panic_led_trap(" BMI270 Error ["," : Invalid gyro configuration",rslt);
                break;
            case BMI2_E_ACC_GYR_INVALID_CFG:
                panic_led_trap(" BMI270 Error ["," : Invalid accel/gyro configuration",rslt);
                break;
            case BMI2_E_INVALID_SENSOR:
                panic_led_trap(" BMI270 Error ["," : Invalid sensor",rslt);
                break;
            case BMI2_E_CONFIG_LOAD:
                panic_led_trap(" BMI270 Error ["," : Configuration loading error",rslt);
                break;
            case BMI2_E_INVALID_PAGE:
                panic_led_trap(" BMI270 Error ["," : Invalid page ",rslt);
                break;
            case BMI2_E_INVALID_FEAT_BIT:
                panic_led_trap(" BMI270 Error ["," : Invalid feature bit",rslt);
                break;
            case BMI2_E_INVALID_INT_PIN:
                panic_led_trap(" BMI270 Error ["," : Invalid interrupt pin",rslt);
                break;
            case BMI2_E_SET_APS_FAIL:
                panic_led_trap(" BMI270 Error ["," : Setting advanced power mode failed",rslt);
                break;
            case BMI2_E_AUX_INVALID_CFG:
                panic_led_trap(" BMI270 Error ["," : Invalid auxiliary configuration",rslt);
                break;
            case BMI2_E_AUX_BUSY:
                panic_led_trap(" BMI270 Error ["," : Auxiliary busy",rslt);
                break;
            case BMI2_E_SELF_TEST_FAIL:
                panic_led_trap(" BMI270 Error ["," : Self test failed",rslt);
                break;
            case BMI2_E_REMAP_ERROR:
                panic_led_trap(" BMI270 Error ["," : Remapping error",rslt);
                break;
            case BMI2_E_GYR_USER_GAIN_UPD_FAIL:
                panic_led_trap(" BMI270 Error ["," : Gyro user gain update failed",rslt);
                break;
            case BMI2_E_SELF_TEST_NOT_DONE:
                panic_led_trap(" BMI270 Error ["," : Self test not done",rslt);
                break;
            case BMI2_E_INVALID_INPUT:
                panic_led_trap(" BMI270 Error ["," : Invalid input",rslt);
                break;
            case BMI2_E_INVALID_STATUS:
                panic_led_trap(" BMI270 Error ["," : Invalid status",rslt);
                break;
            case BMI2_E_CRT_ERROR:
                panic_led_trap(" BMI270 Error ["," : CRT error",rslt);
                break;
            case BMI2_E_ST_ALREADY_RUNNING:
                panic_led_trap(" BMI270 Error ["," : Self test already running",rslt);
                break;
            case BMI2_E_CRT_READY_FOR_DL_FAIL_ABORT:
                panic_led_trap(" BMI270 Error ["," : CRT ready for DL fail abort",rslt);
                break;
            case BMI2_E_DL_ERROR:
                panic_led_trap(" BMI270 Error ["," : DL error",rslt);
                break;
            case BMI2_E_PRECON_ERROR:
                panic_led_trap(" BMI270 Error ["," : PRECON error",rslt);
                break;
            case BMI2_E_ABORT_ERROR:
                panic_led_trap(" BMI270 Error ["," : Abort error",rslt);
                break;
            case BMI2_E_GYRO_SELF_TEST_ERROR:
                panic_led_trap(" BMI270 Error ["," : Gyro self test error",rslt);
                break;
            case BMI2_E_GYRO_SELF_TEST_TIMEOUT:
                panic_led_trap(" BMI270 Error ["," : Gyro self test timeout",rslt);
                break;
            case BMI2_E_WRITE_CYCLE_ONGOING:
                panic_led_trap(" BMI270 Error ["," : Write cycle ongoing",rslt);
                break;
            case BMI2_E_WRITE_CYCLE_TIMEOUT:
                panic_led_trap(" BMI270 Error ["," : Write cycle timeout",rslt);
                break;
            case BMI2_E_ST_NOT_RUNING:
                panic_led_trap(" BMI270 Error ["," : Self test not running",rslt);
                break;
            case BMI2_E_DATA_RDY_INT_FAILED:
                panic_led_trap(" BMI270 Error ["," : Data ready interrupt failed",rslt);
                break;
            case BMI2_E_INVALID_FOC_POSITION:
                panic_led_trap(" BMI270 Error ["," : Invalid FOC position",rslt);
                break;
            default:
                panic_led_trap(" BMI270 Error ["," : Unknown error code",rslt);
                break;
        }
    }
}

/**
 * @brief 
 * 
 * @param rslt 
 */
void BoschSensorClassPSoC::bmm350_print_rslt(int8_t rslt)
{
    if (!_debug)
    {
        return;
    } else {
        switch (rslt)
        {
            case BMM350_OK:
                return;
            case BMM350_E_NULL_PTR:
                panic_led_trap(" BMM350 Error ["," : Null pointer",rslt);
                break;
            case BMM350_E_COM_FAIL:
                panic_led_trap(" BMM350 Error ["," : Communication fail",rslt);
                break;
            case BMM350_E_DEV_NOT_FOUND:
                panic_led_trap(" BMM350 Error ["," : Device not found",rslt);
                break;
            case BMM350_E_INVALID_CONFIG :
                panic_led_trap(" BMM350 Error ["," : Invalid magnetometer configuration",rslt);
                break;
            case BMM350_E_BAD_PAD_DRIVE:
                panic_led_trap(" BMM350 Error ["," : Bad pad drive",rslt);
                break;
            case BMM350_E_RESET_UNFINISHED:
                panic_led_trap(" BMM350 Error ["," : Reset unfinished",rslt);
                break;
            case BMM350_E_INVALID_INPUT:
                panic_led_trap(" BMM350 Error ["," : Invalid input",rslt);
                break;
            case BMM350_E_SELF_TEST_INVALID_AXIS:
                panic_led_trap(" BMM350 Error ["," : Self-test invalid axis selection",rslt);
                break;
            case BMM350_E_OTP_BOOT:
                panic_led_trap(" BMM350 Error ["," : OTP boot",rslt);
                break;
            case BMM350_E_OTP_PAGE_RD:
                panic_led_trap(" BMM350 Error ["," : OTP page read",rslt);
                break;
            case BMM350_E_OTP_PAGE_PRG:
                panic_led_trap(" BMM350 Error ["," : OTP page prog",rslt);
                break;
            case BMM350_E_OTP_SIGN:
                panic_led_trap(" BMM350 Error ["," : OTP sign",rslt);
                break;
            case BMM350_E_OTP_INV_CMD:
                panic_led_trap(" BMM350 Error ["," : OTP invalid command",rslt);
                break;
            case BMM350_E_OTP_UNDEFINED:
                panic_led_trap(" BMM350 Error ["," : OTP undefined",rslt);
                break;
            case BMM350_E_ALL_AXIS_DISABLED:
                panic_led_trap(" BMM350 Error ["," : All axis are disabled",rslt);
                break;
            case BMM350_E_PMU_CMD_VALUE:
                panic_led_trap(" BMM350 Error ["," : Unexpected PMU CMD value",rslt);
                break;
            default:
                panic_led_trap(" BMM350 Error ["," : Unknown error code",rslt);
                break;
        }
    }
}



BoschSensorClassPSoC IMU_BMI270_BMM350(Wire);
