# Bosch Sensor Class for PSoC Kits

This library is specially designed to run with the Bosch Sensors on PSoC Kits like the CY8CKIT-062S2-AI under PSoC for Arduino
which includes the following sensors
- the Bosch Sensortec BMI270 IMU for gyroscope and accelerometer data
- the Bosch Sensortec BMM250 magnetometer 
Both sensors are attached to the internal I2C bus of the PSoC Kit, therefore:

| Board               | sensor    | pin    | purpose    |
|:--------------------|:----------|:-------|:-----------|
| CY8CKIT-062S2-AI    | BMI270    | P0_2   | I2C SCL    |
|                     |           | P0_3   | I2C SDA    |
|                     |           | P1_5   | INT1       |
|                     |           | P0_4   | INT2       |
|                     | BMM350    | P0_2   | I2C SCL    |
|                     |           | P0_3   | I2C SDA    |
|                     |           | P1_0   | INT        |




## Links

- [for the CY8CKIT-062S2-AI](https://www.infineon.com/cms/en/product/evaluation-boards/cy8ckit-062s2-ai/?redirId=273839)
- [PSoC for Arduino](https://github.com/Infineon/arduino-core-psoc6)
- [PSOc for Arduino documentation](https://arduino-core-psoc6.readthedocs.io/en/latest/hw-platforms.html)