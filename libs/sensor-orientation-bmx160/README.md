# BMX-160 Absolute Orientation Sensor

### Overview

This library provides functions for interfacing with the BMX-160 I2C/SPI 16-bit Absolute Orientation Sensor with three axis accelerometer, three axis gyroscope, and three axis magnetometer as used on the CY8CKIT-028-SENSE shield.

NOTE: Bosch does not provide a driver for the BMX160, instead the supported flow is to use the BMI160 and BMM150 drivers together. Unfortunately, this requires a manual edit to the BMI160 driver source code to update the device ID. See the [Bosch community forum](https://community.bosch-sensortec.com/t5/MEMS-sensors-forum/BMX160-driver/m-p/6581) for details.

Data Sheet: https://www.bosch-sensortec.com/products/motion-sensors/absolute-orientation-sensors/bmx160/
GitHub Accelerometer & Gyroscope: https://github.com/BoschSensortec/BMI160_driver
GitHub Magnetometer: https://github.com/BoschSensortec/BMM150-Sensor-API

### Quick Start
Follow the steps below to create a simple application which outputs the
accelerometer, gyroscope, and magnetometer data from the sensor to the UART
1. Create an empty PSoC™ 6 application
2. Add this library to the application
3. Add retarget-io library using the Library Manager
4. Update the Device ID in the BMI160_driver as mentioned in the Overview
5. Place following code in the main.c file; defining IMU_I2C_SDA and IMU_I2C_SCL as appropriate for your hardware/shield kit
```cpp
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "mtb_bmx160.h"

mtb_bmx160_t motion_sensor;
cyhal_i2c_t i2c;
cyhal_i2c_cfg_t i2c_cfg = {
    .is_slave = false,
    .address = 0,
    .frequencyhal_hz = 400000
};

#define IMU_I2C_SDA (?) // Define me
#define IMU_I2C_SCL (?) // Define me

int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    /* Initialize i2c for motion sensor */
    result = cyhal_i2c_init(&i2c, IMU_I2C_SDA, IMU_I2C_SCL, NULL);
    CY_ASSERT(result == CY_RSLT_SUCCESS);
    result = cyhal_i2c_configure(&i2c, &i2c_cfg);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    /* Initialize motion sensor */
    result = mtb_bmx160_init_i2c(&motion_sensor, &i2c, MTB_BMX160_DEFAULT_ADDRESS);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    for (;;)
    {
        /* Get the accel, gyro, and mag data and print the results to the UART */
        mtb_bmx160_data_t data;
        mtb_bmx160_read(&motion_sensor, &data);

        printf("Accel: X:%6d Y:%6d Z:%6d\r\n", data.accel.x, data.accel.y, data.accel.z);
        printf("Gyro : X:%6d Y:%6d Z:%6d\r\n\r\n", data.gyro.x, data.gyro.y, data.gyro.z);
        printf("Mag  : X:%6d Y:%6d Z:%6d\r\n\r\n", data.mag.x, data.mag.y, data.mag.z);

        cyhal_system_delay_ms(1000);
    }
}
```
6. Build the application and program the kit.

### More information

* [API Reference Guide](https://infineon.github.io/sensor-motion-bmx160/html/index.html)
* [Cypress Semiconductor, an Infineon Technologies Company](http://www.cypress.com)
* [Infineon GitHub](https://github.com/infineon)
* [ModusToolbox™](https://www.cypress.com/products/modustoolbox-software-environment)
* [PSoC™ 6 Code Examples using ModusToolbox™ IDE](https://github.com/infineon/Code-Examples-for-ModusToolbox-Software)
* [ModusToolbox™ Software](https://github.com/Infineon/modustoolbox-software)
* [PSoC™ 6 Resources - KBA223067](https://community.cypress.com/docs/DOC-14644)

---
© Cypress Semiconductor Corporation (an Infineon company) or an affiliate of Cypress Semiconductor Corporation, 2021.
