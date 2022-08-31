/***********************************************************************************************//**
 * \file mtb_bmx160.h
 *
 * Description: This file is the public interface of the BMX160 orientation sensor.
 ***************************************************************************************************
 * \copyright
 * Copyright 2021 Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 **************************************************************************************************/

#pragma once

/**
 * \addtogroup group_board_libs Absolute Orientation Sensor
 * \{
 * Basic set of APIs for interacting with the BMX160 absolute orientation
 * sensor. This provides basic initialization and access to to the basic
 * accelerometer, gyroscope, and magnetometer data. It also provides access to
 * the base BMI160 & BMM150 driver for full control.
 * For more information about the motion sensor, see:
 * https://github.com/BoschSensortec/BMI160_driver
 * For more information about the magnetometer sensor, see:
 * https://github.com/BoschSensortec/BMM150-Sensor-API
 *
 * \note Currently, this library only supports being used for a single instance
 * of this device.
 *
 * \note BMX160 support requires delays. If the RTOS_AWARE component is set or
 * CY_RTOS_AWARE is defined, the HAL driver will defer to the RTOS for delays.
 * Because of this, it is not safe to call any functions other than
 * \ref mtb_bmx160_init_i2c until after the RTOS scheduler has started.
 *
 * \note There is a known issue with the BMI160 endianness detection. Any code
 * referencing the structures defined in the BMI160 driver should have this header
 * file, mtb_bmx160.h, first in any includes.
 *
 * \section subsection_board_libs_snippets Code snippets
 * \subsection subsection_board_libs_snippet_1 Snippet 1: Simple initialization with I2C.
 * The following snippet initializes an I2C instance and the BMX160, then reads
 * from the BMX160.
 * \snippet mtb_bmx160_example.c snippet_bmx160_i2c_init
 *
 * \subsection subsection_board_libs_snippet_2 Snippet 2: BMX160 interrupt configuration.
 * The following snippet demonstrates how to configure a BMX160 interrupt.
 * \snippet mtb_bmx160_example.c snippet_bmx160_configure_interrupt
 */

#include "bmi160.h"
#include "bmm150.h"
#include "cy_result.h"
#include "cyhal_gpio.h"
#include "cyhal_i2c.h"
#include "cyhal_spi.h"

#if defined(__cplusplus)
extern "C"
{
#endif

/** \cond INTERNAL */
#if (CYHAL_API_VERSION >= 2)
typedef cyhal_gpio_callback_data_t  _mtb_bmx160_interrupt_pin_t;
#else
typedef cyhal_gpio_t                _mtb_bmx160_interrupt_pin_t;
#endif
/** \endcond */

/**
 * Structure holding the sensor instance specific information.
 *
 * Application code should not rely on the specific content of this struct.
 * They are considered an implementation detail which is subject to change
 * between platforms and/or library releases.
 */
typedef struct
{
    struct bmi160_dev           sensor1;
    struct bmm150_dev           sensor2;
    _mtb_bmx160_interrupt_pin_t intpin1;
    _mtb_bmx160_interrupt_pin_t intpin2;
} mtb_bmx160_t;

/** Structure holding the accelerometer, gyroscope, and magnetometer data read from the device. */
typedef struct
{
    /** Accelerometer data */
    struct bmi160_sensor_data accel;
    /** Gyroscope data */
    struct bmi160_sensor_data gyro;
    /** Magnetometer data */
    struct bmm150_mag_data    mag;
} mtb_bmx160_data_t;

/**
 * Enumeration used for selecting I2C address. Two addresses are needed. One for the motion
 * sensor data and one for the geomagetic sensor.
 */
typedef enum
{
    /** Use the default addresses for the sensors (0x68) */
    MTB_BMX160_ADDRESS_DEFAULT  = ((BMI160_I2C_ADDR << 8) | BMM150_DEFAULT_I2C_ADDRESS),
    /** Use 0x68 for the motion sensor and 0x10 for the geomagetic sensor */
    MTB_BMX160_ADDRESS_68_10    = ((BMI160_I2C_ADDR << 8) | BMM150_DEFAULT_I2C_ADDRESS),
    /** Use 0x68 for the motion sensor and 0x12 for the geomagetic sensor */
    MTB_BMX160_ADDRESS_68_12    = ((BMI160_I2C_ADDR << 8) | BMM150_I2C_ADDRESS_CSB_HIGH_SDO_LOW),
    /** Use 0x69 for the motion sensor and 0x11 for the geomagetic sensor */
    MTB_BMX160_ADDRESS_69_11    = ((0x69 << 8) | BMM150_I2C_ADDRESS_CSB_LOW_SDO_HIGH),
    /** Use 0x69 for the motion sensor and 0x13 for the geomagetic sensor */
    MTB_BMX160_ADDRESS_69_13    = ((0x69 << 8) | BMM150_I2C_ADDRESS_CSB_HIGH_SDO_HIGH)
} mtb_bmx160_address_t;

/** An attempt was made to configure too many gpio pins as interrupts. */
#define MTB_BMX160_RSLT_ERR_INSUFFICIENT_INT_PINS                 \
    (CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR, CY_RSLT_MODULE_BOARD_HARDWARE_BMI160, 0x200))

/**
 * Initialize the sensor for I2C communication. Then applies the default
 * configuration settings for the accelerometer, gyroscope and magnetometer. Known
 * maximum I2C frequency of 1MHz; refer to manufacturer's datasheet for confirmation.
 * See: \ref mtb_bmx160_config_default()
 * @param[in] obj       Pointer to a BMX160 object. The caller must allocate the memory
 *  for this object but the init function will initialize its contents.
 * @param[in] inst      I2C instance to use for communicating with the BMX160 sensor.
 * @param[in] address   BMX160 I2C address, set by hardware implementation.
 * @return CY_RSLT_SUCCESS if properly initialized, else an error indicating what went wrong.
 */
cy_rslt_t mtb_bmx160_init_i2c(mtb_bmx160_t* obj, cyhal_i2c_t* inst, mtb_bmx160_address_t address);

/**
 * Initialize the sensor for SPI communication. Then applies the default configuration settings
 * for the accelerometer, gyroscope and magnetometer. NOTE: The SPI slave select pin (\p spi_ss)
 * is expected to be controlled by this driver, not the SPI block itself. This allows the driver
 * to issue multiple read/write requests back to back.
 * See: \ref mtb_bmx160_config_default()
 * @param[in] obj       Pointer to a BMX160 object. The caller must allocate the memory
 *  for this object but the init function will initialize its contents.
 * @param[in] inst      SPI instance to use for communicating with the BMX160 sensor.
 * @param[in] spi_ss    SPI slave select pin to use for communicating with the BMI160 sensor.
 * @return CY_RSLT_SUCCESS if properly initialized, else an error indicating what went wrong.
 */
cy_rslt_t mtb_bmx160_init_spi(mtb_bmx160_t* obj, cyhal_spi_t* inst, cyhal_gpio_t spi_ss);

/**
 * Configure the sensor to a default mode with accelerometer, gyroscope, & magnetometer
 * enabled with a nominal output data rate. The default values used are from the example in the
 * BMI160 driver repository, see https://github.com/BoschSensortec/BMI160_driver
 * @param[in] obj  Pointer to a BMX160 object.
 * @return CY_RSLT_SUCCESS if properly initialized, else an error indicating what went wrong.
 */
cy_rslt_t mtb_bmx160_config_default(mtb_bmx160_t* obj);

/**
 * Reads the current accelerometer, gyroscope, and magnetometer data from the sensor.
 * @param[in] obj  Pointer to a BMX160 object.
 * @param[out] sensor_data The accelerometer, gyroscope, and magnetometer data read from the sensor
 * @return CY_RSLT_SUCCESS if properly initialized, else an error indicating what went wrong.
 */
cy_rslt_t mtb_bmx160_read(mtb_bmx160_t* obj, mtb_bmx160_data_t* sensor_data);

/**
 * Gets access to the base motion sensor data. This allows for direct manipulation of the
 * sensor for any desired behavior. See https://github.com/BoschSensortec/BMI160_driver for
 * more details on the sensor.
 * @param[in] obj  Pointer to a BMX160 object.
 * @return pointer to the BMI160 configuration structure.
 */
struct bmi160_dev* mtb_bmx160_get_motion(mtb_bmx160_t* obj);

/**
 * Gets access to the base geomagnetic sensor data. This allows for direct manipulation of the
 * sensor for any desired behavior. See https://github.com/BoschSensortec/BMM150-Sensor-API for
 * more details on the sensor.
 * @param[in] obj  Pointer to a BMX160 object.
 * @return pointer to the BMI160 configuration structure.
 */
struct bmm150_dev* mtb_bmx160_get_geo(mtb_bmx160_t* obj);

/**
 * Performs both motion and geomagnetic self tests. Note these tests cause a soft reset
 * of the device and device should be reconfigured after a test.
 * See https://github.com/BoschSensortec/BMI160_driver and
 * https://github.com/BoschSensortec/BMM150-Sensor-API for more details.
 * @param[in] obj  Pointer to a BMX160 object.
 * @return CY_RSLT_SUCCESS if self tests pass, else an error indicating what went wrong.
 */
cy_rslt_t mtb_bmx160_selftest(mtb_bmx160_t* obj);

/**
 * Configure a GPIO pin as an interrupt for the BMX160.
 * This configures the pin as an interrupt, and calls the BMX160 interrupt configuration API
 * with the application supplied settings structure.
 * See https://github.com/BoschSensortec/BMI160_driver for more details.
 * @param[in] obj           Pointer to a BMX160 object.
 * @param[in] intsettings   Pointer to a BMX160 interrupt settings structure.
 * @param[in] pin           Which pin to configure as interrupt
 * @param[in] intr_priority The priority for NVIC interrupt events
 * @param[in] event         The type of interrupt event
 * @param[in] callback      The function to call when the specified event happens. Pass NULL to
 *                          unregister the handler.
 * @param[in] callback_arg  Generic argument that will be provided to the callback when called, can
 *                          be NULL
 * @return CY_RSLT_SUCCESS if interrupt was successfully enabled.
 */
cy_rslt_t mtb_bmx160_config_int(mtb_bmx160_t* obj, struct bmi160_int_settg* intsettings,
                                cyhal_gpio_t pin, uint8_t intr_priority, cyhal_gpio_event_t event,
                                cyhal_gpio_event_callback_t callback, void* callback_arg);

/**
 * Frees up any resources allocated by the motion_sensor as part of \ref mtb_bmx160_init_i2c().
 * @param[in] obj  Pointer to a BMX160 object.
 */
void mtb_bmx160_free(mtb_bmx160_t* obj);

#if defined(__cplusplus)
}
#endif

/** \} group_board_libs */
