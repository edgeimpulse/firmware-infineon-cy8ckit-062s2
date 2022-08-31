/***********************************************************************************************//**
 * \file mtb_bmx160.c
 *
 * Description: This file contains the functions for interacting with the
 *              absolute orientation sensor.
 *
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

#include "cy_utils.h"
#include "mtb_bmx160.h"
#include "cyhal_i2c.h"
#include "cyhal_system.h"

#if defined(__cplusplus)
extern "C"
{
#endif

#define I2C_TIMEOUT         10 // 10 msec
#define BMI160_ERROR(x)     \
    (CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR, CY_RSLT_MODULE_BOARD_HARDWARE_BMX160, x))
#define BMM150_ERROR(x)     \
    (CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR, CY_RSLT_MODULE_BOARD_HARDWARE_BMIX60, 0x100 | x))
#define I2C_WRITE_BUFFER_LENGTH   32
#define SOFT_RESET_DELAY_US       300

static cyhal_i2c_t* i2c = NULL;
static cyhal_spi_t* spi = NULL;
static cyhal_gpio_t spi_ssel = NC;

//--------------------------------------------------------------------------------------------------
// i2c_write_bytes
//--------------------------------------------------------------------------------------------------
static int8_t i2c_write_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint16_t len)
{
    CY_ASSERT((len + 1) < I2C_WRITE_BUFFER_LENGTH);
    uint8_t buf[I2C_WRITE_BUFFER_LENGTH];
    buf[0] = reg_addr;
    for (uint16_t i=0; i < len; i++)
    {
        buf[i+1] = data[i];
    }

    cy_rslt_t result = cyhal_i2c_master_write(i2c, dev_addr, buf, len+1, I2C_TIMEOUT, true);

    return (CY_RSLT_SUCCESS == result)
        ? BMI160_OK
        : BMI160_E_COM_FAIL;
}


//--------------------------------------------------------------------------------------------------
// i2c_read_bytes
//--------------------------------------------------------------------------------------------------
static int8_t i2c_read_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint16_t len)
{
    cy_rslt_t result = cyhal_i2c_master_write(i2c, dev_addr, &reg_addr, 1, I2C_TIMEOUT, false);

    if (CY_RSLT_SUCCESS == result)
    {
        result = cyhal_i2c_master_read(i2c, dev_addr, data, len, I2C_TIMEOUT, true);
    }

    return (CY_RSLT_SUCCESS == result)
        ? BMI160_OK
        : BMI160_E_COM_FAIL;
}


//--------------------------------------------------------------------------------------------------
// bmm150_aux_read
//--------------------------------------------------------------------------------------------------
static int8_t bmm150_aux_read(uint8_t reg_addr, uint8_t* reg_data, uint32_t len, void* intf_ptr)
{
    return bmi160_aux_read(reg_addr, reg_data, len, (struct bmi160_dev*)(intf_ptr));
}


//--------------------------------------------------------------------------------------------------
// bmm150_aux_write
//--------------------------------------------------------------------------------------------------
static int8_t bmm150_aux_write(uint8_t reg_addr, uint8_t* reg_data, uint32_t len, void* intf_ptr)
{
    return bmi160_aux_write(reg_addr, reg_data, len, (struct bmi160_dev*)(intf_ptr));
}


//--------------------------------------------------------------------------------------------------
// spi_write_bytes
//--------------------------------------------------------------------------------------------------
static int8_t spi_write_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint16_t len)
{
    CY_UNUSED_PARAMETER(dev_addr);
    cy_rslt_t result = CY_RSLT_SUCCESS;

    cyhal_gpio_write(spi_ssel, 0);
    result |= cyhal_spi_send(spi, reg_addr);

    for (uint16_t i = 0; i < len; i++)
    {
        result |= cyhal_spi_send(spi, data[i]);
    }
    cyhal_gpio_write(spi_ssel, 1);

    return (CY_RSLT_SUCCESS == result)
        ? BMI160_OK
        : BMI160_E_COM_FAIL;
}


//--------------------------------------------------------------------------------------------------
// spi_read_bytes
//--------------------------------------------------------------------------------------------------
static int8_t spi_read_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint16_t len)
{
    CY_UNUSED_PARAMETER(dev_addr);
    cy_rslt_t result = CY_RSLT_SUCCESS;
    uint8_t value = reg_addr | 0x80;

    cyhal_gpio_write(spi_ssel, 0);
    result |= cyhal_spi_send(spi, value);

    for (uint16_t i = 0; i < len; i++)
    {
        uint32_t val;
        result |= cyhal_spi_recv(spi, &val);
        data[i] = (uint8_t)val;
    }
    cyhal_gpio_write(spi_ssel, 1);

    return (CY_RSLT_SUCCESS == result)
        ? BMI160_OK
        : BMI160_E_COM_FAIL;
}


//--------------------------------------------------------------------------------------------------
// delay_wrapper
//--------------------------------------------------------------------------------------------------
static void delay_ms_wrapper(uint32_t ms)
{
    (void)cyhal_system_delay_ms(ms);
}


//--------------------------------------------------------------------------------------------------
// delay_us_wrapper
//--------------------------------------------------------------------------------------------------
static void delay_us_wrapper(uint32_t period, void* intf_ptr)
{
    CY_UNUSED_PARAMETER(intf_ptr);
    cyhal_system_delay_us(period);
}


//--------------------------------------------------------------------------------------------------
// _mtb_bmx160_pins_equal
//--------------------------------------------------------------------------------------------------
static inline bool _mtb_bmx160_pins_equal(_mtb_bmx160_interrupt_pin_t ref_pin, cyhal_gpio_t pin)
{
    #if (CYHAL_API_VERSION >= 2)
    return (ref_pin.pin == pin);
    #else
    return (ref_pin == pin);
    #endif
}


//--------------------------------------------------------------------------------------------------
// _mtb_bmx160_set_pin
//--------------------------------------------------------------------------------------------------
static inline void _mtb_bmx160_set_pin(_mtb_bmx160_interrupt_pin_t* ref_pin, cyhal_gpio_t pin)
{
    #if (CYHAL_API_VERSION >= 2)
    ref_pin->pin = pin;
    #else
    *ref_pin = pin;
    #endif
}


//--------------------------------------------------------------------------------------------------
// _mtb_bmx160_free_pin
//--------------------------------------------------------------------------------------------------
static inline void _mtb_bmx160_free_pin(_mtb_bmx160_interrupt_pin_t ref_pin)
{
    #if (CYHAL_API_VERSION >= 2)
    cyhal_gpio_free(ref_pin.pin);
    #else
    cyhal_gpio_free(ref_pin);
    #endif
}


//--------------------------------------------------------------------------------------------------
// _mtb_bmx160_config_int
//--------------------------------------------------------------------------------------------------
static cy_rslt_t _mtb_bmx160_config_int(_mtb_bmx160_interrupt_pin_t* intpin, cyhal_gpio_t pin,
                                        bool init, uint8_t intr_priority, cyhal_gpio_event_t event,
                                        cyhal_gpio_event_callback_t callback, void* callback_arg)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    if (NULL == callback)
    {
        _mtb_bmx160_set_pin(intpin, NC);
        cyhal_gpio_free(pin);
    }
    else
    {
        if (init)
        {
            result = cyhal_gpio_init(pin, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, 0);
        }
        if (CY_RSLT_SUCCESS == result)
        {
            _mtb_bmx160_set_pin(intpin, pin);
            #if (CYHAL_API_VERSION >= 2)
            intpin->callback = callback;
            intpin->callback_arg = callback_arg;
            cyhal_gpio_register_callback(pin, intpin);
            #else
            cyhal_gpio_register_callback(pin, callback, callback_arg);
            #endif
            cyhal_gpio_enable_event(pin, event, intr_priority, 1);
        }
    }

    return result;
}


//--------------------------------------------------------------------------------------------------
// _mtb_bmx160_init_common
//--------------------------------------------------------------------------------------------------
static cy_rslt_t _mtb_bmx160_init_common(mtb_bmx160_t* obj, uint8_t bmm_addr)
{
    // Configure the BMI160's iterface to the BMM150
    obj->sensor1.aux_cfg.aux_i2c_addr = bmm_addr;
    obj->sensor1.aux_cfg.aux_sensor_enable = BMI160_ENABLE;
    obj->sensor1.aux_cfg.manual_enable = BMI160_ENABLE;
    obj->sensor1.aux_cfg.aux_rd_burst_len = BMI160_AUX_READ_LEN_2;

    // Configure the BMM150 structure
    obj->sensor2.intf       = BMM150_I2C_INTF;
    obj->sensor2.read       = (bmm150_read_fptr_t)bmm150_aux_read;
    obj->sensor2.write      = (bmm150_write_fptr_t)bmm150_aux_write;
    obj->sensor2.delay_us   = delay_us_wrapper;
    obj->sensor2.intf_ptr   = &(obj->sensor1);

    // Initialize BNI160 sensor
    int8_t status = bmi160_init(&(obj->sensor1));
    if (BMI160_OK == status)
    {
        cyhal_system_delay_us(SOFT_RESET_DELAY_US); // per datasheet, delay needed to reboot
        status = bmi160_aux_init(&(obj->sensor1));
    }
    if (BMI160_OK == status)
    {
        status = bmm150_init(&(obj->sensor2));
    }

    return (BMI160_OK == status) // BMI160+BMM150 initialized successfully (BMM150_OK == BMI160_OK)
        ? mtb_bmx160_config_default(obj)
        : BMI160_ERROR(status);
}


//--------------------------------------------------------------------------------------------------
// mtb_bmx160_init_i2c
//--------------------------------------------------------------------------------------------------
cy_rslt_t mtb_bmx160_init_i2c(mtb_bmx160_t* obj, cyhal_i2c_t* inst, mtb_bmx160_address_t address)
{
    CY_ASSERT(inst != NULL);
    i2c = inst;

    // Configure the BMI160 structure
    obj->sensor1.id         = address >> 8;
    obj->sensor1.intf       = BMI160_I2C_INTF;
    obj->sensor1.read       = (bmi160_read_fptr_t)i2c_read_bytes;
    obj->sensor1.write      = (bmi160_write_fptr_t)i2c_write_bytes;
    obj->sensor1.delay_ms   = delay_ms_wrapper;
    _mtb_bmx160_set_pin(&(obj->intpin1), NC);
    _mtb_bmx160_set_pin(&(obj->intpin2), NC);

    return _mtb_bmx160_init_common(obj, address & 0xFF);
}


//--------------------------------------------------------------------------------------------------
// mtb_bmx160_init_spi
//--------------------------------------------------------------------------------------------------
cy_rslt_t mtb_bmx160_init_spi(mtb_bmx160_t* obj, cyhal_spi_t* inst, cyhal_gpio_t spi_ss)
{
    CY_ASSERT(inst != NULL);
    CY_ASSERT(NC != spi_ss);
    spi = inst;
    spi_ssel = spi_ss;

    /* Configure the BMI160 structure */
    obj->sensor1.id         = 0;
    obj->sensor1.intf       = BMI160_SPI_INTF;
    obj->sensor1.read       = (bmi160_read_fptr_t)spi_read_bytes;
    obj->sensor1.write      = (bmi160_write_fptr_t)spi_write_bytes;
    obj->sensor1.delay_ms   = delay_ms_wrapper;
    _mtb_bmx160_set_pin(&(obj->intpin1), NC);
    _mtb_bmx160_set_pin(&(obj->intpin2), NC);

    return _mtb_bmx160_init_common(obj, BMM150_DEFAULT_I2C_ADDRESS);
}


//--------------------------------------------------------------------------------------------------
// mtb_bmx160_config_default
//--------------------------------------------------------------------------------------------------
cy_rslt_t mtb_bmx160_config_default(mtb_bmx160_t* obj)
{
    // Select the Output data rate, range of accelerometer sensor
    obj->sensor1.accel_cfg.odr   = BMI160_ACCEL_ODR_1600HZ;
    obj->sensor1.accel_cfg.range = BMI160_ACCEL_RANGE_2G;
    obj->sensor1.accel_cfg.bw    = BMI160_ACCEL_BW_NORMAL_AVG4;

    // Select the power mode of accelerometer sensor
    obj->sensor1.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    // Select the Output data rate, range of gyroscope sensor
    obj->sensor1.gyro_cfg.odr   = BMI160_GYRO_ODR_3200HZ;
    obj->sensor1.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    obj->sensor1.gyro_cfg.bw    = BMI160_GYRO_BW_NORMAL_MODE;

    // Select the power mode of gyroscope sensor
    obj->sensor1.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

    // Set the sensor configuration
    int8_t status = bmi160_set_sens_conf(&(obj->sensor1));

    struct bmm150_settings bmm_settings;
    bmm_settings.pwr_mode = BMM150_POWERMODE_FORCED;
    bmm_settings.preset_mode = BMM150_PRESETMODE_REGULAR;

    if (BMI160_OK == status)
    {
        status = bmm150_set_presetmode(&bmm_settings, &(obj->sensor2));
    }

    if (BMI160_OK == status)
    {
        status = bmm150_set_op_mode(&bmm_settings, &(obj->sensor2));
    }

    if (BMI160_OK == status)
    {
        uint8_t aux_addr = 0x42;
        obj->sensor1.aux_cfg.aux_odr = 8;
        status = bmi160_config_aux_mode(&(obj->sensor1));

        if (BMI160_OK == status)
        {
            status = bmi160_set_aux_auto_mode(&aux_addr, &(obj->sensor1));
        }
    }

    return (BMI160_OK == status)
        ? CY_RSLT_SUCCESS
        : BMI160_ERROR(status);
}


//--------------------------------------------------------------------------------------------------
// mtb_bmx160_read
//--------------------------------------------------------------------------------------------------
cy_rslt_t mtb_bmx160_read(mtb_bmx160_t* obj, mtb_bmx160_data_t* sensor_data)
{
    // To read both Accel and Gyro data along with time
    int8_t status = bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL | BMI160_TIME_SEL),
                                           &(sensor_data->accel), &(sensor_data->gyro),
                                           &(obj->sensor1));

    if (BMI160_OK == status)
    {
        uint8_t mag_data[8];
        status = bmi160_read_aux_data_auto_mode(mag_data, &(obj->sensor1));
        if (BMI160_OK == status)
        {
            status = bmm150_aux_mag_data(mag_data, &(sensor_data->mag), &(obj->sensor2));
        }
    }

    return (BMI160_OK == status)
        ? CY_RSLT_SUCCESS
        : BMI160_ERROR(status);
}


//--------------------------------------------------------------------------------------------------
// mtb_bmx160_get_motion
//--------------------------------------------------------------------------------------------------
struct bmi160_dev* mtb_bmx160_get_motion(mtb_bmx160_t* obj)
{
    return &(obj->sensor1);
}


//--------------------------------------------------------------------------------------------------
// mtb_bmx160_get_geo
//--------------------------------------------------------------------------------------------------
struct bmm150_dev* mtb_bmx160_get_geo(mtb_bmx160_t* obj)
{
    return &(obj->sensor2);
}


//--------------------------------------------------------------------------------------------------
// mtb_bmx160_selftest
//--------------------------------------------------------------------------------------------------
cy_rslt_t mtb_bmx160_selftest(mtb_bmx160_t* obj)
{
    int8_t status = bmi160_perform_self_test(BMI160_ACCEL_SEL, &(obj->sensor1));
    cyhal_system_delay_us(SOFT_RESET_DELAY_US); // per datasheet, delay needed after reset to reboot

    if (status == BMI160_OK)
    {
        status = bmi160_perform_self_test(BMI160_GYRO_SEL, &(obj->sensor1));
        cyhal_system_delay_us(SOFT_RESET_DELAY_US); // delay needed after another reset
    }

    // The BMI160 self test causes a reset which loses the aux configuration. Reapply...
    if (status == BMI160_OK)
    {
        status = bmi160_aux_init(&(obj->sensor1));
    }

    if (status == BMI160_OK)
    {
        status = bmm150_perform_self_test(BMM150_SELF_TEST_NORMAL, &(obj->sensor2));
    }

    return (BMI160_OK == status)
        ? CY_RSLT_SUCCESS
        : BMI160_ERROR(status);
}


//--------------------------------------------------------------------------------------------------
// mtb_bmx160_config_int
//--------------------------------------------------------------------------------------------------
cy_rslt_t mtb_bmx160_config_int(mtb_bmx160_t* obj, struct bmi160_int_settg* intsettings,
                                cyhal_gpio_t pin, uint8_t intr_priority, cyhal_gpio_event_t event,
                                cyhal_gpio_event_callback_t callback, void* callback_arg)
{
    cy_rslt_t result;

    if (_mtb_bmx160_pins_equal(obj->intpin1, pin))
    {
        result = _mtb_bmx160_config_int(&(obj->intpin1), pin, false, intr_priority, event, callback,
                                        callback_arg);
    }
    else if (_mtb_bmx160_pins_equal(obj->intpin2, pin))
    {
        result = _mtb_bmx160_config_int(&(obj->intpin2), pin, false, intr_priority, event, callback,
                                        callback_arg);
    }
    else if (_mtb_bmx160_pins_equal(obj->intpin1, NC))
    {
        result = _mtb_bmx160_config_int(&(obj->intpin1), pin, true, intr_priority, event, callback,
                                        callback_arg);
    }
    else if (_mtb_bmx160_pins_equal(obj->intpin2, NC))
    {
        result = _mtb_bmx160_config_int(&(obj->intpin2), pin, true, intr_priority, event, callback,
                                        callback_arg);
    }
    else
    {
        result = MTB_BMX160_RSLT_ERR_INSUFFICIENT_INT_PINS;
    }

    if (result == CY_RSLT_SUCCESS)
    {
        int8_t status = bmi160_set_int_config(intsettings, &(obj->sensor1));
        if (status != BMI160_OK)
        {
            result = BMI160_ERROR(status);
        }
    }

    return result;
}


//--------------------------------------------------------------------------------------------------
// mtb_bmx160_free
//--------------------------------------------------------------------------------------------------
void mtb_bmx160_free(mtb_bmx160_t* obj)
{
    if (!_mtb_bmx160_pins_equal(obj->intpin1, NC))
    {
        _mtb_bmx160_free_pin(obj->intpin1);
    }

    if (!_mtb_bmx160_pins_equal(obj->intpin2, NC))
    {
        _mtb_bmx160_free_pin(obj->intpin2);
    }

    i2c = NULL;
}


#if defined(__cplusplus)
}
#endif
