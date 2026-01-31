#include "sensor_io.h"

#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>

#define I2C_DEV DT_NODELABEL(imu_sensor)

static const struct i2c_dt_spec i2c_spec = I2C_DT_SPEC_GET(I2C_DEV);
static const struct device *i2c_dev;

static int zephyr_i2c_config(void* config_data);
static int i2c_zephyr_write_byte_wrapper(void* driver, uint16_t i2cAddr, uint8_t writeAddr, uint8_t writeVal);
static int i2c_zephyr_write_arr_wrapper(void* driver, uint16_t i2cAddr, uint8_t writeAddr, const uint8_t* writeArr, uint32_t writeLen);
static int i2c_zephyr_read_byte_wrapper(void* driver, uint16_t i2cAddr, uint8_t readAddr, uint8_t* readVal);
static int i2c_zephyr_read_arr_wrapper(void* driver,uint16_t i2cAddr, uint8_t readAddr, uint8_t* readArr, uint32_t readLen);


static const Sensor_IO_Descriptor_s sensor_descriptor =    { 
                                                                //.driver_handle = (void*)i2c_spec.bus,
                                                                .config = &zephyr_i2c_config,
                                                                .write_byte = &i2c_zephyr_write_byte_wrapper,
                                                                .write_array = &i2c_zephyr_write_arr_wrapper,
                                                                .read_byte = &i2c_zephyr_read_byte_wrapper,
                                                                .read_array = &i2c_zephyr_read_arr_wrapper
                                                            };



static int zephyr_i2c_config(void* config_data)
{
    //i2c_dev = (const struct device *)config_data;
    i2c_dev = i2c_spec.bus;
    if(!device_is_ready(i2c_dev))
    {
        return SENSOR_IO_CONFIG_ERROR;
    }
    config_data = (void*)i2c_dev;
    if (!config_data) 
    {
		return SENSOR_IO_CONFIG_ERROR;
    }
    else
    {
        return SENSOR_IO_SUCCESS;
    }
}

const Sensor_IO_Descriptor_s* get_mpu_descriptor(void)
{
    return &sensor_descriptor;
}

const Sensor_IO_Descriptor_s* get_bme_descriptor(void)
{
    return &sensor_descriptor;
}

static int i2c_zephyr_write_byte_wrapper(void* driver, uint16_t i2cAddr, uint8_t writeAddr, uint8_t writeVal)
{
    i2c_dev = (const struct device *)driver;
    return i2c_reg_write_byte(i2c_dev, i2cAddr, writeAddr, writeVal);
}

static int i2c_zephyr_write_arr_wrapper(void* driver, uint16_t i2cAddr, uint8_t writeAddr, const uint8_t* writeArr, uint32_t writeLen)
{
    i2c_dev = (const struct device *)driver;
    return i2c_burst_write(i2c_dev, i2cAddr, writeAddr, writeArr, writeLen);
}

static int i2c_zephyr_read_byte_wrapper(void* driver, uint16_t i2cAddr, uint8_t readAddr, uint8_t* readVal)
{
    int err;
    i2c_dev = (const struct device *)driver;
    printk("Reading byte...\n");
    err = i2c_reg_read_byte(i2c_dev, i2cAddr, readAddr, readVal);
    printk("Byte was read.\n");
    return err;
}
static int i2c_zephyr_read_arr_wrapper(void* driver,uint16_t i2cAddr, uint8_t readAddr, uint8_t* readArr, uint32_t readLen)
{
    i2c_dev = (const struct device *)driver;
    return i2c_burst_read(i2c_dev, i2cAddr, readAddr, readArr, readLen);
}