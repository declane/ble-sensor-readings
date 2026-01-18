#include "gy_521.h"

#define LOG_MODULE_NAME IMU
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

//SCL - 27  SDA - 26
#define I2C_DEV DT_NODELABEL(imu_sensor)
#define GPIO_DEV DT_NODELABEL(imu_ready_pin)

static const struct device *gpiob;
static const struct device *i2c_dev;
static const struct i2c_dt_spec i2c_spec = I2C_DT_SPEC_GET(I2C_DEV);
static struct gpio_callback gpio_cb;
static const struct gpio_dt_spec gpio_spec = GPIO_DT_SPEC_GET(GPIO_DEV, gpios);

static MPU_conf_t imu_config;
static MPU6050_conversions_t curr_conv;

/******************** Private Functions *************************/
/* -- accel_sensitiviy_config() --
 * Input: uint8_t configuration
 * Return: uint8_t register_value
 * Description:
 *
 * Internal function of the MPU6050 driver, that configures the accelerometer parameters
 *
 * It returns the register value to be written for the specified configuration
 * and configures the sensitivity value with which we divide every time we convert to g/s.
 *
 * */
static uint8_t accel_sensitiviy_config(uint8_t config)
{
  uint8_t reg_value; // Value to be written to the register //

  // Based on the configuration chosen by the user //
  // Choose the sensitivity value and the register value for the MPU6050 //
  switch (config) {
    case MPU6050_Accelerometer_2G:
      reg_value = ACCEL_SCALE_2G;
      curr_conv.accel_sensitivity = ACCEL_SENS_2G/Gs_TO_mm_kgs;
      break;
    case MPU6050_Accelerometer_4G:
      reg_value = ACCEL_SCALE_4G;
      curr_conv.accel_sensitivity = ACCEL_SENS_4G/Gs_TO_mm_kgs;
      break;
    case MPU6050_Accelerometer_8G:
      reg_value = ACCEL_SCALE_8G;
      curr_conv.accel_sensitivity = ACCEL_SENS_8G/Gs_TO_mm_kgs;
      break;
    case MPU6050_Accelerometer_16G:
      reg_value = ACCEL_SCALE_16G;
      curr_conv.accel_sensitivity = ACCEL_SENS_16G/Gs_TO_mm_kgs;
      break;
    default:
      reg_value = ACCEL_SCALE_2G;
      curr_conv.accel_sensitivity = ACCEL_SENS_2G/Gs_TO_mm_kgs;
      break;
  }

  return reg_value;
}

/* -- gyro_sensitiviy_config() --
 * Input: uint8_t configuration
 * Return: uint8_t register_value
 * Description:
 *
 * Internal function of the MPU6050 driver, that configures the gyroscope parameters
 *
 * It returns the register value to be written for the specified configuration
 * and configures the sensitivity value with which we divide every time we convert to deg/s.
 *
 * */
static uint8_t gyro_sensitiviy_config(uint8_t config)
{
  uint8_t reg_value; // Value to be written to the register //

  // Based on the configuration chosen by the user //
  // Choose the sensitivity value and the register value for the MPU6050 //
  switch (config) {
    case MPU6050_Gyroscope_250_deg:
      reg_value = GYRO_SCALE_250_DEG;
      curr_conv.gyro_sensitivity = GYRO_SENS_250_DEG/DEG_TO_mDEG;
      break;
    case MPU6050_Gyroscope_500_deg:
      reg_value = GYRO_SCALE_500_DEG;
      curr_conv.gyro_sensitivity = GYRO_SENS_500_DEG/DEG_TO_mDEG;
      break;
    case MPU6050_Gyroscope_1000_deg:
      reg_value = GYRO_SCALE_1K_DEG;
      curr_conv.gyro_sensitivity = GYRO_SENS_1K_DEG/DEG_TO_mDEG;
      break;
    case MPU6050_Gyroscope_2000_deg:
      reg_value = GYRO_SCALE_2K_DEG;
      curr_conv.gyro_sensitivity = GYRO_SENS_2K_DEG/DEG_TO_mDEG;
      break;
    default:
      reg_value = GYRO_SCALE_250_DEG;
      curr_conv.gyro_sensitivity = GYRO_SENS_250_DEG/DEG_TO_mDEG;
      break;
  }

  return reg_value;
}

/****************** Public Functions *********************/
int interrupt_MPU6050_en(gpio_callback_handler_t handler)
{
    int err;
    uint8_t reg_addr = INT_PIN_CFG_REG;
    uint8_t data_buf = 0x30;
    err = i2c_reg_write_byte(i2c_dev, MPU_I2C_ADDR, reg_addr, data_buf);
	if(err)
	{
		return err;
	}

    reg_addr = INT_ENABLE_REG;
    data_buf = IRQ_ENABLE;
    err = i2c_reg_write_byte(i2c_dev, MPU_I2C_ADDR, reg_addr, data_buf);
	if(err)
	{
		return err;
	}

	// Read status to clear
	reg_addr = INT_STATUS_REG;
	err = i2c_reg_read_byte(i2c_dev, MPU_I2C_ADDR, reg_addr, &data_buf);

  gpiob = gpio_spec.port;

  if( !device_is_ready(gpiob))
  {
    printk("GPIO device is not ready... what da heck?");
  }

  gpio_pin_configure(gpiob, INT_DATA_RDY_PIN, (GPIO_INPUT |  GPIO_ACTIVE_HIGH | GPIO_PULL_DOWN));
	gpio_init_callback(&gpio_cb, handler, BIT(INT_DATA_RDY_PIN));
	gpio_add_callback(gpiob, &gpio_cb);
  gpio_pin_interrupt_configure(gpiob, INT_DATA_RDY_PIN, GPIO_INT_EDGE_TO_ACTIVE);

	return err;
}

int interrupt_MPU6050_dis()
{
	int err;
	uint8_t reg_addr = INT_ENABLE_REG;
    uint8_t data_buf = IRQ_DISABLE;
	err = i2c_reg_write_byte(i2c_dev, MPU_I2C_ADDR, reg_addr, data_buf);

	return err;
}


int MPU_read_all_data(MPU6050_t* data_buf)
{
    
    uint16_t temperature;
    int err = 0;
    uint8_t write_data = ACCEL_XOUT_H_REG;
    uint8_t* write_ptr = &write_data;
    
    uint8_t temp_data[14];
    err = i2c_write_read(i2c_dev, (uint8_t)MPU_I2C_ADDR, write_ptr, 1, temp_data, 14);
    if(err)
    {
        LOG_ERR("Failed to read data. (err %i)\n", err);
        return -2;
    }
    //Process accelerometers
    data_buf->Accel_X_RAW = (temp_data[0] << 8) | temp_data[1];
    data_buf->Accel_Y_RAW = (temp_data[2] << 8) | temp_data[3];
    data_buf->Accel_Z_RAW = (temp_data[4] << 8) | temp_data[5];
    data_buf->Ax = data_buf->Accel_X_RAW/curr_conv.accel_sensitivity;
    data_buf->Ay = data_buf->Accel_Y_RAW/curr_conv.accel_sensitivity;
    data_buf->Az = data_buf->Accel_Z_RAW/curr_conv.accel_sensitivity;

    //Process Temeprature
    temperature = (uint16_t) (temp_data[6] << 8 | temp_data[7]);
    data_buf->Temperature = (float) ((uint16_t) temperature / (float) 340.0 + (float) 36.53);

    //Process Gyros
    data_buf->Gyro_X_RAW = (temp_data[8] << 8) | temp_data[9];
    data_buf->Gyro_Y_RAW = (temp_data[10] << 8) | temp_data[11];
    data_buf->Gyro_Z_RAW = (temp_data[12] << 8) | temp_data[13];
    data_buf->Gx = data_buf->Gyro_X_RAW/curr_conv.gyro_sensitivity;
    data_buf->Gy = data_buf->Gyro_Y_RAW/curr_conv.gyro_sensitivity;
    data_buf->Gz = data_buf->Gyro_Z_RAW/curr_conv.gyro_sensitivity;
    
    return err;
    
}

int config_MPU6050(MPU_conf_t *config_param)
{
    uint8_t reg_addr;
    uint8_t config_byte;
    int err = 0;
    
    //Setting sample rate:
    reg_addr = (uint8_t)SMPLRT_DIV_REG;
    config_byte = (uint8_t)config_param->sampling_rate;
    err = i2c_reg_write_byte(i2c_dev, MPU_I2C_ADDR, reg_addr, config_byte);
    if(err)
    {
        LOG_ERR("Failed to Set Sampling Rate. (err %i)\n", err);
        return -2;
    }
    //Setting accel:
    reg_addr = (uint8_t)ACCEL_CONFIG_REG;
    config_byte = accel_sensitiviy_config((uint8_t)config_param->accel_setting);
    err = i2c_reg_write_byte(i2c_dev, MPU_I2C_ADDR, reg_addr, config_byte);
    if(err)
    {
        LOG_ERR("Failed to set Accel. (err %i)\n", err);
        return -2;
    }
    //Setting gyro:
    reg_addr = (uint8_t)GYRO_CONFIG_REG;
    config_byte = gyro_sensitiviy_config((uint8_t)config_param->gyro_setting);
    err = i2c_reg_write_byte(i2c_dev, MPU_I2C_ADDR, reg_addr, config_byte);
    if(err)
    {
        LOG_ERR("Failed to set Gyro. (err %i)\n", err);
        return -2;
    }

    imu_config.accel_setting = config_param->accel_setting;
    imu_config.gyro_setting = config_param->gyro_setting;
    imu_config.sampling_rate = config_param->sampling_rate;
    return err;
}

int init_MPU6050(MPU_conf_t *config_param, gpio_callback_handler_t handler)
{
  printk("Starting zephyr sync mpu init.\n");  
  //Setup zephyr I2C driver
    int err;
    i2c_dev = i2c_spec.bus;
    if (!i2c_dev) {
		  printk("I2C: Device driver not found.\n");
		  return -1;
    }

    //verify we have the correct address
    uint8_t reg_addr = WHO_AM_I_REG;
    uint8_t data_buf;
    err = i2c_reg_read_byte(i2c_dev, MPU_I2C_ADDR, reg_addr,
                                &data_buf);
    if (err)
    {
        LOG_ERR("Unable get WAI data. (err %i)\n", err);
        return -2;
    }
    LOG_INF("Who am I: 0x%x\n", data_buf);

    //turn on sensor
    reg_addr = (uint8_t)PWR_MGMT_1_REG;
    data_buf = 0;
    err = i2c_reg_write_byte(i2c_dev, MPU_I2C_ADDR, reg_addr, data_buf);
    if(err)
    {
        LOG_ERR("Failed to write to MPU. (err %i)\n", err);
        return -2;
    }

    err = config_MPU6050(config_param);
	  err = interrupt_MPU6050_en(handler);

    return err;
}
