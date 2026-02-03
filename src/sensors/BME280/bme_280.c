#include "bme_280.h"
#include "bme_280_config.h"
#include "../sensor_io/sensor_io.h"

const static Sensor_IO_Descriptor_s* io_descriptor;
static BmeConfig_s* configParams = 0;

static uint8_t ctrl_meas_reg;
static uint8_t raw_sensor_data[BME280_DATA_BYTE_COUNT] = {0};
static uint8_t raw_calibration_data[BME280_CAL_FULL_LENGTH] = {0};
static uint16_t dig_T1, dig_P1;
static int16_t  dig_T2, dig_T3, dig_P2, dig_P3, dig_P4,
                dig_P5, dig_P6, dig_P7, dig_P8, dig_P9,
                dig_H2, dig_H4, dig_H5;
static uint8_t  dig_H1, dig_H3;
static int8_t   dig_H6;

static int32_t t_fine;

static uint32_t getTemperatureSamplingTime_us(void);
static uint32_t getPressuringSamplingTime_us(void);
static uint32_t getHumiditySamplingTime_us(void);

int32_t bme280_setup_device(BmeConfig_s* config)
{
    int err;

    // make sure values passed here are acceptable.
    if( (config->filterCoefficient > BmeIIR_Filter_16) || (config->filterCoefficient < BmeIIR_FilterOff) ||
        (config->mode > BmeModeNormal) || (config->mode < BmeModeSleep) ||
        (config->humiditySamplingSelection > BmeSampling_16) || (config->humiditySamplingSelection < BmeSampling_off) ||
        (config->temperatureSamplingSelection > BmeSampling_16) || (config->temperatureSamplingSelection < BmeSampling_off) ||
        (config->pressureSamplingSelection > BmeSampling_16) || (config->pressureSamplingSelection < BmeSampling_off) ||
        (config->standByTime > BmeStandByTime_20ms) || (config->standByTime < BmeStandByTime_0500us) )
        {
            return BME280_RESULT_INVALID_PARAMETER;
        }
    
    io_descriptor = get_bme_descriptor();
    err = io_descriptor->config(io_descriptor->driver_handle);
    if(err == SENSOR_IO_CONFIG_ERROR)
    {
        return BME280_RESULT_IO_DRIVER_ERROR;
    }
   
    // CHANGES TO HUMIDITY CONTROL (ctrl_hum) ONLY BECOME EFFECTIVE
    // AFTER A WRITE TO MEASURE CONTROL (ctrl_meas)
    uint8_t ctrl_humidity_reg = 0;
    ctrl_meas_reg = 0;
    uint8_t config_reg = 0;

    ctrl_humidity_reg = (uint8_t)((uint8_t)config->humiditySamplingSelection & 0x07);
    ctrl_meas_reg = (uint8_t)((uint8_t)config->mode & 0x3) | 
                    (uint8_t)(((uint8_t)config->pressureSamplingSelection & 0x07) << 2) | 
                    (uint8_t)(((uint8_t)config->temperatureSamplingSelection & 0x07) << 5);
    
    config_reg = (uint8_t)((uint8_t)config->filterCoefficient << 2) |
                 (uint8_t)((uint8_t)config->standByTime << 5);

    // write bytes to designated registers.
    err = io_descriptor->write_byte(io_descriptor->driver_handle, BME280_I2C_ADDR_DEF, BME280_REG_CTRL_HUM, ctrl_humidity_reg);
    if(err == SENSOR_IO_CONFIG_ERROR)
    {
        return BME280_RESULT_IO_DRIVER_ERROR;
    }

    err = io_descriptor->write_byte(io_descriptor->driver_handle, BME280_I2C_ADDR_DEF, BME280_REG_CTRL_MEAS, ctrl_meas_reg);
    if(err == SENSOR_IO_CONFIG_ERROR)
    {
        return BME280_RESULT_IO_DRIVER_ERROR;
    }

    err = io_descriptor->write_byte(io_descriptor->driver_handle, BME280_I2C_ADDR_DEF, BME280_REG_CONFIG, config_reg);
    if(err == SENSOR_IO_CONFIG_ERROR)
    {
        return BME280_RESULT_IO_DRIVER_ERROR;
    }

    // read calibration
    err = bme280_read_calibration_sync();
    if(err)
    {
        return err;
    }

    configParams = config;

    return BME280_RESULT_SUCCESS;
}

int32_t bme280_set_mode(BmeMode_e mode)
{
    if(!configParams)
    {
        return BME280_RESULT_MODULE_NOT_CONFIGURED;
    }

    if((mode > BmeModeNormal) || (mode < BmeModeSleep))
    {
        return BME280_RESULT_INVALID_PARAMETER;
    }
    
    ctrl_meas_reg &= 0xFC;
    ctrl_meas_reg |= (uint8_t)mode & 0x03;
    
    // write byte.
    int err = io_descriptor->write_byte(io_descriptor->driver_handle, BME280_I2C_ADDR_DEF, BME280_REG_CTRL_MEAS, ctrl_meas_reg);
    if(err == SENSOR_IO_CONFIG_ERROR)
    {
        ctrl_meas_reg &= 0xFC;
        ctrl_meas_reg |= (uint8_t)configParams->mode & 0x03;
        return BME280_RESULT_IO_DRIVER_ERROR;
    }

    configParams->mode = mode;
    return BME280_RESULT_SUCCESS;
}

int32_t bme280_read_sensor_sync(void)
{
    if(!configParams)
    {
        return BME280_RESULT_MODULE_NOT_CONFIGURED;
    }
    
    // read bytes into buffer
    int err = io_descriptor->read_array(io_descriptor->driver_handle, BME280_I2C_ADDR_DEF, BME280_DATA_START_ADDR, raw_sensor_data, BME280_DATA_BYTE_COUNT);
    if(err == SENSOR_IO_CONFIG_ERROR)
    {
        return BME280_RESULT_IO_DRIVER_ERROR;
    }
    
    return BME280_RESULT_SUCCESS;
}

int32_t bme280_read_calibration_sync(void)
{
    int err;
    int16_t dig_h4_lsb;
    int16_t dig_h4_msb;
    int16_t dig_h5_lsb;
    int16_t dig_h5_msb;

    if(!configParams)
    {
        return BME280_RESULT_MODULE_NOT_CONFIGURED;
    }    
    
    // read calibration into buffer.
    err = io_descriptor->read_array(io_descriptor->driver_handle, 
                                    BME280_I2C_ADDR_DEF, 
                                    BME280_CAL_00_25_START_ADDR, 
                                    raw_calibration_data, 
                                    BME280_CAL_00_25_LENGTH);
    if(err == SENSOR_IO_CONFIG_ERROR)
    {
        return BME280_RESULT_IO_DRIVER_ERROR;
    }
    
    err = io_descriptor->read_array(io_descriptor->driver_handle, 
                                    BME280_I2C_ADDR_DEF, 
                                    BME280_CAL_26_32_START_ADDR, 
                                    (raw_calibration_data + BME280_CAL_00_25_LENGTH), 
                                    BME280_CAL_26_32_LENGTH);
    if(err == SENSOR_IO_CONFIG_ERROR)
    {
        return BME280_RESULT_IO_DRIVER_ERROR;
    }


    dig_T1 = ((uint16_t)raw_calibration_data[1] << 8)  | (uint16_t)raw_calibration_data[0];
    dig_T2 = (int16_t)((uint16_t)raw_calibration_data[3] << 8)  | (uint16_t)raw_calibration_data[2];
    dig_T3 = (int16_t)((uint16_t)raw_calibration_data[5] << 8)  | (uint16_t)raw_calibration_data[4];
    
    dig_P1 = ((uint16_t)raw_calibration_data[7] << 8)  | (uint16_t)raw_calibration_data[6];
    dig_P2 = (int16_t)((uint16_t)raw_calibration_data[9] << 8)  | (uint16_t)raw_calibration_data[8];
    dig_P3 = (int16_t)((uint16_t)raw_calibration_data[11] << 8) | (uint16_t)raw_calibration_data[10];
    dig_P4 = (int16_t)((uint16_t)raw_calibration_data[13] << 8) | (uint16_t)raw_calibration_data[12];
    dig_P5 = (int16_t)((uint16_t)raw_calibration_data[15] << 8) | (uint16_t)raw_calibration_data[14];
    dig_P6 = (int16_t)((uint16_t)raw_calibration_data[17] << 8) | (uint16_t)raw_calibration_data[16];
    dig_P7 = (int16_t)((uint16_t)raw_calibration_data[19] << 8) | (uint16_t)raw_calibration_data[18];
    dig_P8 = (int16_t)((uint16_t)raw_calibration_data[21] << 8) | (uint16_t)raw_calibration_data[20];
    dig_P9 = (int16_t)((uint16_t)raw_calibration_data[23] << 8) | (uint16_t)raw_calibration_data[22];
    
    // seems to skip index 24 (0xA0)...
    dig_H1 = raw_calibration_data[25];
    dig_H2 = (int16_t)(raw_calibration_data[27] << 8) | raw_calibration_data[26];
    dig_H3 = raw_calibration_data[28];

    // Double Check these...
    dig_h4_msb = (int16_t)(int8_t)raw_calibration_data[29] * 16;
    dig_h4_lsb = (int16_t)(raw_calibration_data[30] & 0x0F);
    dig_H4 = dig_h4_msb | dig_h4_lsb;
    dig_h5_msb = (int16_t)(int8_t)raw_calibration_data[30] * 16;
    dig_h5_lsb = (int16_t)(raw_calibration_data[31] >> 4);
    dig_H5 = dig_h5_msb | dig_h5_lsb;

    dig_H6 = raw_calibration_data[32];

    return BME280_RESULT_SUCCESS;
}

// divide return value by 1024 to get releative humidity (RH)
uint32_t bme280_get_humidity(void)
{
    if(!configParams)
    {
        return -1;
    } 
    int32_t v_x1_u32r, adc_H;
    adc_H =   (int32_t)(raw_sensor_data[7]) |       // BME280_REG_PRESS_XLSB
            ( (int32_t)(raw_sensor_data[6]) << 8);  // BME280_REG_HUM_MSB
    v_x1_u32r = (t_fine - ((int32_t)76800));

    v_x1_u32r = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r *
                ((int32_t)dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)dig_H3)) >> 11) +
                ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    return (uint32_t)(v_x1_u32r>>12);
}

int32_t bme280_get_humidity_relHum(void)
{
    if(!configParams)
    {
        return -1;
    }
    return bme280_get_humidity()/1024;
}

// divide return value by 256 to get hPa
int32_t bme280_get_pressure(void)
{
    if(!configParams)
    {
        return -1;
    }
    
    int32_t adc_P = ( (int32_t)(raw_sensor_data[2] & 0xF0) >> 4) |  // BME280_REG_PRESS_XLSB
                    ( (int32_t)(raw_sensor_data[1]) << 4) |         // BME280_REG_PRESS_LSB
                    ( (int32_t)(raw_sensor_data[0]) << 12);         // BME280_REG_PRESS_MSB;
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1*(int64_t)dig_P5)<<17);
    var2 = var2 + (((int64_t)dig_P4)<<35);
    var1 = ((var1 * var1 * (int64_t)dig_P3)>>8) + ((var1 * (int64_t)dig_P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)dig_P1)>>33;
    if (var1 == 0)
    {
        return 0; // avoid exception caused by division by zero
    }
    
    p = 1048576-adc_P;
    p = (((p<<31)-var2)*3125)/var1;
    var1 = (((int64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7)<<4);
    
    return (uint32_t)p;
}

float bme280_get_pressure_hPa(void)
{
    if(!configParams)
    {
        return -1;
    }
    
    return bme280_get_pressure()/256;
}

// divide by 100 to get degrees celsius
int32_t bme280_get_temperature(void)
{
    if(!configParams)
    {
        return -1;
    }
    
    int32_t var1, var2, T, adc_T;
    adc_T = ( (int32_t)(raw_sensor_data[5] & 0xF0) >> 4) |  // BME280_REG_TEMP_XLSB
            ( (int32_t)(raw_sensor_data[4]) << 4) |         // BME280_REG_TEMP_LSB
            ( (int32_t)(raw_sensor_data[3]) << 12);         // BME280_REG_TEMP_MSB
    var1 = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1))) >> 12) *((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

float bme280_get_temperature_c(void)
{
    if(!configParams)
    {
        return -1;
    }
    
    return bme280_get_temperature()/100;
}

uint32_t bme280_get_sample_time_us(void)
{
    if(!configParams)
    {
        return 0;
    }
    
    uint32_t sampleTime = 0;
    switch(configParams->standByTime)
    {
    case BmeStandByTime_0500us:
        sampleTime = 500;
        break;
    case BmeStandByTime_62500us:
        sampleTime = 62500;
        break;
    case BmeStandByTime_125ms:
        sampleTime = 125000;
        break;
    case BmeStandByTime_250ms:
        sampleTime = 250000;
        break;
    case BmeStandByTime_500ms:
        sampleTime = 500000;
        break;
    case BmeStandByTime_1000ms:
        sampleTime = 1000000;
        break;
    case BmeStandByTime_10ms:
        sampleTime = 10000;
        break;
    case BmeStandByTime_20ms:
        sampleTime = 20000;
        break; 
    default:
        sampleTime = 0;
    }
    sampleTime += getTemperatureSamplingTime_us();
    sampleTime += getPressuringSamplingTime_us();
    sampleTime += getHumiditySamplingTime_us();
    sampleTime += 1000;
    return sampleTime;
}

static uint32_t getTemperatureSamplingTime_us(void)
{
    switch(configParams->temperatureSamplingSelection)
    {
    case BmeSampling_off:
        return 0;
    case BmeSampling_1:
        return 2000;
    case BmeSampling_2:
        return 4000;
    case BmeSampling_4:
        return 8000;
    case BmeSampling_8:
        return 16000;
    case BmeSampling_16:
        return 32000;
    default:
        return 0;
    }
}

static uint32_t getPressuringSamplingTime_us(void)
{
    switch(configParams->pressureSamplingSelection)
    {
    case BmeSampling_off:
        return 0;
    case BmeSampling_1:
        return 2500;
    case BmeSampling_2:
        return 4500;
    case BmeSampling_4:
        return 8500;
    case BmeSampling_8:
        return 16500;
    case BmeSampling_16:
        return 32500;
    default:
        return 0;
    }
}

static uint32_t getHumiditySamplingTime_us(void)
{
    switch(configParams->humiditySamplingSelection)
    {
    case BmeSampling_off:
        return 0;
    case BmeSampling_1:
        return 2500;
    case BmeSampling_2:
        return 4500;
    case BmeSampling_4:
        return 8500;
    case BmeSampling_8:
        return 16500;
    case BmeSampling_16:
        return 32500;
    default:
        return 0;
    }
}