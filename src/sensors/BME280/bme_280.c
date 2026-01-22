#include "bme_280.h"
#include "bme_280_config.h"

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

int32_t bme280_setup_device(BmeConfig_s* config)
{
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
    return 0;
}

int32_t bme280_set_mode(BmeMode_e mode)
{
    ctrl_meas_reg &= 0xFC;
    ctrl_meas_reg |= (uint8_t)mode & 0x03;
    
    // write byte.

    return 0;
}

int32_t bme280_read_sensor_sync(void)
{
    // read bytes into buffer
}

int32_t bme280_read_calibration_sync(void)
{
    // read calibration into buffer.

    dig_T1 = (raw_calibration_data[0] << 8)  | raw_calibration_data[1];
    dig_T2 = (raw_calibration_data[2] << 8)  | raw_calibration_data[3];
    dig_T3 = (raw_calibration_data[4] << 8)  | raw_calibration_data[5];
    
    dig_P1 = (raw_calibration_data[6] << 8)  | raw_calibration_data[7];
    dig_P2 = (raw_calibration_data[8] << 8)  | raw_calibration_data[9];
    dig_P3 = (raw_calibration_data[10] << 8) | raw_calibration_data[11];
    dig_P4 = (raw_calibration_data[12] << 8) | raw_calibration_data[13];
    dig_P5 = (raw_calibration_data[14] << 8) | raw_calibration_data[15];
    dig_P6 = (raw_calibration_data[16] << 8) | raw_calibration_data[17];
    dig_P7 = (raw_calibration_data[18] << 8) | raw_calibration_data[19];
    dig_P8 = (raw_calibration_data[20] << 8) | raw_calibration_data[21];
    dig_P9 = (raw_calibration_data[22] << 8) | raw_calibration_data[23];
    
    dig_H1 = raw_calibration_data[24];
    dig_H2 = (raw_calibration_data[25] << 8) | raw_calibration_data[26];
    dig_H3 = raw_calibration_data[27];

    // Double Check these...
    dig_H4 = (raw_calibration_data[28] << 4) | (raw_calibration_data[29] & 0xF);
    dig_H5 = ( ((raw_calibration_data[29] & 0xF0) >> 4) << 8) | (raw_calibration_data[30] & 0xF);

    dig_H6 = raw_calibration_data[31];
}

int32_t bme280_get_humidity(void)
{
    int32_t v_x1_u32r, adc_H;
    v_x1_u32r = (t_fine - ((int32_t)76800));

    v_x1_u32r = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r *
                ((int32_t)dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)dig_H3)) >> 11) +
                ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    return (int32_t)(v_x1_u32r>>12);
}

int32_t bme280_get_pressure(void)
{
    int32_t adc_P;
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

int32_t bme280_get_temperature(void)
{
    int32_t var1, var2, T, adc_T;
    adc_T = ( (int32_t)(raw_sensor_data[BME280_REG_TEMP_XLSB] & 0xF0) >> 4) | 
            ( (int32_t)(raw_sensor_data[BME280_REG_TEMP_LSB]) << 4) | 
            ( (int32_t)(raw_sensor_data[BME280_REG_TEMP_MSB]) << 12);
    var1 = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1))) >> 12) *((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}


