#include "bme_280.h"

int32_t bme280_setup_device(BmeConfig_s* config)
{
    // CHANGES TO HUMIDITY CONTROL (ctrl_hum) ONLY BECOME EFFECTIVE
    // AFTER A WRITE TO MEASURE CONTROL (ctrl_meas)
    uint8_t ctrl_humidity_reg, ctrl_meas_reg, config_reg;

    ctrl_humidity_reg = (uint8_t)(config->humiditySamplingSelection & 0x07);
    ctrl_meas_reg = (uint8_t)(config->mode & 0x3) | 
                    (uint8_t)((config->pressureSamplingSelection & 0x07) << 2) | 
                    (uint8_t)((config->temperatureSamplingSelection & 0x07) << 5);
    
    
}

int32_t bme280_set_mode(BmeMode_e mode);

int32_t bme280_read_sensor_sync(void);

int32_t bme280_get_humidity(void);

int32_t bme280_get_pressure(void);

int32_t bme280_get_temperature(void);


