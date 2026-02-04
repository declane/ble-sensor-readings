#ifndef BME_280_H
#define BME_280_H

#include "stdint.h"

#define BME280_RESULT_SUCCESS               0
#define BME280_RESULT_IO_DRIVER_ERROR       1
#define BME280_RESULT_INVALID_PARAMETER     2
#define BME280_RESULT_MODULE_NOT_CONFIGURED 3

/* 
Burst reads of all the data is recommended for several reasons:
1.  Fastest even if pressure for example is not being sampled.
2.  Allows for shadow reading. Breaking up measurements means we will
    have to synchronize our readings with the chip's samples.

*/

typedef enum {
    BmeModeSleep    = 0,
    BmeModeForced   = 1,
    BmeModeNormal   = 3
} BmeMode_e;

typedef enum {
    BmeIIR_FilterOff    = 0,
    BmeIIR_Filter_2     = 1,
    BmeIIR_Filter_4     = 2,
    BmeIIR_Filter_8     = 3,
    BmeIIR_Filter_16    = 4
} BmeIIR_FilterCoefficient_e;

typedef enum {
    BmeSampling_off = 0,
    BmeSampling_1   = 1,
    BmeSampling_2   = 2,
    BmeSampling_4   = 3,
    BmeSampling_8   = 4,
    BmeSampling_16  = 5
} BmeSamplingSelection_e;

typedef enum {
    BmeStandByTime_0500us   = 0,
    BmeStandByTime_62500us  = 1,
    BmeStandByTime_125ms    = 2,
    BmeStandByTime_250ms    = 3,
    BmeStandByTime_500ms    = 4,
    BmeStandByTime_1000ms   = 5,
    BmeStandByTime_10ms     = 6,
    BmeStandByTime_20ms     = 7
} BmeStandByTime_e;

typedef struct {
    BmeMode_e mode;
    BmeSamplingSelection_e humiditySamplingSelection;
    BmeSamplingSelection_e pressureSamplingSelection;
    BmeSamplingSelection_e temperatureSamplingSelection;
    BmeStandByTime_e standByTime;
    BmeIIR_FilterCoefficient_e filterCoefficient;
} BmeConfig_s;

int32_t bme280_setup_device(BmeConfig_s* config);

int32_t bme280_set_mode(BmeMode_e mode);

int32_t bme280_read_sensor_sync(void);

int32_t bme280_read_calibration_sync(void);

uint32_t bme280_get_humidity(void);

int32_t bme280_get_humidity_relHum(void);

int32_t bme280_get_pressure(void);

float bme280_get_pressure_hPa(void);

int32_t bme280_get_temperature(void);

float bme280_get_temperature_c(void);

uint32_t bme280_get_sample_time_us(void);

/**
 * Sampling time.
 * 
 * Note: if any sampling is set to 0, that part of the equation becomes 0.
 * 
 * resulting time in milliseconds.
 * 
 * t_measure_typical = 1 [2*T_oversampling] + [2*P_oversampling + 0.5] + [2*H_oversampling + 0.5]
 * t_measure_max = 1.25 [2.3*T_oversampling] + [2.3*P_oversampling + 0.575] + [2.3*H_oversampling + 0.575]
 * 
 * 
 */


#endif // BME_280_H