#ifndef BME_280_H
#define BME_280_H

/* 
Burst reads of all the data is recommended for several reasons:
1.  Fastest even if pressure for example is not being sampled.
2.  Allows for shadow reading. Breaking about measurements means we will
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
    BmeIIR_FilterCoefficient_e filterCoefficient;
} BmeConfig_s;


#endif // BME_280_H