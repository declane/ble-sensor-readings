#ifndef SENSOR_IO_H
#define SENSOR_IO_H

#include "stdint.h"

/**
 * @brief Sensor Configuration
 * @details
 *      General configuration function for a sensor.
 * 
 * @param void* - Pointer to device driver that interacts with sensor.
 * @return int  - Code indicating if configuration was successful.
 */
typedef int (*SensorConfig_fp)(void*);

/**
 * @brief Sensor Write Byte
 * @details
 *      Writes a single byte to a sensor.
 *      Correlates with zephyr "i2c_reg_write_byte"
 * 
 * @param void*     - Pointer to device driver that interacts with sensor.
 * @param uint16_t  - Sensor ID. This might be I2C address.
 * @param uint8_t   - Address byte is to be written to within sensor.
 * @param uint8_t   - Byte to be written. 
 */
typedef int (*SensorWriteByte_fp)(void*, uint16_t, uint8_t, uint8_t);

/** 
 * @brief Sensor Write Array
 * @details 
 *      Writes an array of bytes to defined length to snesor.
 *      Correlates with zephyr "i2c_burst_write"
 * 
 * @param void*     - Pointer to device driver that interacts with sensor.
 * @param uint16_t  - Sensor ID. This might be I2C address.
 * @param uint8_t   - Write start address.
 * @param uint8_t*  - Pointer to write array.
 * @param uint32_t  - Number of bytes to write.
 */
typedef int (*SensorWriteArr_fp)(void*, uint16_t,uint8_t, const uint8_t*, uint32_t);

/**
 * @brief Read Byte from Sensor
 * @details
 *      Reads a single byte from the sensor
 *      Correlates with zephyr "i2c_reg_read_byte"
 * 
 * @param void*     - Pointer to device driver that interacts with sensor.
 * @param uint16_t  - Sensor ID. This might be I2C address.
 * @param uint8_t   - Address within sensor to read byte from.
 * @param uint8_t*  - Pointer to variable where btye will be stored after read.
 */
typedef int (*SensorReadByte_fp)(void*,uint16_t,uint8_t,uint8_t*); 

/**
 * @brief Ready Array of Bytes from Sensor
 * @details
 *      Reads an array of bytes of defined size from sensor.
 *      Correlates with zephyr "i2c_burst_read"
 * 
 * @param void*     - Pointer to device driver that interacts with sensor.
 * @param uint16_t  - Sensor ID. This might be I2C address.
 * @param uint8_t   - Address within sensor to start read from.
 * @param uint8_t*  - Pointer to array where data will be stored after read.
 * @param uint32_t  - Number of bytes to be read.
 */
typedef int (*SensorReadArr_fp)(void*,uint16_t,uint8_t,uint8_t*,uint32_t);

typedef struct {
    SensorConfig_fp config;
    SensorWriteByte_fp write_byte;
    SensorWriteArr_fp write_array;
    SensorReadByte_fp read_byte;
    SensorReadArr_fp read_array;
} Sensor_IO_Descriptor_s;

#endif // SENSOR_IO_H