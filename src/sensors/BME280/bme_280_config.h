#ifndef BME_280_CONFIG_H
#define BME_280_CONFIG_H

/*
********** Power Notes **********
Supply Voltages:
Max: 3.6V
Typical: 1.8V
VDD: Internal power for analog and digitgal functional blocks
VDDIO: Digital interface power supply

- On Power up, sensor settles in sleep mode.
- Could cause damage to have any inteface pin at logical level high
  when VDDIO is switched off.
- To reset sensor, cycle VDD or send soft reset. Cycling VDDIO will
  not cause reset.

*/


/*
********** Serial Protocol Selection **********
I2C Interface: CSB connected to VDDIO 
SPI Interface: CSB pulled down

If CSB is pulled down once, I2C is disabled until 
next power-on-reset. 

In I2C mode, SDO Pin can be used to change address.
SDO = GND -> Address = 0x77
SDO = VDD -> Address = 0x76

*/


// Default I2C Address
#define BME280_I2C_ADDR_DEF       0x77
#define BME280_I2C_ADDR_ALT       0x76

// --- Calibration Data Registers (Read-only) ---
#define BME280_REG_CALIB00        0x88 // ... up to 0xA1 (26 bytes)
#define BME280_REG_CALIB26        0xE1 // ... up to 0xF0 (7 bytes)

// --- ID & Reset Register ---
#define BME280_REG_CHIP_ID        0xD0 // Typical value: 0x60
#define BME280_REG_RESET          0xE0 // Soft reset: write 0xB6

// --- Control Registers ---
#define BME280_REG_CTRL_HUM       0xF2 // Humidity oversampling
#define BME280_REG_STATUS         0xF3 // Status register
#define BME280_REG_CTRL_MEAS      0xF4 // Pressure/Temp oversampling + Mode
#define BME280_REG_CONFIG         0xF5 // IIR filter + Standby time

// --- Data Registers (Read-only) ---
// Data read in burst (0xF7 to 0xFE) covers all measurements
#define BME280_REG_PRESS_MSB      0xF7 // Pressure MSB
#define BME280_REG_PRESS_LSB      0xF8 // Pressure LSB
#define BME280_REG_PRESS_XLSB     0xF9 // Pressure XLSB
#define BME280_REG_TEMP_MSB       0xFA // Temperature MSB
#define BME280_REG_TEMP_LSB       0xFB // Temperature LSB
#define BME280_REG_TEMP_XLSB      0xFC // Temperature XLSB
#define BME280_REG_HUM_MSB        0xFD // Humidity MSB
#define BME280_REG_HUM_LSB        0xFE // Humidity LSB

#endif // BME_280_CONFIG_H