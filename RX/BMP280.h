#ifndef _BMP280_H_
#define _BMP280_H_

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include "TxData.h"

void I2C_init();

void BMP280_init();

uint8_t BMP280_ReadRegister(uint8_t register);
void BMP280_WriteRegister(uint8_t register, uint8_t registerData);

void  BMP280_ReadTempAndPressure(uint8_t *TxData);

#define BMP280_ADDRESS 0x77

#define BMP280_CAL_DATA_BYTE_SIZE 26

#define wait_for_completion while(!(TWCR & (1 << TWINT)))
#define I2C_WRITE 0
#define I2C_READ  1

//Register memory address
#define TEMP_XLSB  0xFC
#define TEMP_LSB   0xFB
#define TEMP_MSB   0xFA
#define PRESS_XLSB 0xF9
#define PRESS_LSB  0xF8
#define PRESS_MSB  0xF7
#define CONFIG     0xF5
#define CTRL_MEAS  0xF4
#define STATUS     0xF3
#define RESET      0xE0
#define ID         0xD0
#define CALIB25    0xA1
#define CALIB00    0x88 //CALIB 00-25 I guess each consecutive register is incremented by 1 all the way to CALIB25

//Config register
#define T_SB       0x05 //[7:5] 3 BITS
#define FILTER     0x02 //[4:2] 3 BITS
#define SPI3W_EN   0x00

//T_SB settings
#define STANDBY_TIME_0_5_MS     0x00 // 0.5 ms standby time
#define STANDBY_TIME_62_5_MS    0x01 // 62.5 ms standby time
#define STANDBY_TIME_125_MS     0x02 // 125 ms standby time
#define STANDBY_TIME_250_MS     0x03 // 250 ms standby time
#define STANDBY_TIME_500_MS     0x04 // 500 ms standby time
#define STANDBY_TIME_1000_MS    0x05 // 1000 ms standby time
#define STANDBY_TIME_2000_MS    0x06 // 2000 ms standby time
#define STANDBY_TIME_4000_MS    0x07 // 4000 ms standby time

//Filtering settings
#define NO_FILTERING  0x00
#define _2_SAMPLES    0x01
#define _4_SAMPLES    0x02
#define _8_SAMPLES    0x03
#define _16_SAMPLES   0x04


//CTRL_MEAS register
#define OSRS_T     0x05 //[7:5] 3 BITS
#define OSRS_P     0x02 //[4:2] 3 BITS
#define MODE       0x00 //[1:0] 2 BITS

//OSRS_T & OSRS_P settings
#define NO_OVERSAMPLING  0x00 // Disabled (No oversampling, 16-bit resolution)
#define OVERSAMPLING_X1  0x01 // (17-bit resolution)
#define OVERSAMPLING_X2  0x02 // (18-bit resolution)
#define OVERSAMPLING_X4  0x03 // (19-bit resolution)
#define OVERSAMPLING_X8  0x04 // (20-bit resolution)
#define OVERSAMPLING_X16 0x05 // (21-bit resolution)

//MODE settings
#define MODE_SLEEP   0x00
#define MODE_FORCED  0x01
#define MODE_NORMAL  0x03





#endif /* _BMP280_H_ */