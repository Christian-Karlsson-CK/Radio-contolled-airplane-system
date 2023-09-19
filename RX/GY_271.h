#ifndef _GY_271_H_
#define _GY_271_H_

#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

void GY271_init();

uint8_t GY271_ReadRegister(uint8_t register);
void GY271_WriteRegister(uint8_t register, uint8_t registerData);

void i2c_start(uint8_t address);
void i2c_write(uint8_t register);
void i2c_stop();
//i2c_readNak();
//i2c_readAck();

void GY271_ReadXAndY(uint8_t *TxData);
int16_t GY271_CalcHeading(int16_t x, int16_t y);

#define GY271_ADDRESS 0x0D

#define wait_for_completion while(!(TWCR & (1 << TWINT)))
#define I2C_WRITE 0
#define I2C_READ  1

//Register memory address
#define GY271_X_LSB_REG             0x00
#define GY271_X_MSB_REG             0x01
#define GY271_Y_LSB_REG             0x02
#define GY271_Y_MSB_REG             0x03
#define GY271_Z_LSB_REG             0x04
#define GY271_Z_MSB_REG             0x05
#define GY271_STATUS_REG            0x06
#define GY271_TOUT_LSB_REG          0x07
#define GY271_TOUT_MSB_REG          0x08
#define GY271_CONTROL1_REG          0x09
#define GY271_CONTROL2_REG          0x0A
#define GY271_SET_RESET_PERIOD_REG  0x0B
#define GY271_ID_REG                0x0D

//Status register
#define DOR    2 //Data skipped for reading
#define OVL    1 //Data overflow
#define DRDY   0 //New data is ready

//Config 1 register
#define OSR    6 //[7:6] over sampling rate 
#define RNG    4 //[5:4] magnetic field measurement range or sensitivity of the sensors
#define ODR    2 //[3:2] output data update rate 
#define MODE   0 //[1:0] operational modes

//MODES
#define STANDBY   0
#define CONTINOUS 1

//ODR, OUTPUT DATA RATES
#define HZ_10  0
#define HZ_50  1
#define HZ_100 2
#define HZ_200 3

//RNG, FULL SCALE
#define G_2  0
#define G_8  1

//OSR, OVER SMAPLE RATIO
#define SAMPLE_512  0
#define SAMPLE_256  1
#define SAMPLE_128  2
#define SAMPLE_64   3



//Config 2 register
#define SOFT_RST  7 //Data skipped for reading
#define ROL_PNT   6 //Data overflow
#define INT_ENB   0 //New data is ready

//INT_ENB, INTERRUPT PIN
#define INTERRUPT_ENABLE   0
#define INTERRUPT_DISABLE  1

//ROL_PNT, Pointer roll-over function
#define ROLL_POINTER_DISABLE 0
#define ROLL_POINTER_ENABLE  1

//SOFT_RST, SOFT RESET
#define NO_RESET 0
#define RESET    1



#endif /* _GY_271_H_ */