#include "BMP280.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
//https://mansfield-devine.com/speculatrix/2018/02/avr-basics-using-the-i2c-bus-1-clock-speed/

//This union is used to hold all the calibration data.
static union bmp280_cal_union {
	uint8_t bytes[BMP280_CAL_DATA_BYTE_SIZE];
	struct {
		uint16_t dig_t1;
		int16_t  dig_t2;
		int16_t  dig_t3;
		uint16_t dig_p1;
		int16_t  dig_p2;
		int16_t  dig_p3;
		int16_t  dig_p4;
		int16_t  dig_p5;
		int16_t  dig_p6;
		int16_t  dig_p7;
		int16_t  dig_p8;
		int16_t  dig_p9;
	};
} bmp280_calibration;


void I2C_init() {
    

    // Initialize I2C with appropriate clock frequency
    // Set prescaler to 1 (TWPS0=0, TWPS1=0)
    TWSR = 0;

    // Set bit rate register (TWBR) for the desired I2C clock frequency
    // SCLfreq = F_CPU / (16 + (2 * TWBR * Prescaler) )
    // TWBR = ((F_CPU / 100000)   – 16) / (2 * Prescaler) (Here prescaler = 1)
    TWBR =    ((F_CPU / 100000UL) - 16) / (2*  1); // TWBR = 72, This means that we set the clock frequency to 100kHz.
    _delay_us(10);
}

uint32_t BMP280_init() {
    
    //1. Configure the BMP280
    uint8_t config = (STANDBY_TIME_1000_MS << T_SB) |
                     (_4_SAMPLES           << FILTER) |
                     0;

    BMP280_WriteRegister(CONFIG, config);

    uint8_t ctrl_measu = (OVERSAMPLING_X1 << OSRS_T) |
                         (OVERSAMPLING_X4 << OSRS_P) |
                         (MODE_NORMAL     << MODE);
                     
    BMP280_WriteRegister(CTRL_MEAS, ctrl_measu);

    memset(bmp280_calibration.bytes, 0, sizeof(bmp280_calibration)); //Put all bytes to 0.

    //2. Get calibration data that is unique for each sensor. Calibration data is used to calculate Pressure(pa), temperature and altitude.
    //Temperature calibration data
    bmp280_calibration.dig_t1 = (uint16_t)BMP280_ReadRegister(0x89) << 8 | (uint16_t)BMP280_ReadRegister(0x88);             //27624
    bmp280_calibration.dig_t2 = (int16_t)BMP280_ReadRegister(0x8B) << 8 | (int16_t)BMP280_ReadRegister(0x8A);               //26200
    bmp280_calibration.dig_t3 = (int16_t)BMP280_ReadRegister(0x8D) << 8 | (int16_t)BMP280_ReadRegister(0x8C);               //50

    //Pressure calibration data
    bmp280_calibration.dig_p1 = (uint16_t)BMP280_ReadRegister(0x8F)<< 8 | (uint16_t)BMP280_ReadRegister(0x8E);              //38039
    bmp280_calibration.dig_p2 = (int16_t)BMP280_ReadRegister(0x91) << 8 | (int16_t)BMP280_ReadRegister(0x90);               //-10519
    bmp280_calibration.dig_p3 = (int16_t)BMP280_ReadRegister(0x93) << 8 | (int16_t)BMP280_ReadRegister(0x92);               //3024
    bmp280_calibration.dig_p4 = (int16_t)BMP280_ReadRegister(0x95) << 8 | (int16_t)BMP280_ReadRegister(0x94);               //8068
    bmp280_calibration.dig_p5 = (int16_t)BMP280_ReadRegister(0x97) << 8 | (int16_t)BMP280_ReadRegister(0x96);               //-285
    bmp280_calibration.dig_p6 = (int16_t)BMP280_ReadRegister(0x99) << 8 | (int16_t)BMP280_ReadRegister(0x98);               //-7
    bmp280_calibration.dig_p7 = (int16_t)BMP280_ReadRegister(0x9B) << 8 | (int16_t)BMP280_ReadRegister(0x9A);               //8625
    bmp280_calibration.dig_p8 = (int16_t)BMP280_ReadRegister(0x9D) << 8 | (int16_t)BMP280_ReadRegister(0x9C);               //-6338
    bmp280_calibration.dig_p9 = (int16_t)BMP280_ReadRegister(0x9F) << 8 | (int16_t)BMP280_ReadRegister(0x9E);               //-225
    
    int32_t var1, var2, t_fine;

    int32_t PressureRaw = 0;
    //int32_t TemperatureRaw = ((int32_t)BMP280_ReadRegister(0xF7)<< 12) | ((int32_t)BMP280_ReadRegister(0xF8) << 4) | ((int32_t)BMP280_ReadRegister(0xF9) >> 4);
    PressureRaw =              ((int32_t)BMP280_ReadRegister(0xF7)<< 12) | ((int32_t)BMP280_ReadRegister(0xF8) << 4) | ((int32_t)BMP280_ReadRegister(0xF9) >> 4);

    int32_t TemperatureRaw = 0;
    TemperatureRaw |= (int32_t)BMP280_ReadRegister(0xFA) << 12;
    TemperatureRaw |= (int32_t)BMP280_ReadRegister(0xFB) << 4;
    TemperatureRaw |= (int32_t)BMP280_ReadRegister(0xFC) >> 4;

    var1 = ((((TemperatureRaw >> 3) - ((int32_t)bmp280_calibration.dig_t1 << 1)))
		* ((int32_t)bmp280_calibration.dig_t2)) >> 11;

	var2 = (((((TemperatureRaw >> 4) - ((int32_t)bmp280_calibration.dig_t1))
		* ((TemperatureRaw >> 4) - ((int32_t)bmp280_calibration.dig_t1))) >> 12)
		* ((int32_t)bmp280_calibration.dig_t3)) >> 14;
	t_fine = var1 + var2;
	int32_t bmp280_temp = (t_fine * 5 + 128) >> 8;
    

    int32_t something = bmp280_temp;


    // compute the pressure
	var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
	var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)bmp280_calibration.dig_p6);
	var2 = var2 + ((var1 * ((int32_t)bmp280_calibration.dig_p5)) << 1);
	var2 = (var2 >> 2) + (((int32_t)bmp280_calibration.dig_p4) << 16);
	var1 = (((bmp280_calibration.dig_p3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3)
		+ ((((int32_t)bmp280_calibration.dig_p2) * var1) >> 1)) >> 18;
	var1 = ((((32768 + var1)) * ((int32_t)bmp280_calibration.dig_p1)) >> 15);
    
    uint32_t bmp280_pressure = 0;

	if (var1 == 0) {
		bmp280_pressure = 0;
	} else {
		bmp280_pressure = (((uint32_t)(((int32_t)1048576)-PressureRaw)
			- (var2 >> 12))) * 3125;
		if (bmp280_pressure < 0x80000000) {
			bmp280_pressure = (bmp280_pressure << 1) / ((uint32_t)var1);
		} else {
			bmp280_pressure = (bmp280_pressure / (uint32_t)var1) * 2;
		}
		var1 = (((int32_t)bmp280_calibration.dig_p9) * ((int32_t)(((bmp280_pressure>>3) * (bmp280_pressure >> 3)) >> 13))) >> 12;
		var2 = (((int32_t)(bmp280_pressure >> 2)) * ((int32_t)bmp280_calibration.dig_p8)) >> 13;
		bmp280_pressure = (uint32_t)((uint32_t)bmp280_pressure + ((var1 + var2 + bmp280_calibration.dig_p7) >> 4));
	}
    

    return bmp280_pressure;
}



uint8_t BMP280_ReadRegister(uint8_t reg){
    uint8_t data = TWDR;

    // set the start condition
    TWCR = (1 << TWEN) |
            (1 << TWINT) |
            (1 << TWSTA);
    wait_for_completion;
    
    // send the BMP280 address
    TWDR = (BMP280_ADDRESS << 1) | I2C_WRITE;// (BMP280_ADDRESS << 1) | I2C_WRITE;      // SLA+W, address + write bit
    TWCR = (1 << TWEN) |
           (1 << TWINT);                   // trigger I2C action
    wait_for_completion;
    
    // Send Register memory address that we want to read
    TWDR = reg;                             
    TWCR = (1 << TWEN) | 
           (1 << TWINT);      
    wait_for_completion;
    

    // set another start condition
    TWCR = (1 << TWEN) | 
            (1 << TWINT) | 
            (1 << TWSTA);      
    wait_for_completion;
    
    //Send BMP280 address again but this time in read mode.
    TWDR = (BMP280_ADDRESS << 1) | I2C_READ;
	TWCR = (1<<TWINT) | (1<<TWEN);
	wait_for_completion;
    
    //With the following the BMP280 will send register data to TWDR.
    TWCR = ((1 << TWINT) | (1 << TWEN) | (0 << TWEA)); // without ACK set
    wait_for_completion;

    //Stop condition
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
    while(TWCR & (1<<TWSTO));
    
    return TWDR;
}

void BMP280_WriteRegister(uint8_t reg, uint8_t registerData){
    // set the start condition
    TWCR = ((1 << TWEN) |
            (1 << TWINT) |
            (1 << TWSTA));
    wait_for_completion;
    
    // send the address
    TWDR = (BMP280_ADDRESS << 1) | I2C_WRITE;      // SLA+W, address + write bit
    TWCR = ((1 << TWEN) |
           (1 << TWINT));                   // trigger I2C action
    wait_for_completion;
    
    // specify the register
    TWDR = reg;                             // register value in the data register
    TWCR = ((1 << TWEN) | 
           (1 << TWINT));                   // trigger I2C action
    wait_for_completion;

    // Write register data
    TWDR = registerData;                    // register data is written to the register
    TWCR = ((1 << TWEN) | 
           (1 << TWINT));                   // trigger I2C action
    wait_for_completion;

    // set stop condition
    TWCR = ((1 << TWINT) | (1 << TWEN) | (1 << TWSTO));

}











