#include "GY_271.h"


void GY271_init() {

       GY271_WriteRegister(GY271_SET_RESET_PERIOD_REG, 0x01); //Recommended by datasheet

       uint8_t control_1 =  (SAMPLE_512 << OSR) |
                            (G_8        << RNG) |
                            (HZ_10      << ODR) |
                            (CONTINOUS  << MODE);

       GY271_WriteRegister(GY271_CONTROL1_REG ,control_1);

       uint8_t control_2 =  (NO_RESET            << SOFT_RST) |
                            (ROLL_POINTER_ENABLE << ROL_PNT)  |
                            (INTERRUPT_DISABLE   << INT_ENB);
                            
       GY271_WriteRegister(GY271_CONTROL2_REG ,control_2);
      
}



uint8_t GY271_ReadRegister(uint8_t reg){
    uint8_t data = TWDR;
    // set the start condition
    TWCR = (1 << TWEN) |
            (1 << TWINT) |
            (1 << TWSTA);
    wait_for_completion;
    
    // send the BMP280 address
    TWDR = (GY271_ADDRESS << 1) | I2C_WRITE;// (BMP280_ADDRESS << 1) | I2C_WRITE;      // SLA+W, address + write bit
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
    
    //Send GY271 address again but this time in read mode.
    TWDR = (GY271_ADDRESS << 1) | I2C_READ;
	TWCR = (1<<TWINT) | (1<<TWEN);
	wait_for_completion;
    
    //With the following the GY will send register data to TWDR.
    TWCR = ((1 << TWINT) | (1 << TWEN) | (0 << TWEA)); // without ACK set
    wait_for_completion;

    //Stop condition
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
    while(TWCR & (1<<TWSTO));
    
    return TWDR;
}

void GY271_WriteRegister(uint8_t reg, uint8_t registerData){
    // set the start condition
    TWCR = ((1 << TWEN) |
            (1 << TWINT) |
            (1 << TWSTA));
    wait_for_completion;
    
    // send the address
    TWDR = (GY271_ADDRESS << 1) | I2C_WRITE;      // SLA+W, address + write bit
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

void i2c_start(uint8_t address){
	// set the start condition
    TWCR = ((1 << TWEN) |
            (1 << TWINT) |
            (1 << TWSTA));
    wait_for_completion;
    
    // send the address
    TWDR = address;      // SLA+W, address + write bit
    TWCR = ((1 << TWEN) |
           (1 << TWINT));                   // trigger I2C action
    wait_for_completion;
}


void i2c_write(uint8_t data){
	// Send Register memory address that we want to read OR data we want to write to a register
    TWDR = data;                             
    TWCR = (1 << TWEN) | 
           (1 << TWINT);      
    wait_for_completion;

}
void i2c_stop(){
	 // set stop condition
    TWCR = ((1 << TWINT) | (1 << TWEN) | (1 << TWSTO));
	while(TWCR & (1<<TWSTO));
}

uint8_t i2c_readNak(){ //Used when reading multiple registers

	TWCR = (1<<TWINT) | (1<<TWEN);
	wait_for_completion;
	return TWDR;

}

uint8_t i2c_readAck(){ //Used when reading multiple registers
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	wait_for_completion;

	return TWDR;
}



void GY271_ReadXAndY(uint8_t *TxData){
	int16_t heading;
	int16_t x,y,z;
	uint8_t gy271_DataBuffer[6];

	//Select register to start read from:
	i2c_start((GY271_ADDRESS << 1) | I2C_WRITE);
	i2c_write(GY271_X_LSB_REG);
	i2c_stop();

	//Start read:
	i2c_start((GY271_ADDRESS << 1) | I2C_READ);

	for(uint8_t i=0; i<6; i++) {
		gy271_DataBuffer[i] = (i==5)?i2c_readNak():i2c_readAck(); //Reads with ACK until i==5 then NOACK
	}
	i2c_stop();

	x = (int16_t)gy271_DataBuffer[1] << 8 | (int16_t)gy271_DataBuffer[0];
	y = (int16_t)gy271_DataBuffer[3] << 8 | (int16_t)gy271_DataBuffer[2];
	z = (int16_t)gy271_DataBuffer[5] << 8 | (int16_t)gy271_DataBuffer[4]; //Currently not in use

    //heading = GY271_CalcHeading(x, y);

	//Add heading to transmit buffer.
    //TxData[15] = ((int16_t)heading >> 0);

    TxData[15] = ((int16_t)x >> 0);
    TxData[16] = ((int16_t)x >> 8);

    TxData[17] = ((int16_t)y >> 0);
    TxData[18] = ((int16_t)y >> 8);

}

int16_t GY271_CalcHeading(int16_t x, int16_t y)
{      
       float heading = atan2((double)y,(double)x)*180.0/M_PI;
       heading = (heading < 0)? 360 + heading:heading;
       
	return (int16_t)heading;
}






















