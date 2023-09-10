#include "BMP280.h"

//https://mansfield-devine.com/speculatrix/2018/02/avr-basics-using-the-i2c-bus-1-clock-speed/

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

void BMP280_init() {
    
    uint8_t config = (STANDBY_TIME_1000_MS << T_SB) |
                     (_4_SAMPLES           << FILTER);

    BMP280_WriteRegister(CONFIG, config);

    uint8_t ctrl_measu = (OVERSAMPLING_X1 << OSRS_T) |
                         (OVERSAMPLING_X4 << OSRS_P) |
                         (MODE_NORMAL     << MODE);
                     
    BMP280_WriteRegister(CTRL_MEAS, ctrl_measu);
    
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

void BMP280_Start(void){



}

float BMP280_GetTemp(void){


    
}

float BMP280_GetPress(int oss){


    
}

float BMP280_GetAlt(int oss){


    
}























