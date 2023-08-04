
//https://www.youtube.com/watch?v=mB7LsiscM78&list=PLfIJKC1ud8giTKW0nzHN71hud_238d-JO&index=10

#include <string.h>
#include <stdio.h>
#include <avr/io.h>

#include "uart.h"

#include "NRF24L01.h"
#include "UnoR3Pins.h"

#include "lcd.h"
#include <util/delay.h>

//#include "driver/spi_common.h"
//#include "driver/gpio.h"
//#include "driver/spi_master.h"
//#include "esp_log.h"
//#include "freertos/FreeRTOS.h"
//#include "freertos/task.h"

#define BIT_SET(a, b) ((a) |= (1ULL << (b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1ULL<<(b)))
#define BIT_FLIP(a,b) ((a) ^= (1ULL<<(b)))
#define BIT_CHECK(a,b) (!!((a) & (1ULL<<(b))))

//-------------------Common Methods for both RX and TX-------------------


void SPI_init(){

    BIT_SET(DDRB, PIN_NUM_CS); //Set CS to Output mode
    BIT_SET(PORTB, PIN_NUM_CS); // Set CS to HIGH

    BIT_SET(DDRB, PIN_NUM_CE); //Set CE to Output mode
    BIT_CLEAR(PORTB, PIN_NUM_CE); // Set CE to LOW

    BIT_CLEAR(DDRB, PIN_NUM_MISO); //Dont think these are needed as it will be set by the AVR libc's SPI lib by default i believe
    BIT_SET(DDRB, PIN_NUM_MOSI);
    BIT_SET(DDRB, PIN_NUM_CLK);

    /*SPCR = (0 << SPIE) |
	       (1 << SPE)  |
	       (0 << DORD) |
	       (1 << MSTR) |
	       (0 << CPOL) |
	       (0 << SPR1) | 
           (0 << SPR0);*/
    BIT_SET(SPCR, SPE); //Enable SPI
    BIT_SET(SPCR, MSTR); //Set Atmega328p as Master, if the clock frequenzy bits SPR1 and SPR0 are not changed clock frequency = fck/4 = 4Mhz
    //SPCR = _BV(SPE) | _BV(MSTR)| _BV(SPR0); // fck/16 (1 Mhz)
    BIT_CLEAR(SPCR, SPR0); // Clear SPR0 (clock frequency = fck/4)
    BIT_CLEAR(SPCR, SPR1); // Clear SPR1 (clock frequency = fck/4)

/*
    SPR0 = 0, SPR1 = 0: fck/4 (SPI clock frequency is the system clock divided by 4)
    SPR0 = 0, SPR1 = 1: fck/16 (SPI clock frequency is the system clock divided by 16)
    SPR0 = 1, SPR1 = 0: fck/64 (SPI clock frequency is the system clock divided by 64)
    SPR0 = 1, SPR1 = 1: fck/128 (SPI clock frequency is the system clock divided by 128)
*/
    //SPSR |= _BV(SPI2X); //Double SPI Clock speed

}

void CS_Select(){
    BIT_CLEAR(PORTB, PIN_NUM_CS);
}

void CS_Unselect(){
    BIT_SET(PORTB, PIN_NUM_CS);
}

/*void CE_Enable(){
    BIT_SET(DDRB, PIN_NUM_CE);
}

void CE_Disable(){
    BIT_CLEAR(DDRB, PIN_NUM_CE);
}*/

void CE_Enable(){
    BIT_CLEAR(DDRB, PIN_NUM_CE);
}

void CE_Disable(){
    BIT_SET(DDRB, PIN_NUM_CE);
}


void nrf24_WriteRegister(uint8_t reg, uint8_t data){//This method is used to write to a specific register, the NRF24L01 has registers to configure different settings for the NRF24L01.

    CE_Disable();

    reg = reg|1<<5; //In datasheet w_register says fifth bit needs to be a 1.
    CS_Select();

    /* Start transmission */
    SPDR = reg;

    /* Wait for transmission complete */
    while(!(SPSR & (1<<SPIF)));
    //loop_until_bit_is_set(SPSR, SPIF);

    //Next 8 bits, data contains the changes to be written to the registry
    SPDR = data;
    
    while(!(SPSR & (1<<SPIF)));
    //loop_until_bit_is_set(SPSR, SPIF);

    CS_Unselect();
    CE_Enable();
}

void nrf24_WriteRegisterMulti(uint8_t reg, uint8_t *data, int numberofBytes){//This method is used to write to s specific register, the NRF24L01 has registers to configure different settings for the NRF24L01.
    
    CE_Disable();

    reg = reg|1<<5;

    CS_Select();

    SPDR = reg;//select starting registry
    while(!(SPSR & (1<<SPIF)));

    //Send all the data bytes
    for (int i = 0; i < numberofBytes; i++)
    {
        SPDR = *data;
        while(!(SPSR & (1<<SPIF)));
        data++;
    }
    
    CS_Unselect();

    CE_Enable();
}


uint8_t NRF24_ReadReg(const uint8_t reg){

    CE_Disable();
    CS_Select();

    uint8_t response[2] = { 0 };

    SPDR = reg; //registry to read

    while(!(SPSR & (1<<SPIF)));
    //loop_until_bit_is_set(SPSR, SPIF);

    // Read the FIFO_STATUS
    response[0] = SPDR;
    //lcd_set_cursor(0,1);
    //lcd_printf("STATUS = %u", response[0]);

    SPDR = 0xFF; //Dummy bits that wont do anything. Just to trigger the NRF so it will send the registry data.

    while(!(SPSR & (1<<SPIF)));
    //loop_until_bit_is_set(SPSR, SPIF);

    response[1] = SPDR;

    //lcd_set_cursor(0,1);
    //lcd_printf("REGISTRY = %u", response[1]);
    CS_Unselect();
    CE_Enable();

    return SPDR; //After reg has been sent to the NRF24, NRF24 will send the reg status
}

void ReadRegMulti(uint8_t reg, uint8_t *data, int numberofBytes){
    CE_Disable();


    CS_Select();

    SPDR = reg;//select starting registry
    while(!(SPSR & (1<<SPIF)));

    for (int i = 0; i < numberofBytes; i++)
    {
        SPDR = 0xFF;
        
        while(!(SPSR & (1<<SPIF)));

        *data = SPDR;

        data++;
    }

    CS_Unselect();
    CE_Enable();

}

void nrfsendCmd (uint8_t cmd)
{   
    CE_Disable();

	// Pull the CS Pin LOW to select the device
	CS_Select();

    SPDR = cmd;
    while(!(SPSR & (1<<SPIF)));

	// Pull the CS HIGH to release the device
	CS_Unselect();
    CE_Enable();
}


void NRF24_Init(){

    CE_Disable();

    nrf24_WriteRegister(CONFIG, 0); //Will be configured later.

    uint8_t regValue = NRF24_ReadReg(CONFIG);

    //lcd_set_cursor(0,1);
    //lcd_printf("REGISTRY = %u", regValue);

    nrf24_WriteRegister(EN_AA, 0); //No auto-Acknowledgment

    nrf24_WriteRegister(EN_RXADDR, 0); //Will be configured later.

    nrf24_WriteRegister(SETUP_AW, 0x03);

    nrf24_WriteRegister(SETUP_RETR, 0); //Retransimssion disabled

    nrf24_WriteRegister(RF_CH, 0);

    nrf24_WriteRegister(RF_SETUP, 0x0E); //Power = 0dbm,  data rate = 2mbps

    CE_Enable();
}


//---------------------Transmitter Methods----------------------


void NRF24_TXMode(uint8_t *Address, uint8_t channel){ //put the NRF24L01 in TXMode

    CE_Disable();

    nrf24_WriteRegister(RF_CH, channel); //Choose a channel

    nrf24_WriteRegisterMulti(TX_ADDR, Address, 5); // Write the TX address

    uint8_t regValue = NRF24_ReadReg(TX_ADDR);

    //TESTING:
    uint8_t data[7];
    data[0] = 0;
    data[1] = 1;
    data[2] = 2;
    data[3] = 3;
    data[4] = 4;
    data[5] = 5;
    data[6] = 6;


    ReadRegMulti(TX_ADDR, &data, 5);


    //Power up the NRF24L01
    uint8_t config = NRF24_ReadReg(CONFIG); //Read the current settings.
    config = config | (1<<1);  //If not already a 1, change first bit to 1. That position will make the device power up.
    nrf24_WriteRegister(CONFIG, config); //The write it back
    //Doing it this way will prevent other bits from changing.


    config = NRF24_ReadReg(CONFIG); //TESTING
    CE_Enable();
}

uint8_t NRF24_Transmit(uint8_t *payload){

    uint8_t cmdToSend = W_TX_PAYLOAD; //this command tells the LRF24L01 that following this a payload will be sent.

    uint8_t buffer[33];

    CS_Select();


    CS_Unselect();

    //vTaskDelay(pdMS_TO_TICKS(1)); //Delay for the pin to settle

    uint8_t fifoStatus = NRF24_ReadReg(FIFO_STATUS); //Read fifo status to see if LRF24L01 properly received transmission.
                                                                        //FIFO = first-in-first-out    
    
    if((fifoStatus & (1<<4)) && (!(fifoStatus & (1<<3)))){
        cmdToSend = FLUSH_TX;
        nrfsendCmd(cmdToSend);
        return 1;
    }
    return 0;
}


//----------------Receiver methods--------------------


void NRF24_RXMode(uint8_t *Address, uint8_t channel){ //put the NRF24L01 in TXMode

    CE_Disable();

    //nrf24_WriteRegister(STATUS, 0x00);

    nrf24_WriteRegister(RF_CH, channel); //choose a channel
    //uint8_t rfch = NRF24_ReadReg(RF_CH);
    //lcd_set_cursor(0,0);
    //lcd_printf("rfch: %u", rfch);
    
    nrf24_WriteRegister(EN_RXADDR, 1);

    //uint8_t readreg = NRF24_ReadReg(EN_RXADDR);

    nrf24_WriteRegisterMulti(RX_ADDR_P0, Address, 5); // Write the RX address

    nrf24_WriteRegister(RX_PW_P0, 32); /// 32 bit payload size for pipe 0



    /* We must write the address for Data Pipe 1, if we want to use any pipe from 2 to 5
	 * The Address from DATA Pipe 2 to Data Pipe 5 differs only in the LSB
	 * Their 4 MSB Bytes will still be same as Data Pipe 1
	 *
	 * For Eg->
	 * Pipe 1 ADDR = 0xAABBCCDD11
	 * Pipe 2 ADDR = 0xAABBCCDD22
	 * Pipe 3 ADDR = 0xAABBCCDD33
	 *
	 */

     /*
    // select data pipe 2
	uint8_t en_rxaddr = NRF24_ReadReg(EN_RXADDR);
	en_rxaddr = en_rxaddr | (1<<2);
	nrf24_WriteRegister (EN_RXADDR, en_rxaddr);
    
    nrf24_WriteRegisterMulti(RX_ADDR_P1, Address, 5);  // Write the Pipe1 address
	nrf24_WriteRegister(RX_ADDR_P2, 0xEE);  // Write the Pipe2 LSB address

	nrf24_WriteRegister (RX_PW_P2, 32);   // 32 bit payload size for pipe 2
    */
    

    

    //Power up the NRF24L01 and set to RX mode
    uint8_t config = NRF24_ReadReg(CONFIG); //Read the current settings.
    config = config | (1<<1) | (1<<0);  //If not already a 1, change first bit to 1. That position will make the device power up.
    //config = 3;//TEST
    nrf24_WriteRegister(CONFIG, config); //Then write it back
    //Doing it this way will prevent other bits from changing.
    _delay_ms(20);

    CE_Enable();
}

uint8_t NRF24_RXisDataReady(int pipeNum){

    uint8_t statusReg = NRF24_ReadReg(STATUS);

    //Check if bit number 6 is 1 and that bit 1-3 matches pipeNum.
    if((statusReg & (1<<6))){//if((statusReg & (1<<6)) && (statusReg & (pipeNum<<1))){
        //nrf24_WriteRegister(STATUS, (1<<6));
        lcd_set_cursor(12,0);
        lcd_printf("Rece");
        _delay_ms(1000);
        return 1;
    }
    lcd_set_cursor(12,0);
    lcd_printf("Noth");
    //nrf24_WriteRegister(STATUS, (1<<4));
    //nrfsendCmd(FLUSH_RX);
    //_delay_ms(1000);
    return 0;
}


void NRF24_Receive(uint8_t *dataStorage){

    uint8_t cmdToSend = R_RX_PAYLOAD; //Read payload, payload will probably then be deleted in LRF24 automagicaly

    CS_Select();

    SPDR = cmdToSend;
    while(!(SPSR & (1<<SPIF)));

    for (int i = 0; i < 32; i++)
    {
        SPDR = 0xFF;
        while(!(SPSR & (1<<SPIF)));

        *dataStorage = SPDR;

        dataStorage++;
    }

    CS_Unselect();

    _delay_ms(1); //Delay for the pin to settle
    
    //nrf24_WriteRegister(STATUS, (1<<6));//TEST
    cmdToSend = FLUSH_RX;
    nrfsendCmd(cmdToSend);

    nrf24_WriteRegister(STATUS, (1<<6));
}










