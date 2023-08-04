#ifndef _NRF24L01_H_
#define _NRF24L01_H_

#include "driver/spi_master.h"



//Common
void SPI_init(spi_device_handle_t *spi_handle);

void CS_Select();
void CS_Unselect();
void CE_Enable();
void CE_Disable();

void nrf24_WriteRegister(uint8_t, uint8_t, spi_device_handle_t*);
void nrf24_WriteRegisterMulti(uint8_t, uint8_t*, spi_device_handle_t*, int);

uint8_t NRF24_ReadReg(uint8_t reg, spi_device_handle_t *spi_device_handle);
void ReadRegMulti(uint8_t, uint8_t*, int, spi_device_handle_t*);

void nrfsendCmd (uint8_t, spi_device_handle_t*);

void NRF24_Init(spi_device_handle_t*);

//TX
void NRF24_TXMode(uint8_t *Address, uint8_t channel, spi_device_handle_t *spi_device_handle);
uint8_t NRF24_Transmit(uint8_t *payload, spi_device_handle_t* spi_handle);

//RX
void NRF24_RXMode(uint8_t *Address, uint8_t channel, spi_device_handle_t *spi_device_handle);
uint8_t NRF24_RXisDataReady(int pipeNum ,spi_device_handle_t *spi_device_handle);
void NRF24_Receive(uint8_t *dataStorage, spi_device_handle_t *spi_device_handle);


//void NRF24_Init (void);

//void NRF24_TxMode (uint8_t *Address, uint8_t channel);
//uint8_t NRF24_Transmit (uint8_t *data);

//void NRF24_RxMode (uint8_t *Address, uint8_t channel);
//uint8_t isDataAvailable (int pipenum);
//void NRF24_Receive (uint8_t *data);

//void NRF24_ReadAll (uint8_t *data);

/* Memory Map */
#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define CD          0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD	    0x1C
#define FEATURE	    0x1D

/* Instruction Mnemonics */
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define ACTIVATE      0x50
#define R_RX_PL_WID   0x60
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define W_TX_PAYLOAD_NOACK	0b10110000
#define W_ACK_PAYLOAD 0xA8
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define NOP           0xFF


#endif /* _NRF24L01_H_ */




