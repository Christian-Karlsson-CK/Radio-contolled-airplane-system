
//https://www.youtube.com/watch?v=mB7LsiscM78&list=PLfIJKC1ud8giTKW0nzHN71hud_238d-JO&index=10

#include <string.h>
#include <stdio.h>

#include "NRF24L01.h"
//#include "hal/spi_types.h"
#include "driver/spi_common.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"



//NRF24L01 PINS
#define SPIHOST VSPI_HOST //Using VSPI on ESP32 SPIHOST = The SPI controller peripheral inside ESP32. VSPI_HOST = SPI3_HOST=2
#define PIN_NUM_MISO 19   // SPI PINS
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   17    //To the CSN pin on the LRF24L01+. used to select the specific SPI device with which the ESP32 wants to communicate.
#define PIN_NUM_CE   16   //used to control the NRF24L01 module's operation mode.

void SPI_init(spi_device_handle_t*);


//extern SPI_HandleTypeDef hspi1;
//#define NRF24_SPI &hspi1

static const char *TAG = "TESTING:";


//-------------------Common Methods for both RX and TX-------------------


void SPI_init(spi_device_handle_t *spi_device_handle){
    //gpio_pad_select_gpio(PIN_NUM_CS); //Method maybe not in use anymore.
    gpio_set_direction(PIN_NUM_CS, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_NUM_CS, 1);
    gpio_set_direction(PIN_NUM_CE, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_NUM_CE, 0);
    
    esp_err_t ret; //Used to represent error codes returned by various functions and operations

    spi_bus_config_t buscfg={
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        //.max_transfer_sz = 32,
    };

    //Initialize the SPI bus
    ret = spi_bus_initialize(SPIHOST, &buscfg, SPI_DMA_CH_AUTO); //SPI_DMA_CH_AUTO
    //ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "Initializing bus SPI%d...", SPIHOST+1);
    assert(ret==ESP_OK);
    

    /*if (ret == ESP_OK) {
        ESP_LOGI(TAG, "could initSPI"); //TESTING
    } else {
        ESP_LOGI(TAG, "could not initSPI"); //TESTING
    }*/

    spi_device_interface_config_t dev_config = { //device config
    //.command_bits = 0,
    //.address_bits = 0,
    //.dummy_bits = 0,
    .mode = 0,
    //.duty_cycle_pos = 0,
    //.cs_ena_pretrans = 0,
    //.cs_ena_posttrans = 0,
    .clock_speed_hz = 4000000, // 4 MHz clock speed
    //.input_delay_ns = 0,
    //.spics_io_num = PIN_NUM_CS,
    .spics_io_num = -1,
    .queue_size = 7,
    //.pre_cb = NULL,
    //.post_cb = NULL,
    .flags = SPI_DEVICE_NO_DUMMY,
    };
    
    ret = spi_bus_add_device(SPIHOST, &dev_config, spi_device_handle);
    
	assert(ret==ESP_OK);

}

void CS_Select(){
    gpio_set_level(PIN_NUM_CS, 0);
}

void CS_Unselect(){
    gpio_set_level(PIN_NUM_CS, 1);
}

void CE_Enable(){
    gpio_set_level(PIN_NUM_CE, 1);
}

void CE_Disable(){
    gpio_set_level(PIN_NUM_CE, 0);
}


void nrf24_WriteRegister(uint8_t reg, uint8_t data, spi_device_handle_t *spi_device_handle){//This method is used to write to a specific register, the NRF24L01 has registers to configure different settings for the NRF24L01.

    CE_Disable();

    esp_err_t ret;
    uint8_t buffer[2];
    buffer[0] = reg|1<<5; //In datasheet w_register says fifth bit needs to be a 1.
    buffer[1] = data;
    
    //ESP_LOGI(TAG, "in writeReg: buffer before: buffer[0]= %u, buffer[1]= %u", buffer[0], buffer[1]); //TESTING

    spi_transaction_t trans = {
        .length = 16,  // Length in bits
        .tx_buffer = &buffer,
        .rx_buffer = NULL,
        .user = NULL,
    };

    CS_Select();

    ret = spi_device_transmit(*spi_device_handle, &trans);
    //ESP_LOGI(TAG, "spi_device_transmit: %d", ret); //TESTING
    //ESP_LOGI(TAG, "WRITE REG buffer[0]: %d", buffer[0]); //TESTING
    //ESP_LOGI(TAG, "WRITE DATA buffer[1]: %d", buffer[1]); //TESTING

    CS_Unselect();
    CE_Enable();
}

void nrf24_WriteRegisterMulti(uint8_t reg, uint8_t *data, spi_device_handle_t *spi_device_handle, int numberofBytes){//This method is used to write to s specific register, the NRF24L01 has registers to configure different settings for the NRF24L01.
    
    CE_Disable();

    esp_err_t ret;
    uint8_t buffer[1+numberofBytes];
    buffer[0] = reg|1<<5;
    memcpy(&buffer[1], data, numberofBytes);

    /*
    spi_transaction_t trans[2] = {
        {
        .length = 8,  // Length in bits
        .tx_buffer = &buffer,
        .rx_buffer = NULL,
        .user = NULL,
        },
        {
        .length = 8 * numberofBytes,  // Length in bits
        .tx_buffer = data,
        .rx_buffer = NULL,
        .user = NULL,
        }
    };*/

    spi_transaction_t trans = {
        .length = 8 + 8 * numberofBytes,  // Length in bits
        .tx_buffer = &buffer,
        .rx_buffer = NULL,
        .user = NULL,
    };

    CS_Select();

    ret = spi_device_transmit(*spi_device_handle, &trans);
    //ESP_LOGI(TAG, "WriteRegisterMulti:"); //TESTING
    //ESP_LOGI(TAG, "spi_device_transmit: %d", ret); //TESTING

    CS_Unselect();

    CE_Enable();
}


uint8_t NRF24_ReadReg(uint8_t reg, spi_device_handle_t *spi_device_handle){

    CE_Disable();

    uint8_t data[1];
    data[0] = 0;
    data[1] = 11;
    
    //ESP_LOGI(TAG, "reg BEFORE: %u", reg); //TESTING
	//ESP_LOGI(TAG, "data[0]  BEFORE: %u", data[0]); //TESTING
    //ESP_LOGI(TAG, "data[1]  BEFORE: %u", data[1]); //TESTING
    spi_transaction_t trans = {
        .length = 16,  // Length in bits
        //.rxlength = 16,
        .tx_buffer = &reg,
        .rx_buffer = &data,
        .user = NULL,
    };

    CS_Select();

    

    esp_err_t ret = spi_device_transmit(*spi_device_handle, &trans);
    ESP_ERROR_CHECK(ret);
    //ESP_LOGI(TAG, "reg AFTER: %u", reg); //TESTING
    //ESP_LOGI(TAG, "data[0]  AFTER: %u", data[0]); //TESTING
    //ESP_LOGI(TAG, "data[1]  AFTER: %u", data[1]); //TESTING

    CS_Unselect();
    CE_Enable();

    return data[1];
}

void ReadRegMulti(uint8_t reg, uint8_t *data, int numberofBytes, spi_device_handle_t *spi_device_handle){
    CE_Disable();

    /* Not working!
    spi_transaction_t trans[2] = {
        {
            .length = 8,  // Length in bits of reg
            .tx_buffer = &reg,
            .rx_buffer = NULL,
            .user = NULL,
        },
        {
            .length = 8 * numberofBytes,  // Length in bits of data
            .tx_buffer = NULL,
            .rx_buffer = data,
            .user = NULL,
        }
    };*/

    // Alternative transaction
    spi_transaction_t trans = {
        .length = 8 + 8 *numberofBytes,  // Length in bits of register then length of data bits
        .tx_buffer = &reg,               // the length is probably the total length in bits of either send or receive way.
        .rx_buffer = data,               // we have the STATUS reg (8 bits) that ALWAYS gets sent first no matter the command given.   
        .user = NULL,                    // then we have the bytes we want to read.
    };
    


    CS_Select();

    esp_err_t ret = spi_device_transmit(*spi_device_handle, &trans);
    ESP_ERROR_CHECK(ret);


    ESP_LOGI(TAG, "ReadMulti:"); //TESTING
    ESP_LOGI(TAG, "spi_device_transmit: %d", ret); //TESTING
    ESP_LOGI(TAG, "WRITE REG:            %u", reg); //TESTING
    ESP_LOGI(TAG, "WRITE DATA buffer[0]: %u", data[0]); //TESTING
    ESP_LOGI(TAG, "WRITE DATA buffer[1]: %u", data[1]); //TESTING
    ESP_LOGI(TAG, "WRITE DATA buffer[2]: %u", data[2]); //TESTING
    ESP_LOGI(TAG, "WRITE DATA buffer[3]: %u", data[3]); //TESTING
    ESP_LOGI(TAG, "WRITE DATA buffer[4]: %u", data[4]); //TESTING
    ESP_LOGI(TAG, "WRITE DATA buffer[5]: %u", data[5]); //TESTING
    ESP_LOGI(TAG, "WRITE DATA buffer[6]: %u", data[6]); //TESTING

    CS_Unselect();
    CE_Enable();

    //return data;
}

void nrfsendCmd (uint8_t cmd, spi_device_handle_t *spi_device_handle)
{   
    CE_Disable();

    spi_transaction_t trans = {
        .length = 8,  // Length in bits
        .tx_buffer = &cmd,
        .rx_buffer = NULL,
        .user = NULL,
    };

	// Pull the CS Pin LOW to select the device
	CS_Select();

	esp_err_t ret = spi_device_transmit(*spi_device_handle, &trans);
    ESP_ERROR_CHECK(ret);

	// Pull the CS HIGH to release the device
	CS_Unselect();
    CE_Enable();
}


void NRF24_Init(uint8_t *TX_Address,uint8_t *RX_Address, uint8_t channel, spi_device_handle_t *spi_device_handle){

    CE_Disable();
    
    nrf24_WriteRegister(CONFIG, 0, spi_device_handle); //Will be configured later.

    nrf24_WriteRegister(EN_AA, 0, spi_device_handle); //No auto-Acknowledgment

    //nrf24_WriteRegister(EN_RXADDR, 0, spi_device_handle); //Will be configured later.

    nrf24_WriteRegister(SETUP_AW, 0x03, spi_device_handle); //5bytes for address (setup address width)

    nrf24_WriteRegister(SETUP_RETR, 0, spi_device_handle); //Retransimssion disabled

    nrf24_WriteRegister(RF_SETUP, 0x0E, spi_device_handle); //Power = 0dbm,  data rate = 2mbps

    nrf24_WriteRegister(RF_CH, channel, spi_device_handle); //choose a channel
    
    nrf24_WriteRegister(EN_RXADDR, 1,spi_device_handle);

    nrf24_WriteRegisterMulti(RX_ADDR_P0, RX_Address, spi_device_handle, 5); // Write the RX address

    nrf24_WriteRegister(RX_PW_P0, 32, spi_device_handle); /// 32 bit payload size for pipe 0

    nrf24_WriteRegisterMulti(TX_ADDR, TX_Address, spi_device_handle, 5); // Write the TX address

    CE_Enable();
}


//---------------------Transmitter Methods----------------------


void NRF24_TXMode(spi_device_handle_t *spi_device_handle){ //put the NRF24L01 in TXMode

    CE_Disable();

    //nrf24_WriteRegister(RF_CH, channel, spi_device_handle); //Choose a channel

    //nrf24_WriteRegisterMulti(TX_ADDR, Address, spi_device_handle, 5); // Write the TX address

    //Power up the NRF24L01
    uint8_t config = NRF24_ReadReg(CONFIG, spi_device_handle); //Read the current settings.
    //config = config | (1<<1) |;  //If not already a 1, change first bit to 1. That position will make the device power up.
    config = 114;
    nrf24_WriteRegister(CONFIG, config, spi_device_handle); //The write it back
    //Doing it this way will prevent other bits from changing.
    vTaskDelay(pdMS_TO_TICKS(2));

    CE_Enable();
    vTaskDelay(pdMS_TO_TICKS(1));
}

uint8_t NRF24_Transmit(uint8_t *payload, spi_device_handle_t *spi_device_handle){
    CE_Enable();
    uint8_t cmdToSend = W_TX_PAYLOAD; //this command tells the LRF24L01 that following this a payload will be sent.
    
    uint8_t buffer[33];
    memcpy(&buffer[1], payload, 32);
    buffer[0] = cmdToSend;
    //for (size_t i = 1; i < 32; i++)
    //{
    //    buffer[i] = payload[i];
    //}
    

    //memcpy(&buffer[0], &cmdToSend, 1);
    //memcpy(&buffer[1], payload, 32);
    //strncpy((char*)&buffer[1], payload, 32);
    //ESP_LOGI(TAG, "Y %u, X %u", buffer[2], buffer[3]); //TESTING*/
    //buffer[0] = cmdToSend;
    

    spi_transaction_t trans = {
        .length = 8*33,  // Length in bits of command cmd is 1byte the payload is 32 byte
        .tx_buffer = &buffer, //Command to send
        .rx_buffer = NULL,
        .user = NULL,
    };

    CS_Select();

	esp_err_t ret = spi_device_transmit(*spi_device_handle, &trans);
    ESP_ERROR_CHECK(ret);
    //ESP_LOGI(TAG, "Transmit:"); //TESTING
    //ESP_LOGI(TAG, "spi_device_transmit: %d", ret); //TESTING*/
    //ESP_LOGI(TAG, "%s", buffer); //TESTING*/
    

/*
        spi_transaction_t trans = {
        .length = 8*1,  // Length in bits of command cmd is 1byte the payload is 32 byte
        .tx_buffer = &cmdToSend, //Command to send
        .rx_buffer = NULL,
        .user = NULL,
    };

    esp_err_t ret = spi_device_transmit(*spi_device_handle, &trans);
    ESP_LOGI(TAG, "spi_device_transmit command: %d", ret); //TESTING

    vTaskDelay(pdMS_TO_TICKS(20));

    spi_transaction_t trans2 = {
        .length = 8*32,  // Length in bits of command cmd is 1byte the payload is 32 byte
        .tx_buffer = payload, //Command to send
        .rx_buffer = NULL,
        .user = NULL,
    };

    ret = spi_device_transmit(*spi_device_handle, &trans2);
    ESP_LOGI(TAG, "spi_device_transmit payload: %d", ret); //TESTING*/
    vTaskDelay(pdMS_TO_TICKS(20));

    uint8_t fifoStatus = NRF24_ReadReg(FIFO_STATUS, spi_device_handle);
    //ESP_LOGI(TAG, "FIFO STATUS BEFORE RESET: %u", fifoStatus); //TESTING
    CS_Unselect();

    vTaskDelay(pdMS_TO_TICKS(1)); //Delay for the pin to settle

    //ESP_LOGI(TAG, "READ FIFO STATUS AFTER TRANSMIT:"); //TESTING
    fifoStatus = NRF24_ReadReg(FIFO_STATUS, spi_device_handle); //Read fifo status to see if LRF24L01 properly received transmission.
                                                                        //FIFO = first-in-first-out    
    //fifoStatus = NRF24_ReadReg(FIFO_STATUS, spi_device_handle);
    //ESP_LOGI(TAG, "FIFO STATUS BEFORE RESET: %u", fifoStatus); //TESTING
    if((fifoStatus & (1<<4)) && (!(fifoStatus & (1<<3)))){
        //ESP_LOGI(TAG, "FIFO STATUS READ OK:"); //TESTING
        cmdToSend = FLUSH_TX;
        nrfsendCmd(cmdToSend, spi_device_handle);

        // reset FIFO_STATUS
		nrf24_WriteRegister(FIFO_STATUS, 0x00, spi_device_handle);

        //fifoStatus = NRF24_ReadReg(FIFO_STATUS, spi_device_handle);
        //ESP_LOGI(TAG, "FIFO STATUS RESET(MUST BE 17): %u", fifoStatus); //TESTING

        //fifoStatus = NRF24_ReadReg(STATUS, spi_device_handle);
        //ESP_LOGI(TAG, "STATUS: %u", fifoStatus); //TESTING

        return 1;
    }
    return 0;
}


//----------------Receiver methods--------------------

void NRF24_RXMode(spi_device_handle_t *spi_device_handle){

    CE_Disable();

    //Power up the NRF24L01 and set to RX mode
    uint8_t config = NRF24_ReadReg(CONFIG, spi_device_handle); //Read the current settings.
    config = config | (0<<PWR_UP);  //Power down before changing registers.
    nrf24_WriteRegister(CONFIG, config, spi_device_handle); //Then write it back
    config = config | (1<<PWR_UP) | (1<<PRIM_RX); //power up and in correct mode.
    nrf24_WriteRegister(CONFIG, config, spi_device_handle);
    vTaskDelay(pdMS_TO_TICKS(2));

    CE_Enable();
    vTaskDelay(pdMS_TO_TICKS(1));
}

uint8_t NRF24_RXisDataReady(int pipeNum ,spi_device_handle_t *spi_device_handle){
    uint8_t statusReg = NRF24_ReadReg(FIFO_STATUS, spi_device_handle);
    ESP_LOGI(TAG, "FIFO: %u", statusReg);

    statusReg = NRF24_ReadReg(STATUS, spi_device_handle);
    ESP_LOGI(TAG, "STATUS: %u", statusReg);



    //Check if bit number 6 is 1 and that bit 1-3 matches pipeNum.
    if(statusReg & (1<<RX_DR)){//if((statusReg & (1<<RX_DR)) && (statusReg & (pipeNum<<RX_P_NO))){
        nrf24_WriteRegister(STATUS, (1<<RX_DR), spi_device_handle); //Write 1 to clear bit.
        return 1;
    }
    return 0;
}


void NRF24_Receive(uint8_t *dataStorage, spi_device_handle_t *spi_device_handle){

    uint8_t cmdToSend = R_RX_PAYLOAD;

    spi_transaction_t trans = {
        .length = 8*33,  // Length in bits of command
        .tx_buffer = &cmdToSend, //Command to send
        .rx_buffer = dataStorage,
        .user = NULL,
    };

    CS_Select();
	esp_err_t ret = spi_device_transmit(*spi_device_handle, &trans);
    ESP_ERROR_CHECK(ret);
    CS_Unselect();

    vTaskDelay(pdMS_TO_TICKS(1)); //Delay for the pin to settle

    //cmdToSend = FLUSH_RX;
    nrfsendCmd(FLUSH_RX, spi_device_handle);
}










