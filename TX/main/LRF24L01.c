
//https://www.youtube.com/watch?v=mB7LsiscM78&list=PLfIJKC1ud8giTKW0nzHN71hud_238d-JO&index=10

#include "LRF24L01.h"
#include "hal/spi_types.h"
#include "driver/spi_common.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

#define SPIHOST VSPI_HOST //Using VSPI on ESP32 SPIHOST = The SPI controller peripheral inside ESP32. VSPI_HOST = SPI3_HOST=2
#define PIN_NUM_MISO 19   // SPI PINS
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5    //To the CSN pin on the LRF24L01+. used to select the specific SPI device with which the ESP32 wants to communicate.
#define PIN_NUM_CE   17   //used to control the NRF24L01 module's operation mode.

//extern SPI_HandleTypeDef hspi1;
//#define NRF24_SPI &hspi1

static const char *TAG = "example";

void SPI_init(spi_device_handle_t*);



void SPI_init(spi_device_handle_t *spi_device_handle){
    gpio_pad_select_gpio(PIN_NUM_CS); //Method maybe not in use anymore.
    gpio_set_direction(PIN_NUM_CS, GPIO_MODE_OUTPUT);
    
    esp_err_t ret; //Used to represent error codes returned by various functions and operations
    //spi_bus_handle_t spi_bus_handle; //Might not be needed?
    //spi_device_handle_t spi_device_handle;

    ESP_LOGI(TAG, "Initializing bus SPI%d...", SPIHOST+1);

    spi_bus_config_t buscfg={
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };

    //Initialize the SPI bus
    ret = spi_bus_initialize(SPIHOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    spi_device_interface_config_t dev_config = { //device config
    .command_bits = 0,
    .address_bits = 8,
    .dummy_bits = 0,
    .mode = 0,
    .duty_cycle_pos = 0,
    .cs_ena_pretrans = 0,
    .cs_ena_posttrans = 0,
    .clock_speed_hz = 1000000,
    .input_delay_ns = 0,
    .spics_io_num = PIN_NUM_CS,
    .flags = 0,
    .queue_size = 1,
    .pre_cb = NULL,
    .post_cb = NULL,
    .flags = 0,
    };
    
    ret = spi_bus_add_device(SPIHOST, &dev_config, spi_device_handle);
    ESP_ERROR_CHECK(ret);

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


void nrf24_WriteRegister(uint8_t reg, uint8_t data, spi_device_handle_t *spi_device_handle){//This method is used to write to s specific register, the NRF24L01 has registers to configure different settings for the NRF24L01.

    uint8_t buffer[2];
    buffer[0] = reg|1<<5; //In datasheet w_register says fifth bit needs to be a 1.
    buffer[1] = data;
    
    // Write to NRF24L01 register
    //uint8_t registerAddress = 0x10;
    //uint8_t data = 0xAB;

    spi_transaction_t trans = {
        .length = 16,  // Length in bits
        .tx_buffer = &buffer,
        .rx_buffer = NULL,
        .user = NULL,
    };

    CS_Select();

    spi_device_transmit(*spi_device_handle, &trans);

    CS_Unselect();
}

void nrf24_WriteRegisterMulti(uint8_t reg, uint8_t *data, spi_device_handle_t *spi_device_handle, int numberofBytes){//This method is used to write to s specific register, the NRF24L01 has registers to configure different settings for the NRF24L01.

    uint8_t buffer = reg|1<<5;

    
    
    // Write to NRF24L01 register
    //uint8_t registerAddress = 0x10;
    //uint8_t data = 0xAB;

    spi_transaction_t trans[2] = {
        {
        .length = 16,  // Length in bits
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
    };

    CS_Select();

    spi_device_transmit(*spi_device_handle, &trans);

    CS_Unselect();
}


uint8_t ReadReg(uint8_t reg, spi_device_handle_t *spi_device_handle){

    uint8_t data;

    CS_Select();

    spi_transaction_t trans = {
        .length = 16,  // Length in bits
        .tx_buffer = &reg,
        .rx_buffer = &data,
        .user = NULL,
    };

    esp_err_t ret = spi_device_transmit(*spi_device_handle, &trans);
    ESP_ERROR_CHECK(ret);


    CS_Unselect();

    return data;
}

void ReadRegMulti(uint8_t reg, uint8_t *data, int numberofBytes, spi_device_handle_t *spi_device_handle){

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
    };

    /* Alternative transaction
    spi_transaction_t trans = {
        .length = 8 + 8 *numberofBytes,  // Length in bits of register then length of data bits
        .tx_buffer = &reg,
        .rx_buffer = data,
        .user = NULL,
    };
    */


    CS_Select();

    esp_err_t ret = spi_device_transmit(*spi_device_handle, &trans);
    ESP_ERROR_CHECK(ret);


    CS_Unselect();

    return data;
}

void nrfsendCmd (uint8_t cmd, spi_device_handle_t *spi_device_handle)
{   
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
	CS_UnSelect();
}


void NRF24_Init(spi_device_handle_t *spi_device_handle){

    CE_Disable();
    CS_UnSelect(); //This call is not needed but for extra safety i guess.

    nrf24_WriteRegister(CONFIG, 0, spi_device_handle); //Will be configured later.

    nrf24_WriteRegister(EN_AA, 0, spi_device_handle); //No auto-Acknowledgment

    nrf24_WriteRegister(EN_RXADDR, 0, spi_device_handle); //Will be configured later.

    nrf24_WriteRegister(SETUP_AW, 0x03, spi_device_handle);

    nrf24_WriteRegister(SETUP_RETR, 0, spi_device_handle); //Retransimssion disabled

    nrf24_WriteRegister(RF_CH, 0, spi_device_handle);

    nrf24_WriteRegister(RF_SETUP, 0x0E, spi_device_handle); //Power = 0dbm,  data rate = 2mbps

    

}



