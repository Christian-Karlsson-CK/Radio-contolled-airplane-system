
/*
Hjälp video:
https://www.youtube.com/watch?v=oHHOCdmLiII&list=PLHX2-9M57aE5LVZnGwbo2cjnqvLqtu2OS&index=3

idf.py create-project -p . [PROJECT_NAME]
	Använd detta kommando för att skapa ett nytt projekt

idf.py build
	build projektet

idf.py monitor -p COM3
	få in händelser direkt från MCUn

idf.py flash -p COM3 monitor
    builda & flasha till MCU, monitor är optional om man vill övervaka.


*/

#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/adc.h"

//#include "hal/spi_types.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"

#include "NRF24L01.h"

//NRF24L01 PINS
#define SPIHOST HSPI_HOST //Using VSPI on ESP32 SPIHOST = The SPI controller peripheral inside ESP32. VSPI_HOST = SPI3_HOST=2
#define PIN_NUM_MISO 19   // SPI PINS
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   17    //To the CSN pin on the LRF24L01+. used to select the specific SPI device with which the ESP32 wants to communicate.
#define PIN_NUM_CE   16   //used to control the NRF24L01 module's operation mode.

//Joystick 1 PINS
#define PIN_NUM_X      36
#define PIN_NUM_Y      39
#define PIN_NUM_BUTTON 34

//Switch
#define PIN_NUM_SW_UP    32
#define PIN_NUM_SW_DOWN  33


static const char *TAG = "Debug:";


void app_main(void)
{
    //Joystick
    gpio_set_direction(PIN_NUM_X, GPIO_MODE_INPUT); //X axis
    gpio_set_direction(PIN_NUM_Y, GPIO_MODE_INPUT); //Y axis
    gpio_set_direction(PIN_NUM_BUTTON, GPIO_MODE_INPUT); //Joystick button
    gpio_set_pull_mode(PIN_NUM_BUTTON, GPIO_PULLUP_ONLY);

    adc1_config_width(ADC_WIDTH_BIT_10); // Analog digital converter, 10bit = 0-1023
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11); //I think this will improve edges on joystick
    adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11);

    //Switch
    gpio_set_direction(PIN_NUM_SW_UP, GPIO_MODE_INPUT);
    gpio_set_direction(PIN_NUM_SW_DOWN, GPIO_MODE_INPUT);


    spi_device_handle_t spi_device_handle; //SPI handle for the SPI communication to NRF24L01+

    SPI_init(&spi_device_handle);

    uint8_t TxAddress[] = {0xEE, 0xDD, 0xCC, 0xBB, 0xAA}; //40bits
    uint8_t TxData[32];

    uint8_t RxAddress[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE};
    uint8_t RxData[32];
    
    NRF24_Init(TxAddress, RxAddress, 55, &spi_device_handle);

    //NRF24_TXMode(&spi_device_handle);
    
    NRF24_RXMode(&spi_device_handle);
    
    while (true)
    {   
        uint16_t whole = (uint16_t)((RxData[4] << 8) | RxData[3]);
        uint16_t decimal = (uint16_t)((RxData[6] << 8) | RxData[5]);

        if (NRF24_RXisDataReady(0, &spi_device_handle) == 1)
        {   
            ESP_LOGI(TAG, "We have data available!");
            NRF24_Receive(RxData, &spi_device_handle);

            ESP_LOGI(TAG, "%.02u.%.02uV", whole, decimal);

            vTaskDelay(pdMS_TO_TICKS(1000));
            
        }
        else{
            ESP_LOGI(TAG, "NO MESSAGE :-(");
            vTaskDelay(pdMS_TO_TICKS(1000));
            
        }
        
        uint16_t xReading = adc1_get_raw(ADC1_CHANNEL_0);
        uint16_t yReading = adc1_get_raw(ADC1_CHANNEL_3);
        //ESP_LOGI(TAG, "Y: %u, X:%u", yReading, xReading);

        uint8_t switchUp = gpio_get_level(PIN_NUM_SW_UP);
        uint8_t switchDown = gpio_get_level(PIN_NUM_SW_DOWN);
        //ESP_LOGI(TAG, "UP: %u, DOWN: %u", switchUp, switchDown);

        //buffer[1] = (uint8_t)(value & 0xFF); // Extract the lower 8 bits of the uint16_t
        //buffer[2] = (uint8_t)((value >> 8) & 0xFF);

        TxData[2] = (uint8_t)(xReading & 0xFF);
        TxData[3] = (uint8_t)(xReading >> 8);
        TxData[4] = (uint8_t)(yReading & 0xFF);
        TxData[5] = (uint8_t)(yReading >> 8);


        uint16_t receivedValueX = (uint16_t)((TxData[3] << 8) | TxData[2]);
        uint16_t receivedValueY = (uint16_t)((TxData[5] << 8) | TxData[4]);
        //ESP_LOGI(TAG, "Set together: Y: %u, X:%u", receivedValueY, receivedValueX);

        /*if(NRF24_Transmit(TxData, &spi_device_handle)){
            //ESP_LOGI(TAG, "SEEMS TO TRANSMIT");
        }
        nrf24_WriteRegister(STATUS, 32, &spi_device_handle);*/




        //uint8_t reg = NRF24_ReadReg(STATUS, &spi_device_handle);
        //ESP_LOGI(TAG, "STATUS RESET: %u", reg);
        //reg = NRF24_ReadReg(FIFO_STATUS, spi_device_handle);
        //ESP_LOGI(TAG, "FIFO_STATUS: %u", reg);

        vTaskDelay(pdMS_TO_TICKS(10));
        //vTaskDelay(100);
    }
    


}
