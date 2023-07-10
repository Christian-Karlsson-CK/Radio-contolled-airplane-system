
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
    flasha till MCU, monitor är optional om man vill övervaka.


*/

#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/adc.h"

#include "hal/spi_types.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"

//NRF24L01 PINS
#define SPIHOST VSPI_HOST //Using VSPI on ESP32 SPIHOST = The SPI controller peripheral inside ESP32. VSPI_HOST = SPI3_HOST=2
#define PIN_NUM_MISO 19   // SPI PINS
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5    //To the CSN pin on the LRF24L01+. used to select the specific SPI device with which the ESP32 wants to communicate.
#define PIN_NUM_CE   17   //used to control the NRF24L01 module's operation mode.

//Joystick 1 PINS
#define PIN_NUM_X      36
#define PIN_NUM_Y      39
#define PIN_NUM_BUTTON 34



static const char *TAG = "example";


void app_main(void)
{

    gpio_set_direction(PIN_NUM_X, GPIO_MODE_INPUT); //X axis
    gpio_set_direction(PIN_NUM_Y, GPIO_MODE_INPUT); //Y axis

    gpio_set_direction(PIN_NUM_BUTTON, GPIO_MODE_INPUT); //Joystick button
    gpio_set_pull_mode(PIN_NUM_BUTTON, GPIO_PULLUP_ONLY);

    adc1_config_width(ADC_WIDTH_BIT_10); // Analog digital converter, 10bit = 0-1023
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_0); //I think this will improve edges on joystick
    adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11);


    spi_device_handle_t spi_device_handle; //SPI handle for the SPI communication to NRF24L01+

    SPI_init(&spi_device_handle);



    while (true)
    {   
        
        //int xReading = adc1_get_raw(ADC1_CHANNEL_0);
        //int yReading = adc1_get_raw(ADC1_CHANNEL_3);



        //ESP_LOGI(TAG, "Y: %d, X:%d", yReading, xReading);


        ESP_LOGI(TAG, "Y: %d, X:%d");


        vTaskDelay(100);
    }
    


}
