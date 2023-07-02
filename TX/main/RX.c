
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

start idf.py menuconfig

*/

#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/adc.h"


static const char *TAG = "example";

void app_main(void)
{

    gpio_set_direction(GPIO_NUM_16, GPIO_MODE_INPUT);
    gpio_set_direction(GPIO_NUM_17, GPIO_MODE_INPUT);
    gpio_set_direction(GPIO_NUM_5, GPIO_MODE_INPUT);

    gpio_set_pull_mode(GPIO_NUM_5, GPIO_PULLUP_ONLY);

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11);

    //bool on = true;

    while (true)
    {   
        
        int xReading = adc1_get_raw(ADC1_CHANNEL_0);
        int yReading = adc1_get_raw(ADC1_CHANNEL_3);
        

        ESP_LOGI(TAG, "Y: %d, X:%d", yReading, xReading);
        
        vTaskDelay(10);
    }
    


}
