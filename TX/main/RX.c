
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
    flasha till MCU monitor är optional om man vill övervaka.

start idf.py menuconfig

*/

#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"



void app_main(void)
{

    gpio_set_direction(GPIO_NUM_5, GPIO_MODE_OUTPUT);

    bool on = true;

    while (true)
    {
        if (on)
        {
            gpio_set_level(GPIO_NUM_5, 0);
            on = false;
        }
        else{
            gpio_set_level(GPIO_NUM_5, 1);
            on = true;
        }
        
        vTaskDelay(10);
    }
    


}
