
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

#include "driver/spi_common.h"
#include "driver/spi_master.h"

#include "NRF24L01.h"
#include "lcd.h"
#include "GPIO_PINS.h"

#define SWITCH_TO_TX_COMMAND      255
#define RECEIVED_ACKNOWLEDGMENT   123 

void init_controls();

void TransmitData(uint8_t *TxData, spi_device_handle_t *spi_device_handle);
void ReadAllAnalog(uint8_t *TxData);

uint8_t ReceiveData(spi_device_handle_t *spi_device_handle);

static const char *TAG = "Debug:";

typedef struct AnalogReadings
{   
    uint16_t throttle;
    uint16_t rudder;
    uint16_t ailerons;
    uint16_t elevator;
    uint8_t  switchRates;
    uint8_t  switchArm;
    uint8_t  potentiometerScreen;
    uint8_t  potentiometerTrim;
    uint8_t  switchBuzzer_RTH;
    uint8_t  switchTrim;
}AnalogReadings;


void app_main(void)
{
    //Joystick
    init_controls();
    lcd_init();

    spi_device_handle_t spi_device_handle; //SPI handle for the SPI communication to NRF24L01+
    SPI_init(&spi_device_handle);

    uint8_t TxData[32];

    NRF24_Init(&spi_device_handle);

    NRF24_TXMode(&spi_device_handle);

    //NRF24_RXMode(&spi_device_handle);
    
    uint8_t TX_RX_Switch_counter = 0; //This counter is used to switch between RX and TX mode. TX mode sneds all analog stick and potentiometer values.
                                      //RX mode is used to receive status data from the RX on the airplane, for instance Battery voltage. 

    while (true)
    {   
        
        TX_RX_Switch_counter++;
        if (TX_RX_Switch_counter < 125)
        {   
            //NRF24_TXMode(&spi_device_handle);
            ReadAllAnalog(TxData);
            TxData[6] = 0;
            TransmitData(TxData, &spi_device_handle);
            vTaskDelay(pdMS_TO_TICKS(10));
            ESP_LOGI(TAG, "TX");
        }
        else if (TX_RX_Switch_counter >= 125)
        {   
            TxData[6] = SWITCH_TO_TX_COMMAND;
            TransmitData(TxData, &spi_device_handle);
            

            NRF24_RXMode(&spi_device_handle);
            vTaskDelay(pdMS_TO_TICKS(10));
            uint8_t tries = 0;

            while (!ReceiveData(&spi_device_handle) && tries < 5)
            {   
                ESP_LOGI(TAG, "RX");
                ESP_LOGI(TAG, "No message!");
                vTaskDelay(pdMS_TO_TICKS(2));
                tries++;
            }
            if (tries <= 5)
            {
                ESP_LOGI(TAG, "RX");
                ESP_LOGI(TAG, "Message!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            }
            
            tries = 0;
            TX_RX_Switch_counter = 0;
            //vTaskDelay(pdMS_TO_TICKS(10));
            NRF24_TXMode(&spi_device_handle);
        }
        
        
        /*
        NRF24_TXMode(&spi_device_handle);
        ReadAllAnalog(TxData);
        TxData[6] = SWITCH_TO_TX_COMMAND;
        TransmitData(TxData, &spi_device_handle);
        

        NRF24_RXMode(&spi_device_handle);
        while (!ReceiveData(&spi_device_handle))
        {
            ESP_LOGI(TAG, "No message!");
            //vTaskDelay(pdMS_TO_TICKS(1000));    
        }
        ESP_LOGI(TAG, "We got message!");
        //vTaskDelay(pdMS_TO_TICKS(10000));
        */
        

        
    }
}

void init_controls(){
    //Joystick
    gpio_set_direction(PIN_RUDDER, GPIO_MODE_INPUT); //X axis
    gpio_set_direction(PIN_THROTTLE, GPIO_MODE_INPUT); //Y axis

    adc1_config_width(ADC_WIDTH_BIT_10); // Analog digital converter, 10bit = 0-1023
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11); //I think this will improve edges on joystick
    adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11);

    //Switch
    gpio_set_direction(PIN_NUM_SW_UP, GPIO_MODE_INPUT);
    gpio_set_direction(PIN_NUM_SW_DOWN, GPIO_MODE_INPUT);
}


void TransmitData(uint8_t *TxData, spi_device_handle_t *spi_device_handle){

    if(NRF24_Transmit(TxData, spi_device_handle)){
        ESP_LOGI(TAG, "SEEMS TO TRANSMIT");
    }
    nrf24_WriteRegister(STATUS, 32, spi_device_handle);
    //vTaskDelay(pdMS_TO_TICKS(10));
}

uint8_t ReceiveData(spi_device_handle_t *spi_device_handle){
    uint8_t RxData[32];

    if (NRF24_RXisDataReady(0, spi_device_handle) == 1)
    {   
        ESP_LOGI(TAG, "We have data available!");
        NRF24_Receive(RxData, spi_device_handle);

        //vTaskDelay(pdMS_TO_TICKS(3000));
        uint8_t whole = RxData[3];
        uint8_t decimal = RxData[4];

        //uint8_t whole = 12;
        //uint8_t decimal = 4;
        
        ESP_LOGI(TAG, "whole: %u, decimal: %u", whole, decimal);

        
        char buffer[6]; // Enough space for 5 digits plus null-terminator
        snprintf(buffer, sizeof(buffer), "%u", whole); // Format the value as a string

        char buffer1[6]; // Enough space for 5 digits plus null-terminator
        snprintf(buffer1, sizeof(buffer1), "%u", decimal); // Format the value as a string
    
        if (whole < 10)
        {   
            ESP_LOGI(TAG, "whole < 10");
            lcd_send_data('0');
            lcd_send_data(buffer[0]);
        }
        else{
            ESP_LOGI(TAG, "whole >= 10");
            lcd_send_data(buffer[0]);
            lcd_send_data(buffer[1]);
        }

        lcd_send_data('.');
        
        if (decimal < 10)
        {
            lcd_send_data('0');
            lcd_send_data(buffer1[0]);
        }
        else{
            lcd_send_data(buffer1[0]);
            lcd_send_data(buffer1[1]);
        }
        lcd_send_data('V');
    
        //ESP_LOGI(TAG, "buffer:%c", buffer[0]);
        //ESP_LOGI(TAG, "buffer:%c", buffer[1]);
        //lcd_send_data(buffer[0]);
        //lcd_send_data(buffer[1]);
        //lcd_send_data('V');
        //ESP_LOGI(TAG, "%.02u.%.02uV", whole, decimal);

        //vTaskDelay(pdMS_TO_TICKS(3000));
        return 1;
    }
    else{
        ESP_LOGI(TAG, "NO MESSAGE :-(");
        //vTaskDelay(pdMS_TO_TICKS(1));
        return 0;
    }
    
    
}

void ReadAllAnalog(uint8_t *TxData){
    uint16_t xReading = adc1_get_raw(ADC1_CHANNEL_0);
        uint16_t yReading = adc1_get_raw(ADC1_CHANNEL_3);
        //ESP_LOGI(TAG, "Y: %u, X:%u", yReading, xReading);

        uint8_t switchUp = gpio_get_level(PIN_NUM_SW_UP);
        uint8_t switchDown = gpio_get_level(PIN_NUM_SW_DOWN);
        //ESP_LOGI(TAG, "UP: %u, DOWN: %u", switchUp, switchDown);
    
        TxData[2] = (uint8_t)(xReading & 0xFF);
        TxData[3] = (uint8_t)(xReading >> 8);
        TxData[4] = (uint8_t)(yReading & 0xFF);
        TxData[5] = (uint8_t)(yReading >> 8);

        //uint16_t receivedValueX = (uint16_t)((TxData[3] << 8) | TxData[2]);
        //uint16_t receivedValueY = (uint16_t)((TxData[5] << 8) | TxData[4]);
        //ESP_LOGI(TAG, "Y: %u, X:%u", receivedValueY, receivedValueX);
}
