#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/adc.h"
#include "driver/uart.h"

#include "driver/spi_common.h"
#include "driver/spi_master.h"

#include "NRF24L01.h"
#include "lcd.h"
#include "GPIO_PINS.h"

#define SWITCH_TO_TX_COMMAND        255
#define NO_COMMAND                  0

#define COMMAND_BYTE                6

#define TX_TO_RX_THRESHOLD          50
//#define RECEIVED_ACKNOWLEDGMENT   123

typedef struct AnalogReadings
{   
    uint16_t throttle;
    uint16_t rudder;
    uint16_t ailerons;
    uint16_t elevator;
    uint8_t  switch1;
    uint8_t  switch2;
    uint8_t  potentiometer1;
    uint8_t  potentiometer2;
}AnalogReadings;

void init_controls();

void TransmitData(uint8_t *TxData, spi_device_handle_t *spi_device_handle);
void ReadAllAnalog(uint8_t *TxData);

uint8_t ReceiveData(spi_device_handle_t *spi_device_handle, uint8_t *RxData);