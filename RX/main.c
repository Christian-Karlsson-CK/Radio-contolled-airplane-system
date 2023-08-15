#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "uart.h"
#include "analogRead.h"
#include "millis.h"
#include "servo.h"
#include "NRF24L01.h"
#include "UnoR3Pins.h"

#include "lcd.h"


#define BIT_SET(a, b) ((a) |= (1ULL << (b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1ULL<<(b)))
#define BIT_FLIP(a,b) ((a) ^= (1ULL<<(b)))
#define BIT_CHECK(a,b) (!!((a) & (1ULL<<(b))))

#define ON_BOARD_LED 5

#define SERVO_1 3

#define BATTERY_MONITOR_PIN 3

void ConvertToPercentage(double*);
uint16_t read_analog_pin(uint8_t pin);

int main()
{   

    volatile millis_t milliSecsSinceLastCheck = 0;

    double horz;
    double vert;

    uint8_t RxAddress[] = {0xEE, 0xDD, 0xCC, 0xBB, 0xAA}; //40bits
    uint8_t RxData[32];

    uint8_t TxAddress[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE};
    uint8_t TxData[32];

    init_servo();
    //init_serial();
    millis_init();
    sei();

    lcd_init();
    lcd_enable_blinking();
    lcd_enable_cursor();
    

    SPI_init();
    
    NRF24_Init(TxAddress, RxAddress, 55);
    //NRF24_RXMode();
    NRF24_TXMode();
                                            //PWR UP/PWRDOWN DELAY 2MS
                                            //TX MODE = CE HIGH FOR 10 MICROSECONDS
                                            //CE positive edge to csn low 4microseconds
    BIT_CLEAR(DDRC, BATTERY_MONITOR_PIN);
    
    while (1) {

        //Battery monitor:
        int read = analogRead(BATTERY_MONITOR_PIN);
        float voltage = (analogRead(BATTERY_MONITOR_PIN) / 1023.0) * 4.7 * 3;
        float voltage2 = (200 / 1023.0) * 4.7 * 3;
        
        uint16_t whole = voltage;
        uint16_t decimal = (voltage - whole) * 100;

        

        TxData[2] = (uint8_t)(whole & 0xFF);
        TxData[3] = (uint8_t)(whole >> 8);
        TxData[4] = (uint8_t)(decimal & 0xFF);
        TxData[5] = (uint8_t)(decimal >> 8);

        uint16_t whole1 = (uint16_t)((TxData[3] << 8) | TxData[2]);
        uint16_t deci = (uint16_t)((TxData[5] << 8) | TxData[4]);

        //uint16_t whole1 = 11;
        //uint16_t deci = 22;

        lcd_set_cursor(0,0);
        lcd_printf("%.02u.%.02u", whole1, deci);
        

        //lcd_set_cursor(7,0);
        //lcd_printf("Volts");
        
        //TxMode
        NRF24_Transmit(TxData, 32);

        if(NRF24_Transmit(TxData, 32)){
            lcd_set_cursor(0,1);
            lcd_printf("Msg sent");
        }
        else
        {
            lcd_set_cursor(0,1);
            lcd_printf("Msg not sent");
        }
        _delay_ms(1000);
        lcd_clear();
        

        //RxMode
        /*if (NRF24_RXisDataReady(0) == 1)
        {   
            NRF24_Receive(RxData);
            //_delay_ms(10);
            
            uint16_t receivedValueX = (uint16_t)((RxData[3] << 8) | RxData[2]);
            uint16_t receivedValueY = (uint16_t)((RxData[5] << 8) | RxData[4]);

            double percentX = receivedValueX;
            double percentY = receivedValueY;
            ConvertToPercentage(&percentX);
            ConvertToPercentage(&percentY);

            //lcd_clear();
            lcd_set_cursor(0,1);
            lcd_printf("%u", receivedValueX);
            lcd_set_cursor(7,1);
            lcd_printf("%u", receivedValueY);

            servo1_set_percentage(percentX);

            _delay_ms(50);
            
        }
        else{
            lcd_set_cursor(2,1);
            lcd_printf("NO MESSAGE");
            _delay_ms(1000);
            
        }*/
  	}
    return 0;
}

void ConvertToPercentage(double *reading){ //Works with values from 0-1023.
    *reading -= 512;
    *reading /= 512;
    *reading *= 100;
}

uint16_t read_analog_pin(uint8_t pin) {
    ADMUX = (ADMUX & 0xF0) | (pin & 0x0F); // Select the ADC channel, ADC channel have same number as the analog pin.

    ADCSRA |= (1 << ADSC); // Start conversion
    while (ADCSRA & (1 << ADSC)); // Wait for the conversion to complete

    return ADC; // Return the ADC result (a 10-bit value)
}



