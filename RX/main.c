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

#define SERVO_1 4

void ConvertToPercentage(double*);

int main()
{   

    volatile millis_t milliSecsSinceLastCheck = 0;

    double horz;
    double vert;

    uint8_t RxAddress[] = {0xEE, 0xDD, 0xCC, 0xBB, 0xAA}; //40bits
    uint8_t RxData[32];

    //init_servo();
    //init_serial();
    millis_init();
    sei();

    lcd_init();
    lcd_enable_blinking();
    lcd_enable_cursor();
    

    SPI_init();
    _delay_ms(100);				// Power on reset 100ms
    
    NRF24_Init();
    NRF24_RXMode(RxAddress, 10);

    //BIT_SET(DDRB, ON_BOARD_LED); //Sätt led_pin_red till output mode
    //BIT_SET(DDRD, SERVO_1); //Servo

    while (1) {
        /*
        if (NRF24_RXisDataReady(1) == 1)
        {
           NRF24_Receive(RxAddress, );
        }
        */
         

        //ConvertToPercentage(&horz);

        //servo1_set_percentage(horz);
  	}
    return 0;
}

void ConvertToPercentage(double *reading){
    *reading -= 512;
    *reading /= 512;
    *reading *= 100;
}


/*void WaitForStartButtonClick(){
    while (1){ //Wait for start condition: Joystick button press.
        if (BUTTON_IS_CLICKED(PIND,SEL_PIN))
            break;
    }
    _delay_ms(100);
}*/

