#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "uart.h"
#include "analogRead.h"
#include "millis.h"
#include "servo.h"


#define BIT_SET(a, b) ((a) |= (1ULL << (b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1ULL<<(b)))
#define BIT_FLIP(a,b) ((a) ^= (1ULL<<(b)))
#define BIT_CHECK(a,b) (!!((a) & (1ULL<<(b))))

#define ON_BOARD_LED 5

#define SERVO_1 4

#define VERT_PIN 0
#define HORZ_PIN 1
#define SEL_PIN  2

#define BUTTON_IS_CLICKED(PINB,BUTTON_PIN) !BIT_CHECK(PINB,BUTTON_PIN)

void ConvertToPercentage(double*);

void WaitForStartButtonClick();

int main()
{   

    volatile millis_t milliSecsSinceLastCheck = 0;

    double horz;
    double vert;

    initJoystick();
    init_servo();
    init_serial();
    millis_init();
    sei();     

    BIT_SET(DDRB, ON_BOARD_LED); //Sätt led_pin_red till output mode
    BIT_SET(DDRD, SERVO_1); //Servo

    while (1) {

		horz = analogRead(HORZ_PIN);
  		vert = analogRead(VERT_PIN);

        ConvertToPercentage(&horz);

        servo1_set_percentage(horz);

        if (BUTTON_IS_CLICKED(PIND,SEL_PIN)){
            BIT_SET(PORTB, ON_BOARD_LED);
        }
        else{
            BIT_CLEAR(PORTB, ON_BOARD_LED);
        }
  	}
    return 0;
}

void initJoystick(){
    BIT_CLEAR(DDRC,VERT_PIN); //data direction register (DDR) is used to set the direction of a pin
	BIT_CLEAR(DDRC,HORZ_PIN); //the code clears (sets to 0) the respective bits in the DDRC register, indicating that the pins connected to the vertical and horizontal buttons should be configured as input pins.

	//Sätt till INPUT_PULLUP
    BIT_CLEAR(DDRD,SEL_PIN); // INPUT MODE
    BIT_SET(PORTD,SEL_PIN); 

}

void ConvertToPercentage(double *reading){
    *reading -= 512;
    *reading /= 512;
    *reading *= 100;
}


void WaitForStartButtonClick(){
    while (1){ //Wait for start condition: Joystick button press.
        if (BUTTON_IS_CLICKED(PIND,SEL_PIN))
            break;
    }
    _delay_ms(100);
}

