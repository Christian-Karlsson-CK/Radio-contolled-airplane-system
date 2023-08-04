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
    NRF24_RXMode(RxAddress, 55);

    //nrf24_WriteRegister(STATUS, 0);
    //nrf24_WriteRegister(FIFO_STATUS, 0);
    //nrfsendCmd(FLUSH_RX);
    //if (STATUS == 78)
    //{
    //    nrfsendCmd(FLUSH_RX);
    //    nrf24_WriteRegister(STATUS, 0);
    //}
    

    //BIT_SET(DDRB, ON_BOARD_LED); //Sätt led_pin_red till output mode
    //BIT_SET(DDRD, SERVO_1); //Servo

    while (1) {

        CE_Enable();

        uint8_t reg = NRF24_ReadReg(STATUS);
        lcd_set_cursor(0,0);
        lcd_printf("S%u", reg);

        reg = NRF24_ReadReg(FIFO_STATUS);
        lcd_set_cursor(4,0);
        lcd_printf("F%u", reg);
        //_delay_ms(1000);

        reg = NRF24_ReadReg(RPD);
        lcd_set_cursor(8,0);
        lcd_printf("R%u", reg);

        //64 16 1 first loop 64=New data received and bit 3:1 is all 0 meaning datapipe 0, 16=data in RX fifo. 1 = RPD
        //78 17 1 from second loop i believe 78 = 64bit still set(RX_DR),bit 3:1 is 111 meaning RX fifo empty. 1 = RPD

        if (NRF24_RXisDataReady(0) == 1)
        {   
            lcd_set_cursor(0,1);
            lcd_printf("M:");

            NRF24_Receive(RxData);
            for (size_t i = 0; i < 30; i++)
            {
                lcd_set_cursor(i+2,1);
                lcd_printf("%c", RxData[i]);
            
                //lcd_set_cursor(0,1);
                //lcd_printf("%c", RxData[i+1]);
            
            }
            //_delay_ms(10000);
            reg = NRF24_ReadReg(STATUS);
            lcd_set_cursor(0,0);
            lcd_printf("S%u", reg);

            reg = NRF24_ReadReg(FIFO_STATUS);
            lcd_set_cursor(4,0);
            lcd_printf("F%u", reg);

            reg = NRF24_ReadReg(RPD);
            lcd_set_cursor(8,0);
            lcd_printf("R%u", reg);

            _delay_ms(2000);
            
        }
        else{
            lcd_set_cursor(2,1);
            lcd_printf("NO MESSAGE");
            //_delay_ms(100);
            CE_Disable();
            //lcd_set_cursor(2,1);
            //lcd_printf("Disabled");
            _delay_ms(2000);

            CE_Enable();
            //lcd_set_cursor(2,1);
            //lcd_printf("Enabled");
            _delay_ms(20);
            
        }

        
         

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

