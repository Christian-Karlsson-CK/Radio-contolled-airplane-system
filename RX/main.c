#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "uart.h"
#include "analogRead.h"
//#include "millis.h"
#include "servo.h"
#include "NRF24L01.h"
#include "UnoR3Pins.h"

#include "lcd.h"

#define SWITCH_TO_TX_COMMAND 255


#define BIT_SET(a, b) ((a) |= (1ULL << (b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1ULL<<(b)))
#define BIT_FLIP(a,b) ((a) ^= (1ULL<<(b)))
#define BIT_CHECK(a,b) (!!((a) & (1ULL<<(b))))

//#define ON_BOARD_LED 5

//#define SERVO_1 3

void ConvertToPercentage(double*);
uint16_t read_analog_pin(uint8_t pin);
uint8_t ReceiveData(uint8_t *RxData);
void TransmitData(uint8_t *TxData);

//ISR(TIMER2_OVF_vect) {
//    PORTD ^= (1 << PIN_NUM_THROTTLE);    // Toggle ESC pin at each overflow
//}

int main()
{   

    //volatile millis_t milliSecsSinceLastCheck = 0;

    double horz;
    double vert;

    init_servo();
    //init_serial();
    //millis_init();
    sei();

    lcd_init();
    //lcd_enable_blinking();
    //lcd_enable_cursor();
    //lcd_set_cursor(0,0);
    //lcd_printf("HEJ");

    SPI_init();
    
    NRF24_Init();
    NRF24_RXMode();
    //NRF24_TXMode();
                                            //PWR UP/PWRDOWN DELAY 2MS
                                            //TX MODE = CE HIGH FOR 10 MICROSECONDS
                                            //CE positive edge to csn low 4microseconds
    BIT_CLEAR(DDRC, BATTERY_MONITOR_PIN);
    
    uint8_t TxData[32];
    uint8_t RxData[32];

    double percentX = 512;
    double percentY = 512;



    //BIT_SET(DDRD, PIN_NUM_THROTTLE);

    //TCCR2A = (1 << COM2A1) | (1 << WGM21) | (1 << WGM20); // Clear OC2A on Compare Match
    //TCCR2B = (1 << CS21); // Set prescaler to 8 for an 8 kHz PWM frequency
    
    // Initialize ESC to lowest throttle value
    //OCR2A = 0; // Example value, adjust to match your ESC's minimum pulse width
    
    // Enable Timer2 overflow interrupt
    //TIMSK2 |= (1 << TOIE2);

    //int speed = 1;

    while (1) {
        
        //OCR2A = 0;
        
        //OCR2A = 255;
        //_delay_ms(10000);




        
        //Battery monitor:
        float voltage = (analogRead(BATTERY_MONITOR_PIN) / 1023.0) * 4.7 * 3;
        
        TxData[2] = voltage; //Heltal
        TxData[3] = (voltage - TxData[2]) * 100; //Decimal

        ReceiveData(RxData);
        //_delay_ms(41);// 41 seems to be good delay
        if (RxData[6] == SWITCH_TO_TX_COMMAND)
        {
            NRF24_TXMode();
            TransmitData(TxData);
            NRF24_RXMode();
        }
        
        //NRF24_TXMode();
        //TransmitData(TxData);
        //NRF24_RXMode();
        _delay_ms(10);// 41 seems to be good delay
        
    }
    //return 0;
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

uint8_t ReceiveData(uint8_t *RxData){
    //uint8_t RxData[32];
    if (NRF24_RXisDataReady(0) == 1)
    {   
        NRF24_Receive(RxData);
        //_delay_ms(10);
        
        uint16_t receivedValueX = (uint16_t)((RxData[3] << 8) | RxData[2]);
        uint16_t receivedValueY = (uint16_t)((RxData[5] << 8) | RxData[4]);
        uint16_t receivedValueX1 = (uint16_t)((RxData[7] << 8) | RxData[6]);
        uint16_t receivedValueY1 = (uint16_t)((RxData[9] << 8) | RxData[8]);
        uint16_t pot1 = (uint16_t)((RxData[11] << 8) | RxData[10]);
        uint16_t pot2 = (uint16_t)((RxData[13] << 8) | RxData[12]);
        uint8_t switch1Up  = RxData[14];
        uint8_t switch1Down  = RxData[15];
        uint8_t switch2Up  = RxData[16];
        uint8_t switch2Down  = RxData[17];

        //uint8_t command = RxData[6];

        if (receivedValueX >= 50 && receivedValueX <= 1000)
        {
            double percentX = receivedValueX;
            ConvertToPercentage(&percentX);
            servo1_set_percentage(percentX);
        }
        
        if (receivedValueY >= 50 && receivedValueY <= 1000)
        {
            double percentY = receivedValueY;
            ConvertToPercentage(&percentY);
            servo2_set_percentage(percentY);
        }

        if (receivedValueX1 >= 0 && receivedValueX1 <= 1023)
        {
            double percentX = receivedValueX1;
            ConvertToPercentage(&percentX);
            //servo3_set_percentage(percentX);
        }
        
        if (receivedValueY1 >= 0 && receivedValueY1 <= 1023)
        {
            double percentY = receivedValueY1;
            ConvertToPercentage(&percentY);
            //servo4_set_percentage(percentY);
        }

        //double percentX = receivedValueX;
        //double percentY = receivedValueY;
        /*
        if (percentX > 1023)
        {
            percentX = 512;
        }
        if (percentY > 1023)
        {
            percentY = 512;
        }*/
        
        
        
        
        /*
        lcd_clear();
        lcd_set_cursor(0,0);
        lcd_printf("%u", receivedValueX);
        lcd_set_cursor(4,0);
        lcd_printf("%u", receivedValueY);
        lcd_set_cursor(8,0);
        lcd_printf("%u", receivedValueX1);
        lcd_set_cursor(12,0);
        lcd_printf("%u", receivedValueY1);

        lcd_set_cursor(0,1);
        lcd_printf("%u", pot1);
        lcd_set_cursor(4,1);
        lcd_printf("%u", pot2);

        lcd_set_cursor(8,1);
        lcd_printf("%u", switch1Up);
        lcd_set_cursor(10,1);
        lcd_printf("%u", switch1Down);

        lcd_set_cursor(12,1);
        lcd_printf("%u", switch2Up);
        lcd_set_cursor(14,1);
        lcd_printf("%u", switch2Down);
        */
        

        //_delay_ms(60);
        return 1;
    }
    
    else{
        //lcd_set_cursor(2,1);
        //lcd_printf("NO MESSAGE");
        //_delay_ms(50);
        return 0;
    }
}

void TransmitData(uint8_t *TxData){
    if(NRF24_Transmit(TxData, 32)){
            //lcd_set_cursor(0,1);
            //lcd_printf("Msg sent");
        }
        else
        {
            //lcd_set_cursor(0,1);
            //lcd_printf("Msg not sent");
        }
        //_delay_ms(10);
        //lcd_clear();
        

}

