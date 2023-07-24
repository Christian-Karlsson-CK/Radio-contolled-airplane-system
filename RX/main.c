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

//NRF24L01 PINS
//#define SPIHOST VSPI_HOST //Using VSPI on ESP32 SPIHOST = The SPI controller peripheral inside ESP32. VSPI_HOST = SPI3_HOST=2
#define PIN_NUM_MISO 12   // SPI PINS
#define PIN_NUM_MOSI 11
#define PIN_NUM_CLK  13
#define PIN_NUM_CS   10    //To the CSN pin on the LRF24L01+. used to select the specific SPI device with which the ESP32 wants to communicate.
#define PIN_NUM_CE   9     //used to control the NRF24L01 module's operation mode.


#define ON_BOARD_LED 5

#define SERVO_1 4

//#define VERT_PIN 0
//#define HORZ_PIN 1
//#define SEL_PIN  2

//#define BUTTON_IS_CLICKED(PINB,BUTTON_PIN) !BIT_CHECK(PINB,BUTTON_PIN)

void ConvertToPercentage(double*);

//void WaitForStartButtonClick();

int main()
{   

    volatile millis_t milliSecsSinceLastCheck = 0;

    double horz;
    double vert;

    //initJoystick();
    init_servo();
    init_serial();
    millis_init();
    sei();

    BIT_CLEAR(DDRB, PIN_NUM_MISO); //Pull up or down resistor?
    BIT_SET(DDRB, PIN_NUM_MOSI);
    BIT_SET(DDRB, PIN_NUM_CLK);
    BIT_SET(DDRB, PIN_NUM_CS);
    BIT_SET(DDRB, PIN_NUM_CE);

    BIT_SET(DDRB, ON_BOARD_LED); //Sätt led_pin_red till output mode
    BIT_SET(DDRD, SERVO_1); //Servo

    //spi_device_handle_t spi_device_handle; //SPI handle for the SPI communication to NRF24L01+

    SPI_init(&spi_device_handle);

    uint8_t RxAddress[] = {0xEE, 0xDD, 0xCC, 0xBB, 0xAA};
    uint8_t RxData[32];

    NRF24_Init(&spi_device_handle);

    NRF24_RXMode(RxAddress, 10, &spi_device_handle);

    while (1) {
         if (NRF24_RXisDataReady(1) == 1)
         {
            NRF24_Receive(RxAddress, );
         }
         
		//horz = analogRead(HORZ_PIN);
  		//vert = analogRead(VERT_PIN);

        //ConvertToPercentage(&horz);

        //servo1_set_percentage(horz);

        /*if (BUTTON_IS_CLICKED(PIND,SEL_PIN)){
            BIT_SET(PORTB, ON_BOARD_LED);
        }
        else{
            BIT_CLEAR(PORTB, ON_BOARD_LED);
        }*/
  	}
    return 0;
}

/*void initJoystick(){
    BIT_CLEAR(DDRC,VERT_PIN); //data direction register (DDR) is used to set the direction of a pin
	BIT_CLEAR(DDRC,HORZ_PIN); //the code clears (sets to 0) the respective bits in the DDRC register, indicating that the pins connected to the vertical and horizontal buttons should be configured as input pins.

	//Sätt till INPUT_PULLUP
    BIT_CLEAR(DDRD,SEL_PIN); // INPUT MODE
    BIT_SET(PORTD,SEL_PIN); 

}*/

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

