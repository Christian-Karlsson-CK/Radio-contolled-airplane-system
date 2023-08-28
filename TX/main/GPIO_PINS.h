#ifndef _GPIO_PINS_H_
#define _GPIO_PINS_H_


//LCD PINS
#define LCD_RS_PIN GPIO_NUM_33
//#define LCD_RW_PIN GPIO_NUM_? //Not in use
#define LCD_EN_PIN GPIO_NUM_25
#define LCD_D4_PIN GPIO_NUM_26
#define LCD_D5_PIN GPIO_NUM_27
#define LCD_D6_PIN GPIO_NUM_14
#define LCD_D7_PIN GPIO_NUM_16

//NRF24L01 PINS (SPI)
#define SPIHOST HSPI_HOST           //Using HSPI on ESP32 SPIHOST = The SPI controller peripheral inside ESP32. HSPI_HOST = SPI2_HOST=1
#define PIN_NUM_MISO GPIO_NUM_19   
#define PIN_NUM_MOSI GPIO_NUM_23
#define PIN_NUM_CLK  GPIO_NUM_18
#define PIN_NUM_CS   GPIO_NUM_5    //To the CSN pin on the LRF24L01+. used to select the specific SPI device with which the ESP32 wants to communicate.
#define PIN_NUM_CE   GPIO_NUM_17    //used to control the NRF24L01 module's operation mode.

//Joystick potentiometers
#define PIN_RUDDER     GPIO_NUM_39
#define PIN_THROTTLE   GPIO_NUM_36
#define PIN_AILERONS   GPIO_NUM_4
#define PIN_ELEVATOR   GPIO_NUM_2

//Switches 3-way
#define PIN_SWITCH1_UP   GPIO_NUM_34
#define PIN_SWITCH1_DOWN GPIO_NUM_35
#define PIN_SWITCH2_UP   GPIO_NUM_22
#define PIN_SWITCH2_DOWN GPIO_NUM_21
//#define PIN_SWITCH3_UP   GPIO_NUM_3
//#define PIN_SWITCH3_DOWN GPIO_NUM_21
//#define PIN_SWITCH4_UP   GPIO_NUM_2
//#define PIN_SWITCH4_DOWN GPIO_NUM_15

//POTENTIOMETERS
#define PIN_POT1  GPIO_NUM_32
#define PIN_POT2  GPIO_NUM_13

#endif /* _GPIO_PINS_H_ */