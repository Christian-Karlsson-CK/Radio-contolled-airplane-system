#ifndef _UNOR3PINS_H_
#define _UNOR3PINS_H_

//Main flight controls on PORTD and PORTB
#define PIN_NUM_ELEVATOR PD6 //PORTD
#define PIN_NUM_AILERONS PD5 //PORTD
#define PIN_NUM_THROTTLE PD3 //PORTD
#define PIN_NUM_RUDDER   PB1 //PORTB

//NRF24L01+ Transceiver, SPI (PORTB) 
#define PIN_NUM_MISO PB4    
#define PIN_NUM_MOSI PB3
#define PIN_NUM_CLK  PB5
#define PIN_NUM_CS   PB2   //To the CSN pin on the LRF24L01+. used to select the specific SPI device with which the ESP32 wants to communicate.
#define PIN_NUM_CE   PB0    //used to control the NRF24L01 module's operation mode.

//GY-NEO6MV2 GPS, UART (PORTD)
#define PIN_NUM_TX   PD1
#define PIN_NUM_RX   PD0

//GY-BMP280 BAROMETRIC PRESSURE & TEMPERATURE SENSOR (PORTC)
#define PIN_NUM_SDA  PC4
#define PIN_NUM_SCL  PC5

//BATTERY MONITOR PORTC
#define BATTERY_MONITOR_PIN PC0


#endif /* _UNOR3PINS_H_ */






