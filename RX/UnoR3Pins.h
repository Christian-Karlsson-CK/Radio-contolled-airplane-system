
//SPI (PORTB) NRF24L01+
#define PIN_NUM_MISO 4    
#define PIN_NUM_MOSI 3
#define PIN_NUM_CLK  5
#define PIN_NUM_CS   2    //To the CSN pin on the LRF24L01+. used to select the specific SPI device with which the ESP32 wants to communicate.
#define PIN_NUM_CE   0     //used to control the NRF24L01 module's operation mode.

//Main flight controls on PORTD and PORTB
#define PIN_NUM_ELEVATOR 6 //PORTD
#define PIN_NUM_AILERONS 5 //PORTD
#define PIN_NUM_THROTTLE 3 //PORTD
#define PIN_NUM_RUDDER   1 //PORTB

//Battery PORTC
#define BATTERY_MONITOR_PIN 0









