//LCD PINS
#define LCD_RS_PIN GPIO_NUM_25
//#define LCD_RW_PIN GPIO_NUM_? //Not in use
#define LCD_EN_PIN GPIO_NUM_26
#define LCD_D4_PIN GPIO_NUM_27
#define LCD_D5_PIN GPIO_NUM_14
#define LCD_D6_PIN GPIO_NUM_22
#define LCD_D7_PIN GPIO_NUM_13

//NRF24L01 PINS (SPI)
#define SPIHOST HSPI_HOST           //Using HSPI on ESP32 SPIHOST = The SPI controller peripheral inside ESP32. HSPI_HOST = SPI2_HOST=1
#define PIN_NUM_MISO GPIO_NUM_19   
#define PIN_NUM_MOSI GPIO_NUM_23
#define PIN_NUM_CLK  GPIO_NUM_18
#define PIN_NUM_CS   GPIO_NUM_17    //To the CSN pin on the LRF24L01+. used to select the specific SPI device with which the ESP32 wants to communicate.
#define PIN_NUM_CE   GPIO_NUM_16    //used to control the NRF24L01 module's operation mode.

//Joystick for throttle and rudder PINS
#define PIN_RUDDER     GPIO_NUM_36
#define PIN_THROTTLE   GPIO_NUM_39

//Switch
#define PIN_NUM_SW_UP    GPIO_NUM_32
#define PIN_NUM_SW_DOWN  GPIO_NUM_33