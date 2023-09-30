
/*
Hjälp video:
https://www.youtube.com/watch?v=oHHOCdmLiII&list=PLHX2-9M57aE5LVZnGwbo2cjnqvLqtu2OS&index=3

idf.py create-project -p . [PROJECT_NAME]
	Använd detta kommando för att skapa ett nytt projekt

idf.py build
	build projektet

idf.py monitor -p COM3
	få in händelser direkt från MCUn

idf.py flash -p COM3 monitor
    builda & flasha till MCU, monitor är optional om man vill övervaka.
*/
#include "TX.h"

static const char *TAG = "Debug:";

uint8_t TxData[32];
uint8_t RxData[32];

int sendCounter = 0; //Testing

void app_main(void)
{   
    InitControls();
    lcd_init();
    SPI_init();
    NRF24_Init();
    NRF24_TXMode();
    //NRF24_RXMode(); //Testing
    
    uint8_t TX_RX_Switch_counter = 0; //This counter is used to switch between RX and TX mode. TX mode sends all analog stick and potentiometer values.
                                      //RX mode is used to receive status data from the RX on the airplane, for instance Battery voltage. 

    while (true)
    {   
        TX_RX_Switch_counter++;

        //TX MODE, SEND CONTROL DATA
        if (TX_RX_Switch_counter < TX_TO_RX_THRESHOLD)
        {   
            ReadAllAnalog(TxData);
            TxData[COMMAND_BYTE] = NO_COMMAND;

            TransmitData(TxData);
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        //RX MODE, SEND COMMAND TO MAKE RX(ON AIRPLANE SIDE) SWITCH TO TX MODE
        else if (TX_RX_Switch_counter >= TX_TO_RX_THRESHOLD)
        {   
            TxData[COMMAND_BYTE] = SWITCH_TO_TX_COMMAND;
            TransmitData(TxData);
            TransmitData(TxData);
            TransmitData(TxData);
            
            sendCounter++; //Testing
            
            NRF24_RXMode();
            //vTaskDelay(pdMS_TO_TICKS(0.5));

            bool hasReceived =  listenForIncomingRadioTransmission(MAX_ATTEMPTS);
            
            if (hasReceived)
            {   
                NRF24_Receive(RxData);
                //updateLCD();
            }
            
            updateLCD();
            TX_RX_Switch_counter = 0;
            NRF24_TXMode();
        }
    }
    
}


void InitControls(){
    //Joysticks
    gpio_set_direction(PIN_RUDDER,   GPIO_MODE_INPUT);
    gpio_set_direction(PIN_THROTTLE, GPIO_MODE_INPUT); 
    gpio_set_direction(PIN_AILERONS, GPIO_MODE_INPUT);
    gpio_set_direction(PIN_ELEVATOR, GPIO_MODE_INPUT);

    adc1_config_width(ADC_WIDTH_BIT_10); // Analog digital converter, 10bit = 0-1023
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11); //Throttle
    adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11); //Rudder

    adc2_config_channel_atten(ADC2_CHANNEL_0, ADC_ATTEN_DB_11); //Ailerons
    adc2_config_channel_atten(ADC2_CHANNEL_2, ADC_ATTEN_DB_11); //Elevator

    //Switches
    gpio_set_direction(PIN_SWITCH1_UP,   GPIO_MODE_INPUT);
    gpio_set_direction(PIN_SWITCH1_DOWN, GPIO_MODE_INPUT);
    gpio_set_direction(PIN_SWITCH2_UP,   GPIO_MODE_INPUT);
    gpio_set_direction(PIN_SWITCH2_DOWN, GPIO_MODE_INPUT);

    //Potentiometers
    gpio_set_direction(PIN_POT1, GPIO_MODE_INPUT);
    gpio_set_direction(PIN_POT2, GPIO_MODE_INPUT);

    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11);   //POT1
    adc2_config_channel_atten(ADC2_CHANNEL_4, ADC_ATTEN_DB_11);   //POT2
}


void TransmitData(uint8_t *TxData){

    if(NRF24_Transmit(TxData)){
        //ESP_LOGI(TAG, "SEEMS TO TRANSMIT");
    }
    NRF24_WriteRegister(STATUS, CLEAR_TX_DS);
}

void ReadAllAnalog(uint8_t *TxData){
    uint16_t rudder =   adc1_get_raw(ADC1_CHANNEL_3);
    uint16_t throttle = adc1_get_raw(ADC1_CHANNEL_0);

    uint16_t ailerons;
    adc2_get_raw(ADC2_CHANNEL_0, ADC_WIDTH_BIT_10, &ailerons);
    uint16_t elevator;
    adc2_get_raw(ADC2_CHANNEL_2, ADC_WIDTH_BIT_10, &elevator);

    uint16_t pot1 = adc1_get_raw(ADC1_CHANNEL_4);
    uint16_t pot2;
    adc2_get_raw(ADC2_CHANNEL_4, ADC_WIDTH_BIT_10, &pot2);

    uint8_t switch1Up =   gpio_get_level(PIN_SWITCH1_UP);
    uint8_t switch1Down = gpio_get_level(PIN_SWITCH1_DOWN);

    uint8_t switch2Up =   gpio_get_level(PIN_SWITCH2_UP);
    uint8_t switch2Down = gpio_get_level(PIN_SWITCH2_DOWN);

    
    TxData[THROTTLE_LSB] =  (uint8_t)(throttle);
    TxData[THROTTLE_MSB] =  (uint8_t)(throttle >> 8);
    TxData[RUDDER_LSB]   =  (uint8_t)(rudder); 
    TxData[RUDDER_MSB]   =  (uint8_t)(rudder >> 8);
    TxData[AILERONS_LSB] =  (uint8_t)(ailerons);
    TxData[AILERONS_MSB] =  (uint8_t)(ailerons >> 8);
    TxData[ELEVATOR_LSB] =  (uint8_t)(elevator);
    TxData[ELEVATOR_MSB] =  (uint8_t)(elevator >> 8);
    TxData[POT1_LSB] = (uint8_t)(pot1);
    TxData[POT1_MSB] = (uint8_t)(pot1 >> 8);
    TxData[POT2_LSB] = (uint8_t)(pot2);
    TxData[POT2_MSB] = (uint8_t)(pot2 >> 8);
    TxData[SWITCH1UP] = switch1Up;
    TxData[SWITCH1DOWM] = switch1Down;
    TxData[SWITCH2UP] = switch2Up;
    TxData[SWITCH2DOWM] = switch2Down;

    /*
    uint16_t rudderT = (uint16_t)((TxData[5] << 8) | TxData[4]);
    uint16_t throttleT = (uint16_t)((TxData[3] << 8) | TxData[2]);
    uint16_t aileronsT = (uint16_t)((TxData[7] << 8) | TxData[6]);
    uint16_t elevatorT = (uint16_t)((TxData[9] << 8) | TxData[8]);
    uint16_t pot1T = (uint16_t)((TxData[11] << 8) | TxData[10]);
    uint16_t pot2T = (uint16_t)((TxData[13] << 8) | TxData[12]);
    
    ESP_LOGI(TAG, "rudder: %u, throttle: %u", rudderT, throttleT);
    ESP_LOGI(TAG, "ailerons: %u, elevator: %u", aileronsT, elevatorT);
    ESP_LOGI(TAG, "Pot1: %u, Pot2: %u", pot1T, pot2T);
    ESP_LOGI(TAG, "sw1Up: %u, sw1Down: %u", TxData[14], TxData[15]);
    ESP_LOGI(TAG, "sw2Up: %u, sw2Down: %u", TxData[16], TxData[17]);
    

    vTaskDelay(pdMS_TO_TICKS(1000));
    */
    
}

uint8_t listenForIncomingRadioTransmission(uint8_t maxTries){
    uint8_t tries = 0;

    //LISTEN FOR INCOMMING TRANSMISSION FROM TX (AIRPLANE SIDE) MAXIMUM 5 TRIES THEN BACK TO TX MODE
    while (!NRF24_RXisDataReady(0) && tries < maxTries)
    {   
        ESP_LOGI(TAG, "NO MessageXXXXXXXXXXXXX");
        vTaskDelay(pdMS_TO_TICKS(0.2));
        tries++;    
    }

    if (tries < maxTries){
        ESP_LOGI(TAG, "Message Received!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        return 1;
    }
    else
        return 0;
}

void updateLCD(){

    if (RxData[BAT_VOLTAGE_WHOLE] > 99)
        RxData[BAT_VOLTAGE_WHOLE] = 0;
    
    if (RxData[BAT_VOLTAGE_DECIMAL] > 99)
        RxData[BAT_VOLTAGE_DECIMAL] = 0;

    RxData[BAT_VOLTAGE_WHOLE] = 12;
    RxData[BAT_VOLTAGE_DECIMAL] = 5;

    int16_t x = 0;
        x |= (((int16_t)RxData[MAGNETIC_X_LSB]) << 0);
        x |= (((int16_t)RxData[MAGNETIC_X_MSB]) << 8);
        
    int16_t y = 0;
        y |= (((int16_t)RxData[MAGNETIC_Y_LSB]) << 0);
        y |= (((int16_t)RxData[MAGNETIC_Y_MSB]) << 8);

    uint32_t bmp280_pressure = (((uint32_t)RxData[PRESSURE_LSB_0]) << 0)
                             | (((uint32_t)RxData[PRESSURE_MID_8]) << 8)
                             | (((uint32_t)RxData[PRESSURE_MID_16]) << 16)
                             | (((uint32_t)RxData[PRESSURE_MSB_24]) << 24);

    int32_t bmp280_temp = (((uint32_t)RxData[TEMPERTURE_LSB_0]) << 0)
                        | (((uint32_t)RxData[TEMPERTURE_MID_8]) << 8)
                        | (((uint32_t)RxData[TEMPERTURE_MID_16]) << 16)
                        | (((uint32_t)RxData[TEMPERTURE_MSB_24]) << 24);

    int8_t temp_whole = bmp280_temp / 100;
    uint8_t temp_decimal = (bmp280_temp - (temp_whole*100));
    if (temp_decimal >= 10)
        temp_decimal /= 10;

    uint16_t altitude_GPS = (uint16_t)(((RxData[GPS_ALTITUDE_MSB]) << 8) | RxData[GPS_ALTITUDE_LSB]);

    uint16_t heading = (uint16_t)CalculateHeading(x, y);

    double altitude_BMP280 = CalculateAltitude(bmp280_pressure);


    uint16_t test = 0;
        test |= (((uint16_t)RxData[30]) << 0);
        test |= (((uint16_t)RxData[31]) << 8);

    uint8_t test2 = RxData[32];
    
    if (TxData[SWITCH2UP] == 0 && TxData[SWITCH2DOWM] == 0)
    {
        lcd_set_cursor(0,0);
        lcd_printf("BAT=%.2u.%.1u HDG=%u  ", RxData[BAT_VOLTAGE_WHOLE], RxData[BAT_VOLTAGE_DECIMAL], heading);

        lcd_set_cursor(0,1);
        if (RxData[GPS_FIX] == 1)
            lcd_printf("ALT=%uM KMH=%u    ", altitude_GPS, RxData[GPS_GROUND_SPEED_KMPH]);
        else
            lcd_printf("---NO GPS FIX---");
    }

    else if (TxData[SWITCH2UP] == 1)
    {
        lcd_set_cursor(0,0);
        lcd_printf("FIX=%u  CEL=%d.%u  ", RxData[GPS_FIX], temp_whole, temp_decimal);
        
        lcd_set_cursor(0,1);
        lcd_printf("SAT=%u  PA=%u     ", RxData[GPS_SATELLITE_COUNT], bmp280_pressure);  
    }
    
    else if (TxData[SWITCH2DOWM] == 1)
    {   
        
        if (RxData[GPS_FIX] == 1)
        {
            lcd_set_cursor(0,0);
            lcd_printf("LAT=%u.%u%u%c     ", RxData[GPS_LATITUDE_DEGREES], RxData[GPS_LATITUDE_MIN],
                                             RxData[GPS_LATITUDE_SEC],     RxData[GPS_LATITUDE_DIR]);
            lcd_set_cursor(0,1);
            lcd_printf("LON=%u.%u%u%c     ", RxData[GPS_LONGITUDE_DEGREES], RxData[GPS_LONGITUDE_MIN], 
                                             RxData[GPS_LONGITUDE_SEC],     RxData[GPS_LONGITUDE_DIR]);
        }
        else
        {
            lcd_set_cursor(0,0);
            lcd_printf("----------------");
            
            lcd_set_cursor(0,1);
            lcd_printf("---NO GPS FIX---");    
        }
        
    }
    //vTaskDelay(pdMS_TO_TICKS(10));
}


double CalculateAltitude(uint32_t pressure){
    return (1 - pow(pressure / (double)103490, 0.1903)) / 0.0000225577; //101325
}

double CalculateHeading(int16_t x, int16_t y){
    double heading = atan2(y,x)*180.0/M_PI;
    heading = (heading < 0)? 360 + heading:heading;  

    return heading;
}

