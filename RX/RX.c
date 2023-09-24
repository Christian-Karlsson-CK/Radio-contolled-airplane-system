#include "RX.h"

volatile uint8_t TxData[32];

typedef struct HomeLocation
{   
    uint8_t latitude_deg;
    uint8_t latitude_min;
    uint8_t latitude_sec;
    uint8_t latitude_dir;
    uint8_t longitude_deg;
    uint8_t longitude_min;
    uint8_t longitude_sec;
    uint8_t longitude_dir;
    bool hasHomeLocation;
}HomeLocation;

HomeLocation homeLocation;

int main()
{   
    init_RX();
    init_servo();
    uart_init();
    lcd_init();
    SPI_init();
    I2C_init();
    BMP280_init();
    GY271_init();
    NRF24_Init();
    NRF24_RXMode();
    //NRF24_TXMode();

    sei(); // Enable global interrupts

    uint8_t RxData[32];
    uint8_t counter = 0; //Testing
    uint8_t receivecounter = 0; //Testing

    while (1) {
        /*REGULAR CODE*************************************************************************************/
        
        ReceiveData(RxData);

        ActOnReceivedData(RxData);

        if (!homeLocation.hasHomeLocation && TxData[GPS_FIX] == 1)
            getHomeLocation();
        

        if (RxData[COMMAND_BYTE] == SWITCH_TO_TX_COMMAND)
        {   
            //Prepare transmit buffer with sensor readings
            //ReadBatteryVoltage(TxData);
            receivecounter++; //Testing
            TxData[2] = receivecounter; //Testing

            if (receivecounter >= 99)
            {
                receivecounter = 0;
            }
            
            BMP280_ReadTempAndPressure(TxData);
            GY271_ReadXAndY(TxData);
            //GPS data comes from the interrupt

            NRF24_TXMode();
            TransmitData(TxData);
            _delay_ms(1);
            NRF24_RXMode();
        }
        _delay_ms(5);// 41 seems to be good delay
        
        
    }
    //return 0;
}

void ConvertToPercentage(double *reading){ //Works with values from 0-1023.
    *reading -= 512;
    *reading /= 512;
    *reading *= 100;
}

uint8_t ReceiveData(uint8_t *RxData){

    if (NRF24_RXisDataReady(0) == 1)
    {   
        NRF24_Receive(RxData);
        return 1;
    }
    
    else
        return 0;
    
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
}

void ActOnReceivedData(uint8_t *RxData){

    uint16_t throttle = (uint16_t)((RxData[THROTTLE_MSB] << 8) | RxData[THROTTLE_LSB]);
    uint16_t rudder   = (uint16_t)((RxData[RUDDER_MSB]   << 8) | RxData[RUDDER_LSB]);
    uint16_t ailerons = (uint16_t)((RxData[AILERONS_MSB] << 8) | RxData[AILERONS_LSB]);
    uint16_t elevator = (uint16_t)((RxData[ELEVATOR_MSB] << 8) | RxData[ELEVATOR_LSB]);
    uint16_t pot1     = (uint16_t)((RxData[POT1_MSB]     << 8) | RxData[POT1_LSB]);
    uint16_t pot2     = (uint16_t)((RxData[POT2_MSB]     << 8) | RxData[POT2_LSB]);
    uint8_t switch1Up    = RxData[SWITCH1UP];
    uint8_t switch1Down  = RxData[SWITCH1DOWM];
    uint8_t switch2Up    = RxData[SWITCH2UP];
    uint8_t switch2Down  = RxData[SWITCH2DOWM];
    
    if (switch1Up) //Return to home
    {
        steerHome();
    }
    else{
        if (throttle >= 50 && throttle <= 1000)
        {
            double percent = throttle;
            ConvertToPercentage(&percent);
            servo1_set_percentage(percent);

        }

        if (rudder >= 50 && rudder <= 1000)
        {
            double percent = rudder;
            ConvertToPercentage(&percent);
            //servo1_set_percentage(percent);
    
        }
        
        if (ailerons >= 50 && ailerons <= 1000)
        {
            double percent = ailerons;
            ConvertToPercentage(&percent);
            servo2_set_percentage(percent);
        }
        
        if (elevator >= 50 && elevator <= 1000)
        {
            double percent = elevator;
            ConvertToPercentage(&percent);
            //servo2_set_percentage(percent);
        }
    }

    //BIT_SET(PORTD, BUZZER_PIN);
    if (switch1Down)
        BIT_SET(PORTD, BUZZER_PIN);
    else
        BIT_CLEAR(PORTD, BUZZER_PIN);
    

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

}

ReadBatteryVoltage(uint8_t *TxData){

    float voltage = (analogRead(BATTERY_MONITOR_PIN) / 1023.0) * REFERENCE_VOLTAGE * VOLTAGE_UPSCALE_FACTOR;

    TxData[BAT_VOLTAGE_WHOLE]   = voltage; //integer number (Heltal)
    TxData[BAT_VOLTAGE_DECIMAL] = (voltage - TxData[2]) * 100; //Decimal number
}

void init_RX(){
    BIT_CLEAR(DDRC, BATTERY_MONITOR_PIN);
    BIT_SET(DDRD, BUZZER_PIN);
    homeLocation.hasHomeLocation = false;
}

void getHomeLocation(){
    homeLocation.latitude_deg = TxData[GPS_LATITUDE_DEGREES];
    homeLocation.latitude_min = TxData[GPS_LATITUDE_MIN];
    homeLocation.latitude_sec = TxData[GPS_LATITUDE_SEC];
    homeLocation.latitude_dir = TxData[GPS_LATITUDE_DIR];

    homeLocation.longitude_deg = TxData[GPS_LONGITUDE_DEGREES];
    homeLocation.longitude_min = TxData[GPS_LONGITUDE_MIN];
    homeLocation.longitude_sec = TxData[GPS_LONGITUDE_SEC];
    homeLocation.longitude_dir = TxData[GPS_LONGITUDE_DIR];

    homeLocation.hasHomeLocation = true;
}

void steerHome(){}
