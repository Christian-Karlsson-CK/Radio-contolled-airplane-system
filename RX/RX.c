#include "RX.h"

int main()
{   
    //volatile millis_t milliSecsSinceLastCheck = 0;

    init_servo();
    //init_serial();
    millis_init();
    sei();

    lcd_init();

    SPI_init();
    
    NRF24_Init();
    NRF24_RXMode();
    //NRF24_TXMode();

    BIT_CLEAR(DDRC, BATTERY_MONITOR_PIN);
    
    uint8_t TxData[32];
    uint8_t RxData[32];

    while (1) {
        /*
        #define BIT_SET(a, b) ((a) |= (1ULL << (b)))
        #define BIT_CLEAR(a,b) ((a) &= ~(1ULL<<(b)))

        BIT_SET(DDRD, 5); //PORTB pin 6
        while (true)
        {   
            BIT_SET(PORTD, 5);
            _delay_ms(1);
            BIT_CLEAR(PORTD, 5);
            _delay_ms(1);
            _delay_ms(18);
        }
        */
        

        ReceiveData(RxData);

        ActOnReceivedData(RxData);

        if (RxData[COMMAND_BYTE] == SWITCH_TO_TX_COMMAND)
        {   
            ReadBatteryVoltage(TxData);
            NRF24_TXMode();
            TransmitData(TxData);
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

    uint16_t rudder = (uint16_t)((RxData[3] << 8) | RxData[2]);
    uint16_t throttle = (uint16_t)((RxData[5] << 8) | RxData[4]);
    uint16_t ailerons = (uint16_t)((RxData[7] << 8) | RxData[6]);
    uint16_t elevator = (uint16_t)((RxData[9] << 8) | RxData[8]);
    uint16_t pot1 = (uint16_t)((RxData[11] << 8) | RxData[10]);
    uint16_t pot2 = (uint16_t)((RxData[13] << 8) | RxData[12]);
    uint8_t switch1Up  = RxData[14];
    uint8_t switch1Down  = RxData[15];
    uint8_t switch2Up  = RxData[16];
    uint8_t switch2Down  = RxData[17];

    if (rudder >= 50 && rudder <= 1000)
    {
        double percent = rudder;
        ConvertToPercentage(&percent);
        servo1_set_percentage(percent);
    }
    
    if (throttle >= 50 && throttle <= 1000)
    {
        double percent = throttle;
        ConvertToPercentage(&percent);
        servo2_set_percentage(percent);
    }

    if (ailerons >= 50 && ailerons <= 1000)
    {
        double percent = ailerons;
        ConvertToPercentage(&percent);
        //servo3_set_percentage(percent);
    }
    
    if (elevator >= 50 && elevator <= 1000)
    {
        double percent = elevator;
        ConvertToPercentage(&percent);
        //servo4_set_percentage(percent);
    }

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

    TxData[2] = voltage; //integer number (Heltal)
    TxData[3] = (voltage - TxData[2]) * 100; //Decimal number
}
