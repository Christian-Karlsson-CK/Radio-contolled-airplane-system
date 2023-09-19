#include "RX.h"

uint8_t TxData[32];

int main()
{   
    //volatile millis_t milliSecsSinceLastCheck = 0;

    init_RX();

    init_servo();
    //init_uart();
    //millis_init();
    sei(); // Enable global interrupts

    lcd_init();

    SPI_init();

    I2C_init();

    BMP280_init();
    GY271_init();

    
    NRF24_Init();
    NRF24_RXMode();
    //NRF24_TXMode();

    uint8_t RxData[32];
    uint8_t counter = 0;

    while (1) {
        
        /*************************************************************************************************/
        //GY-271 TEST:

        /*
        counter++;
        //ReadBatteryVoltage(TxData);
        TxData[2] = counter + 1;
        TxData[3] = counter + 1;
        
        TxData[15] = GY271_ReadRegister(GY271_X_LSB_REG);
        TxData[16] = GY271_ReadRegister(GY271_X_MSB_REG);

        TxData[17] = GY271_ReadRegister(GY271_Y_LSB_REG);
        TxData[18] = GY271_ReadRegister(GY271_Y_MSB_REG);

        TxData[19] = GY271_ReadRegister(GY271_Z_LSB_REG);
        TxData[20] = GY271_ReadRegister(GY271_Z_MSB_REG);

        TxData[21] = GY271_ReadRegister(GY271_STATUS_REG);

        GY271_GetHeading(TxData);
        
        


        TransmitData(TxData);

        
        _delay_ms(2000);
        */
        /*************************************************************************************************/

        /*REGULAR CODE*************************************************************************************/
        
        
        ReceiveData(RxData);

        ActOnReceivedData(RxData);

        if (RxData[COMMAND_BYTE] == SWITCH_TO_TX_COMMAND)
        {   
            //Prepare transmit buffer with sensor readings
            ReadBatteryVoltage(TxData);
            BMP280_ReadTempAndPressure(TxData);
            GY271_ReadXAndY(TxData);
            
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

    uint16_t rudder = (uint16_t)((RxData[19] << 8) | RxData[18]);
    uint16_t throttle = (uint16_t)((RxData[3] << 8) | RxData[2]);
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
        servo2_set_percentage(percent);
 
    }
    
    if (throttle >= 50 && throttle <= 1000)
    {
        double percent = throttle;
        ConvertToPercentage(&percent);
        servo1_set_percentage(percent);

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

void init_RX(){
    BIT_CLEAR(DDRC, BATTERY_MONITOR_PIN);
}

void updateTxDataWithGps(uint8_t data){
    TxData[5] = data;
}
