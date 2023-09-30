#include "RX.h"

volatile uint8_t TxData[32];

typedef struct HomeLocation
{   
    double latitude_DecimalDegrees;
    double longitude_DecimalDegrees;
    bool hasHomeLocation;
}HomeLocation;

typedef struct CurrentLocation
{   
    double latitude_DecimalDegrees;
    double longitude_DecimalDegrees;
}CurrentLocation;

HomeLocation homeLocation;
CurrentLocation currentLocation;


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

        if (!homeLocation.hasHomeLocation && TxData[GPS_FIX] == 1){
            setHomeLocationInDecimalDegrees();
            //homeLocation.latitude_DecimalDegrees = 58 + (00 / 60.0) + (6.9 / 3600.0);
            //homeLocation.longitude_DecimalDegrees = 15 + (8 / 60.0) + (54.4 / 3600.0);
            
            //north representing 0° or 360°
            //east representing 90°
            //south representing 180°
            //west representing 270°
            //58°19'51.0"N lat  to the east 
            //15°38'13.2"E lon
            //58°36'50.0"N TEST values to the north
            //15°08'28.0"E

            //58°00'06.9"N 
            //15°08'54.4"E south
        }

        ActOnReceivedData(RxData);

        

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
    
    if (switch1Down && TxData[GPS_FIX] == 1) //Return to home
    {
        returnToHome();
    }
    
    else{
        if (throttle >= 50 && throttle <= 1000)
        {
            double percent = throttle;
            ConvertToPercentage(&percent);
            //servo1_set_percentage(percent);

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
            servo1_set_percentage(percent);
        }
        
        if (elevator >= 50 && elevator <= 1000)
        {
            double percent = elevator;
            ConvertToPercentage(&percent);
            servo2_set_percentage(percent);
        }
    }
    

    //BIT_SET(PORTD, BUZZER_PIN);
    if (switch1Up)
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

    float voltage = (analogRead(BATTERY_MONITOR_PIN) / 1023.0) 
                    * REFERENCE_VOLTAGE * VOLTAGE_UPSCALE_FACTOR;

    TxData[BAT_VOLTAGE_WHOLE]   = voltage; //integer number (Heltal)
    TxData[BAT_VOLTAGE_DECIMAL] = (voltage - TxData[BAT_VOLTAGE_WHOLE]) * 100; //Decimal number
}

void init_RX(){
    BIT_CLEAR(DDRC, BATTERY_MONITOR_PIN);
    BIT_SET(DDRD, BUZZER_PIN);
    homeLocation.hasHomeLocation = false;
}

void setHomeLocationInDecimalDegrees(){

    homeLocation.latitude_DecimalDegrees = TxData[GPS_LATITUDE_DEGREES] + (TxData[GPS_LATITUDE_MIN] / 60.0) + (TxData[GPS_LATITUDE_SEC] / 3600.0);
    if (TxData[GPS_LATITUDE_DIR] == 'W') {
        homeLocation.latitude_DecimalDegrees = -homeLocation.latitude_DecimalDegrees;
    }

    homeLocation.longitude_DecimalDegrees = TxData[GPS_LONGITUDE_DEGREES] + (TxData[GPS_LONGITUDE_MIN] / 60.0) + (TxData[GPS_LONGITUDE_SEC] / 3600.0);
    if (TxData[GPS_LATITUDE_DIR] == 'S') {
        homeLocation.longitude_DecimalDegrees = -homeLocation.longitude_DecimalDegrees;
    }
    
    homeLocation.hasHomeLocation = true;
}

void updateCurrentLocationInDecimalDegrees(){

    currentLocation.latitude_DecimalDegrees = TxData[GPS_LATITUDE_DEGREES] + (TxData[GPS_LATITUDE_MIN] / 60.0) + (TxData[GPS_LATITUDE_SEC] / 3600.0);
    if (TxData[GPS_LATITUDE_DIR] == 'W') {
        currentLocation.latitude_DecimalDegrees = -currentLocation.latitude_DecimalDegrees;
    }

    currentLocation.longitude_DecimalDegrees = TxData[GPS_LONGITUDE_DEGREES] + (TxData[GPS_LONGITUDE_MIN] / 60.0) + (TxData[GPS_LONGITUDE_SEC] / 3600.0);
    if (TxData[GPS_LATITUDE_DIR] == 'S') {
        currentLocation.longitude_DecimalDegrees = -currentLocation.longitude_DecimalDegrees;
    }
    
}

void returnToHome(){
    
    updateCurrentLocationInDecimalDegrees();
    uint16_t bearingToHome = (uint16_t)getBearingToHome();
    uint16_t currentHeading = (uint16_t)calculateHeading();
    
    bearingToHome += 400; //To get rid of airplane maneuver problems if the bearing home is close to 0/360 degrees.
    currentHeading += 400;

    uint16_t altitude_GPS = (uint16_t)(((TxData[GPS_ALTITUDE_MSB]) << 8) | TxData[GPS_ALTITUDE_LSB]);
    int8_t altitudeCorrectionValue = 0;

    //Control Speed
    if (TxData[GPS_GROUND_SPEED_KMPH] < 40) //      Going too slow
    {
        //servo3_set_percentage(30); //65% throttle
    }
    else if (TxData[GPS_GROUND_SPEED_KMPH] > 60) //  Going to fast
    {
        //servo3_set_percentage(-40); //30% throttle
    }
    else{
        //servo3_set_percentage(-20); //40% throttle Cruise speed
    }


    //Control Altitude
    
    if (altitude_GPS < 100) //Airplane too low
        altitudeCorrectionValue = -10;

    else if (altitude_GPS > 110) //Airplane to high
        altitudeCorrectionValue = 10;

    else
        altitudeCorrectionValue = 0;
    
    
    
    //Control Direction using Ailerons OR rudder, combined with elevator
    if (currentHeading < bearingToHome) //Turn right, then how much to turn
    {   
        
        
        uint16_t difference = bearingToHome - currentHeading;

        TxData[29] = (uint16_t)difference >> 0;
        TxData[30] = (uint16_t)difference >> 8;
        if (difference > 50){
            servo1_set_percentage(30);
            servo2_set_percentage(-30 + altitudeCorrectionValue);
        
        }
        else if (difference <= 50 && difference > 30){
            servo1_set_percentage(20);
            servo2_set_percentage(-30 + altitudeCorrectionValue);
        }

        else if (difference <= 30 && difference > 10){
            servo1_set_percentage(10);
            servo2_set_percentage(-30 + altitudeCorrectionValue);
        }

        else if (difference <= 10 && difference > 5){
            servo1_set_percentage(5);
            servo2_set_percentage(-30 + altitudeCorrectionValue);
        }

        else if (difference <= 4){
            servo1_set_percentage(0);
            servo2_set_percentage(0 + altitudeCorrectionValue);
        }
    }
    
    else if (currentHeading >= bearingToHome) //Turn left, then how much to turn
    {   
        
        uint16_t difference = currentHeading - bearingToHome;
        TxData[29] = (uint16_t)difference >> 0;
        TxData[30] = (uint16_t)difference >> 8;
        if (difference > 50){
            servo1_set_percentage(-30);
            servo2_set_percentage(-30 + altitudeCorrectionValue);
        
        }
        else if (difference <= 50 && difference > 30){
            servo1_set_percentage(-20);
            servo2_set_percentage(-30 + altitudeCorrectionValue);
        }

        else if (difference <= 30 && difference > 10){
            servo1_set_percentage(-10);
            servo2_set_percentage(-30 + altitudeCorrectionValue);
        }

        else if (difference <= 10 && difference > 5){
            servo1_set_percentage(-5);
            servo2_set_percentage(-30 + altitudeCorrectionValue);
        }

        else if (difference <= 4){
            servo1_set_percentage(0);
            servo2_set_percentage(0 + altitudeCorrectionValue);
        }
        
    }
    
}

double getBearingToHome(){
    // Convert latitude and longitude from degrees to radians
    double currentLatHeadingRadians = currentLocation.latitude_DecimalDegrees * M_PI / 180.0;
    double currentLonHeadingRadians = currentLocation.longitude_DecimalDegrees * M_PI / 180.0;
    double homeLatHeadingRadians = homeLocation.latitude_DecimalDegrees * M_PI / 180.0; 
    double homeLonHeadingRadians = homeLocation.longitude_DecimalDegrees * M_PI / 180.0;

    double deltaLon = homeLonHeadingRadians - currentLonHeadingRadians;

    double y = sin(deltaLon) * cos(homeLatHeadingRadians);
    double x = cos(currentLatHeadingRadians) * sin(homeLatHeadingRadians) - sin(currentLatHeadingRadians) * cos(homeLatHeadingRadians) * cos(deltaLon);

    double bearingToHome = atan2(y, x);

    // Convert initial bearing from radians to degrees
    bearingToHome = bearingToHome * 180.0 / M_PI;

    // Normalize the initial bearing to the range [0, 360)
    //if (bearingToHome < 0) {
    //    bearingToHome += 360.0;
    //}

    if (bearingToHome < 0) {
        bearingToHome += 360.0;
    }   
    else if (bearingToHome >= 360.0) {
    bearingToHome -= 360.0;
    }

    return bearingToHome;
}


double calculateHeading(){

    int16_t x = 0;
        x |= (((int16_t)TxData[MAGNETIC_X_LSB]) << 0);
        x |= (((int16_t)TxData[MAGNETIC_X_MSB]) << 8);
        
    int16_t y = 0;
        y |= (((int16_t)TxData[MAGNETIC_Y_LSB]) << 0);
        y |= (((int16_t)TxData[MAGNETIC_Y_MSB]) << 8);

    double heading = atan2(y,x)*180.0/M_PI;
    heading = (heading < 0)? 360 + heading:heading;  

    return heading;
}


