
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
#include <math.h>

static const char *TAG = "Debug:";

void app_main(void)
{   
    InitControls();
    lcd_init();

    spi_device_handle_t spi_device_handle; //SPI handle for the SPI communication to NRF24L01+
    SPI_init(&spi_device_handle);

    uint8_t TxData[32];
    uint8_t RxData[32];

    NRF24_Init(&spi_device_handle);

    NRF24_TXMode(&spi_device_handle);

    uint8_t customDegreeSymbol[8] = {
        0b00110,
        0b01001,
        0b01001,
        0b00110,
        0b00000,
        0b00000,
        0b00000,
        0b00000
    };

    int receptionCounter = 0;

    //NRF24_RXMode(&spi_device_handle);
    
    uint8_t TX_RX_Switch_counter = 0; //This counter is used to switch between RX and TX mode. TX mode sends all analog stick and potentiometer values.
                                      //RX mode is used to receive status data from the RX on the airplane, for instance Battery voltage. 

    while (true)
    {   
        /*************************************************************************************************/
        //GPS TEST:
        /*
        if(!ReceiveData(&spi_device_handle, RxData))
            {   
                ESP_LOGI(TAG, "NO MessageXXXXXXXXXXXXXXX");
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            else
            {
                ESP_LOGI(TAG, "Message Received!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

                uint8_t whole = RxData[3];
                uint8_t decimal = RxData[4];

                //uint8_t groundSpeed_whole = RxData[6];
                //uint8_t groundSpeed_decimal = RxData[7];

                //uint8_t token = RxData[8];

                //uint8_t dhopWhole = RxData[10];
                //uint8_t dhopDecimal = RxData[];

                uint8_t inter = RxData[16];

                uint8_t gpsData0 = RxData[17];
                uint8_t gpsData0inif = RxData[18];
                uint8_t gpsData1inif = RxData[19];
                uint8_t gpsData2inif = RxData[20];
                uint8_t gpsData3inif = RxData[21];
                uint8_t gpsData4inif = RxData[22];
                uint8_t gpsData5inif = RxData[23];

                //uint8_t fix = RxData[13];

                //uint8_t ext = RxData[16];

                uint8_t latitude_DEG = RxData[GPS_LATITUDE_DEGREES];
                uint8_t latitude_MIN = RxData[GPS_LATITUDE_MIN];
                uint8_t latitude_SEC = RxData[GPS_LATITUDE_SEC];
                uint8_t latitude_DIR = RxData[GPS_LATITUDE_DIR];

                uint8_t longitude_DEG = RxData[GPS_LONGITUDE_DEGREES];
                uint8_t longitude_MIN = RxData[GPS_LONGITUDE_MIN];
                uint8_t longitude_SEC = RxData[GPS_LONGITUDE_SEC];
                uint8_t longitude_DIR = RxData[GPS_LONGITUDE_DIR];

                uint8_t fix = RxData[GPS_FIX];

                uint8_t sat = RxData[GPS_SATELLITE_COUNT];

                uint8_t kmph = RxData[GPS_GROUND_SPEED_KMPH];

                uint16_t altitude = (uint16_t)(((RxData[GPS_ALTITUDE_MSB]) << 8) | RxData[GPS_ALTITUDE_LSB]);
                    //altitude |= (((int16_t)RxData[GPS_ALTITUDE_LSB]) << 0);
                    //altitude |= (((int16_t)RxData[GPS_ALTITUDE_MSB]) << 8);

                

                
                lcd_clear();
                vTaskDelay(pdMS_TO_TICKS(100));

                lcd_set_cursor(0,0);
                if ((whole > 99) | (decimal > 99))
                {
                    lcd_printf("00.0V");
                }
                else{
                    decimal /= 10;
                    lcd_printf("%.2u.%.1uV", whole, decimal);
                }

                //lcd_set_cursor(0,1);
                //lcd_printf("S:%u.%u", groundSpeed_whole, groundSpeed_decimal);

                //lcd_set_cursor(6,0);
                //lcd_printf("I:%c", inter);

                lcd_set_cursor(6,0);
                lcd_printf("G:%u", kmph);

                lcd_set_cursor(9,0);
                lcd_printf("LAS:%u", latitude_SEC);

                lcd_set_cursor(9,1);
                lcd_printf("LOS:%u", longitude_SEC);

                lcd_set_cursor(0,1);
                lcd_printf("ALT:%uM", altitude);

                //lcd_set_cursor(14,0);
                //lcd_printf("%c", gpsData0);

                //lcd_set_cursor(10,1);
                //lcd_printf("%c%c%c%c%c%c", gpsData0inif, gpsData1inif, gpsData2inif, gpsData3inif, gpsData4inif, gpsData5inif);

                //lcd_set_cursor(0,1);
                //lcd_printf("ALT:%u", altitude);

                //lcd_set_cursor(9,0);
                //lcd_printf("LSB:%u", RxData[GPS_ALTITUDE_LSB]);
                //lcd_set_cursor(9,1);
                //lcd_printf("MSB:%u", RxData[GPS_ALTITUDE_MSB]);
                
                vTaskDelay(pdMS_TO_TICKS(1000));

            }
            */

        /*************************************************************************************************/

        /*REGULAR CODE*************************************************************************************/
        
        TX_RX_Switch_counter++;

        //NORMAL TX MODE TO SEND CONTROL DATA
        if (TX_RX_Switch_counter < TX_TO_RX_THRESHOLD)
        {   
            ReadAllAnalog(TxData);
            TxData[COMMAND_BYTE] = NO_COMMAND;

            TransmitData(TxData, &spi_device_handle);
            vTaskDelay(pdMS_TO_TICKS(10));

        }

        //SEND COMMAND TO MAKE RX(ON AIRPLANE SIDE) SWITCH TO TX MODE
        else if (TX_RX_Switch_counter >= TX_TO_RX_THRESHOLD)
        {   
            TxData[COMMAND_BYTE] = SWITCH_TO_TX_COMMAND;
            TransmitData(TxData, &spi_device_handle);

            NRF24_RXMode(&spi_device_handle);
            //vTaskDelay(pdMS_TO_TICKS(2));
            uint8_t tries = 0;

            //LISTEN FOR INCOMMING TRANSMISSION FROM TX (AIRPLANE SIDE) MAXIMUM 5 TRIES THEN BACK TO NORMAL TX MODE
            while (!ReceiveData(&spi_device_handle, RxData) && tries < 5)
            {   
                ESP_LOGI(TAG, "NO MessageXXXXXXXXXXXXX");
                vTaskDelay(pdMS_TO_TICKS(0.5));
                tries++;
                
            }
            if (tries < 5)
            {   
                //lcd_clear();
                ESP_LOGI(TAG, "Message Received!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                receptionCounter++;
                uint8_t whole = RxData[BAT_VOLTAGE_WHOLE];
                uint8_t decimal = RxData[BAT_VOLTAGE_DECIMAL];

                int16_t x = 0;
                    x |= (((int16_t)RxData[MAGNETIC_X_MSB]) << 0);
                    x |= (((int16_t)RxData[MAGNETIC_X_LSB]) << 8);
                   
                int16_t y = 0;
                    y |= (((int16_t)RxData[MAGNETIC_Y_LSB]) << 0);
                    y |= (((int16_t)RxData[MAGNETIC_Y_MSB]) << 8);

                uint32_t bmp280_pressure = 0;
                    bmp280_pressure |= (((uint32_t)RxData[PRESSURE_LSB_0]) << 0);
                    bmp280_pressure |= (((uint32_t)RxData[PRESSURE_MID_8]) << 8);
                    bmp280_pressure |= (((uint32_t)RxData[PRESSURE_MID_16]) << 16);
                    bmp280_pressure |= (((uint32_t)RxData[PRESSURE_MSB_24]) << 24);

                /*int32_t bmp280_temp = 0;
                    bmp280_temp |= (((int32_t)RxData[11]) << 24);
                    bmp280_temp |= (((int32_t)RxData[10]) << 16);
                    bmp280_temp |= (((int32_t)RxData[9]) << 8);
                    bmp280_temp |= (((int32_t)RxData[8]) << 0);
                */
                

                //int16_t heading = 0;
                //    heading |= (((int16_t)RxData[17]) << 8);
                //    heading |= (((int16_t)RxData[16]) << 0);

                //HERE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                uint8_t latitude_DEG = RxData[GPS_LATITUDE_DEGREES];
                uint8_t latitude_MIN = RxData[GPS_LATITUDE_MIN];
                uint8_t latitude_SEC = RxData[GPS_LATITUDE_SEC];
                uint8_t latitude_DIR = RxData[GPS_LATITUDE_DIR];

                uint8_t longitude_DEG = RxData[GPS_LONGITUDE_DEGREES];
                uint8_t longitude_MIN = RxData[GPS_LONGITUDE_MIN];
                uint8_t longitude_SEC = RxData[GPS_LONGITUDE_SEC];
                uint8_t longitude_DIR = RxData[GPS_LONGITUDE_DIR];

                uint8_t fix = RxData[GPS_FIX];

                uint8_t sat = RxData[GPS_SATELLITE_COUNT];

                uint8_t kmph = RxData[GPS_GROUND_SPEED_KMPH];

                uint16_t altitude_GPS = (uint16_t)(((RxData[GPS_ALTITUDE_MSB]) << 8) | RxData[GPS_ALTITUDE_LSB]);

                float heading = CalculateHeading(x, y); 

                double altitude_BMP280 = CalculateAltitude(bmp280_pressure);

                //lcd_clear();
                //vTaskDelay(pdMS_TO_TICKS(100));

                //lcd_set_cursor(0,1);
                //lcd_printf("S:%u.%u", groundSpeed_whole, groundSpeed_decimal);

                //lcd_set_cursor(6,0);
                //lcd_printf("I:%c", inter);

                //lcd_set_cursor(6,0);
                //lcd_printf("G:%u", kmph);

                lcd_set_cursor(9,0);
                lcd_printf("LAS:%u", latitude_DEG);

                lcd_set_cursor(9,1);
                lcd_printf("LOS:%u", longitude_DEG);

                lcd_set_cursor(0,1);
                lcd_printf("ALT:%uM", altitude_GPS);

                //lcd_set_cursor(14,0);
                //lcd_printf("%c", gpsData0);

                //lcd_set_cursor(10,1);
                //lcd_printf("%c%c%c%c%c%c", gpsData0inif, gpsData1inif, gpsData2inif, gpsData3inif, gpsData4inif, gpsData5inif);

                //lcd_set_cursor(0,1);
                //lcd_printf("ALT:%u", altitude_BMP280);

                //lcd_set_cursor(9,0);
                //lcd_printf("LSB:%u", RxData[GPS_ALTITUDE_LSB]);
                //lcd_set_cursor(9,1);
                //lcd_printf("MSB:%u", RxData[GPS_ALTITUDE_MSB]);

                //UNTIL HERE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                
                
                
                lcd_set_cursor(0,0);
                if ((whole > 99) | (decimal > 99))
                {
                    lcd_printf("00.0V");
                }
                else{
                    decimal /= 10;
                    lcd_printf("%.2u.%.1uV", whole, decimal);
                }

                
                //lcd_set_cursor(6,0);
                //lcd_printf("R:%d", receptionCounter);

                //lcd_set_cursor(5,1);
                //lcd_printf("     ");

                //lcd_set_cursor(0,1);
                //lcd_printf("ALT:%dM", (int)altitude_BMP280);

                //lcd_set_cursor(10,1);
                //lcd_printf("H:%d", (int)heading);
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            
            tries = 0;
            TX_RX_Switch_counter = 0;
            NRF24_TXMode(&spi_device_handle);
        }
        
        /*************************************************************************************************/
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


void TransmitData(uint8_t *TxData, spi_device_handle_t *spi_device_handle){

    if(NRF24_Transmit(TxData, spi_device_handle)){
        //ESP_LOGI(TAG, "SEEMS TO TRANSMIT");
    }
    NRF24_WriteRegister(STATUS, CLEAR_TX_DS, spi_device_handle);
}

uint8_t ReceiveData(spi_device_handle_t *spi_device_handle, uint8_t *RxData){

    if (NRF24_RXisDataReady(0, spi_device_handle) == 1)
    {   
        //ESP_LOGI(TAG, "We have data available!");
        NRF24_Receive(RxData, spi_device_handle);

        return 1;
    }
    else{
        ESP_LOGI(TAG, "NO MESSAGE :-(");
        return 0;
    }
    
    
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

    
    TxData[2] =  (uint8_t)(throttle & 0xFF);
    TxData[3] =  (uint8_t)(throttle >> 8);
    TxData[4] =  (uint8_t)(rudder & 0xFF); 
    TxData[5] =  (uint8_t)(rudder >> 8);
    TxData[27] =  (uint8_t)(ailerons & 0xFF);
    TxData[28] =  (uint8_t)(ailerons >> 8);
    TxData[8] =  (uint8_t)(elevator & 0xFF);
    TxData[9] =  (uint8_t)(elevator >> 8);
    TxData[10] = (uint8_t)(pot1 & 0xFF);
    TxData[11] = (uint8_t)(pot1 >> 8);
    TxData[12] = (uint8_t)(pot2 & 0xFF);
    TxData[13] = (uint8_t)(pot2 >> 8);
    TxData[14] = switch1Up;
    TxData[15] = switch1Down;
    TxData[16] = switch2Up;
    TxData[17] = switch2Down;

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


double CalculateAltitude(uint32_t pressure){
    return (1 - pow(pressure / (double)103490, 0.1903)) / 0.0000225577; //101325
}

float CalculateHeading(int16_t x, int16_t y){
    float heading = atan2((double)y,(double)x)*180.0/M_PI;
    heading = (heading < 0)? 360 + heading:heading;  

    return heading;
}

