
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

void app_main(void)
{   
    InitControls();
    lcd_init();

    spi_device_handle_t spi_device_handle; //SPI handle for the SPI communication to NRF24L01+
    SPI_init(&spi_device_handle);

    uint8_t TxData[32];
    uint8_t RxData[32];

    NRF24_Init(&spi_device_handle);

    //NRF24_TXMode(&spi_device_handle);

    NRF24_RXMode(&spi_device_handle);
    
    uint8_t TX_RX_Switch_counter = 0; //This counter is used to switch between RX and TX mode. TX mode sends all analog stick and potentiometer values.
                                      //RX mode is used to receive status data from the RX on the airplane, for instance Battery voltage. 

    while (true)
    {   
        /*************************************************************************************************/
        //GPS TEST:
        /*
        if(!ReceiveData(&spi_device_handle, RxData))
            {   
                ESP_LOGI(TAG, "NO MessageXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            else
            {
                ESP_LOGI(TAG, "Message Received!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

                uint8_t whole = RxData[3];
                uint8_t decimal = RxData[4];

                uint8_t gps1 = RxData[5];
                uint8_t gps2 = RxData[6];

                lcd_set_cursor(0,0);
                lcd_printf("%.2u.%.2uV", whole, decimal);

                lcd_set_cursor(0,1);
                lcd_printf("%c, %c", gps1, gps2);
                
                vTaskDelay(pdMS_TO_TICKS(10));

            }
            */

        /*************************************************************************************************/

                /*************************************************************************************************/
        //BMP280 TEST:
        if(!ReceiveData(&spi_device_handle, RxData))
            {   
                ESP_LOGI(TAG, "NO MessageXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            else
            {
                ESP_LOGI(TAG, "Message Received!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

                uint8_t whole = RxData[3];
                uint8_t decimal = RxData[4];

                //uint8_t bmp280 = RxData[8];
                //uint16_t bmp280 = (uint16_t)((RxData[9] << 8) | RxData[8]);
                uint8_t BxData[4] = {0x00, 0x00, 0xFD, 0xA0};
                uint32_t bmp280 = 0;
                    bmp280 |= (((uint32_t)RxData[11]) << 24);
                    bmp280 |= (((uint32_t)RxData[10]) << 16);
                    bmp280 |= (((uint32_t)RxData[9]) << 8);
                    bmp280 |= (((uint32_t)RxData[8]) << 0);
                    

                    double altitude;
                    altitude = (1 - pow(bmp280 / (double)103490, 0.1903)) / 0.0000225577; //101325
                    //double alt = 0;
                    //alt |= (((uint8_t)RxData[11] & 0xFF) << 56);
                    //alt |= (((uint8_t)RxData[11] & 0xFF) << 48);
                    //alt |= (((uint8_t)RxData[11] & 0xFF) << 40);
                    //alt |= (((uint8_t)RxData[11] & 0xFF) << 32);

                    //alt |= (((uint8_t)RxData[12] & 0xFF) << 24);
                    //alt |= (((uint8_t)RxData[13] & 0xFF) << 16);
                    //alt |= (((uint8_t)RxData[9] & 0xFF) << 8);
                    //alt |= (((uint8_t)RxData[8] & 0xFF) << 0);

                    //bmp280 |= ((int32_t)BxData[0] & 0xFF);
                    //bmp280 |= (((int32_t)BxData[1] & 0xFF) << 8);
                    //bmp280 |= (((int32_t)BxData[1] & 0xFF) << 16);
                    //bmp280 |= (((int32_t)BxData[0] & 0xFF) << 24);
                //uint16_t bmp280 = 65535;
                
                
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

                lcd_set_cursor(0,1);
                lcd_printf("ALT:%.2lfM", altitude);
                
                vTaskDelay(pdMS_TO_TICKS(100));
                

            }

        /*************************************************************************************************/

        /*REGULAR CODE*************************************************************************************/
        /*
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
                ESP_LOGI(TAG, "NO MessageXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
                vTaskDelay(pdMS_TO_TICKS(0.7));
                tries++;
            }
            if (tries <= 5)
            {
                ESP_LOGI(TAG, "Message Received!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

                uint8_t whole = RxData[3];
                uint8_t decimal = RxData[4];

                lcd_set_cursor(0,0);
                if ((whole > 99) | (decimal > 99))
                {
                    lcd_printf("00.0V");
                }
                else{
                    decimal /= 10;
                    lcd_printf("%.2u.%.1uV", whole, decimal);
                }
                
                
            }
            
            tries = 0;
            TX_RX_Switch_counter = 0;
            NRF24_TXMode(&spi_device_handle);
        }
        */
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

    TxData[18] =  (uint8_t)(rudder & 0xFF); 
    TxData[19] =  (uint8_t)(rudder >> 8);
    TxData[2] =  (uint8_t)(throttle & 0xFF);
    TxData[3] =  (uint8_t)(throttle >> 8);
    TxData[6] =  (uint8_t)(ailerons & 0xFF);
    TxData[7] =  (uint8_t)(ailerons >> 8);
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
    uint16_t rudderT = (uint16_t)((TxData[3] << 8) | TxData[2]);
    uint16_t throttleT = (uint16_t)((TxData[5] << 8) | TxData[4]);
    uint16_t aileronsT = (uint16_t)((TxData[7] << 8) | TxData[6]);
    uint16_t elevatorT = (uint16_t)((TxData[9] << 8) | TxData[8]);
    uint16_t pot1T = (uint16_t)((TxData[11] << 8) | TxData[10]);
    uint16_t pot2T = (uint16_t)((TxData[13] << 8) | TxData[12]);
    
    ESP_LOGI(TAG, "rudder: %u, throttle: %u", rudderT, throttleT);
    ESP_LOGI(TAG, "ailerons: %u, elevator: %u", aileronsT, elevatorT);
    ESP_LOGI(TAG, "Pot1: %u, Pot2: %u", pot1T, pot2T);
    ESP_LOGI(TAG, "sw1Up: %u, sw1Down: %u", TxData[14], TxData[15]);
    ESP_LOGI(TAG, "sw2Up: %u, sw2Down: %u", TxData[16], TxData[17]);
    */

    //vTaskDelay(pdMS_TO_TICKS(10));
    
    
}
