#include <avr/io.h>
#include <stdio.h>

#include <util/setbaud.h>
#include <util/delay.h>

#include "GY_NEO6MV2.h"
#include "RX.h"

/* The data needed to from sentence GPGGA and GPVTG
   All data separated by ','

                 Latitude     Longitude     Fix   Sats    Alt
                 ____________ _____________ _     __      _______
$GPGGA,181614.00,5819.50633,N,01506.69262,E,1,    11,0.89,143.9,M,31.7,M,   ,*51
   1      2           3     4      5      6 7      8  9     10     11  12 13  checksum


       Bearing	        km/h
       __               _____  
$GPVTG,  ,T, ,M,0.166,N,0.308,K,A*29
   1   2  3 4 5   6   7   8   9 checksum


Example extracting latitude and longitude degrees and minutes 
4916.46,N    Latitude 49 deg. 16.45 min. North
12311.12,W   Longitude 123 deg. 11.12 min. West

Latitude and Longitude different formats:
Decimal degrees (DD): 41.40338, 2.17403
Degrees, minutes, and seconds (DMS): 41°24'12.2"N 2°10'26.5"E
Degrees and decimal minutes (DMM): 41 24.2028, 2 10.4418

*/


//volatile uint8_t sentenceReceived = 0;
extern uint8_t TxData[32];
volatile char receivedData;

volatile char gpsData[80];
char gpgga[] = {'$','G','P','G','G','A'};
char gpvtg[] = {'$','G','P','V','T','G'};
volatile uint8_t i = 0;

uint8_t fieldNumber = 0;


ISR(USART_RX_vect) {
    // Data received

    receivedData = UDR0;

    //gpsData[i] = (char)UDR0; funkar inte varför?

    gpsData[i] = receivedData;
    //TxData[15] = gpsData[i];

    i++; //used as pointer for gpsData array. resets to 0 if we are not reading from the start of either $GPGGA or $GPVTG sentences.

        if(i < 7)
        {
            if((gpsData[i-1] != gpgga[i-1]) && (gpsData[i-1] != gpvtg[i-1]))
            i = 0;
        }

        else if(i >= 15 && receivedData == '*'){ //the '*' is good to check that we have reached the end of a sentence, after the '*' there is only the checksum 

            char* token = strtok(gpsData, ","); //Take first token from the sentence, it will either be $GPGGA or $GPVTG, we check bellow:

            if (gpsData[4] == 'G') //GPGGA
            {   

                while (token != NULL)
                {
                    fieldNumber++;

                    if (fieldNumber == LATITUDE)
                    {   
                        TxData[GPS_LATITUDE_DEGREES] = (atoi(token) / 100);
                        TxData[GPS_LATITUDE_MIN] = (atoi(token) - (TxData[GPS_LATITUDE_DEGREES] * 100));
                        TxData[GPS_LATITUDE_SEC] = ((atof(token) - 5819) *100);
                    }

                    else if (fieldNumber == LATITUDE_DIR)
                    {   
                        TxData[GPS_LATITUDE_DIR] = token[0];
                    }

                    else if (fieldNumber == LONGITUDE)
                    {   
                        TxData[GPS_LONGITUDE_DEGREES] = (atoi(token) / 100);
                        TxData[GPS_LONGITUDE_MIN] = (atoi(token) - (TxData[GPS_LONGITUDE_DEGREES] * 100));
                        TxData[GPS_LONGITUDE_SEC] = ((atof(token) - 1506) *100);
                    }

                    else if (fieldNumber == LONGITUDE_DIR)
                    {   
                        TxData[GPS_LONGITUDE_DIR] = token[0];
                    }

                    else if (fieldNumber == FIX)
                    {   
                        TxData[GPS_FIX] = atoi(token);
                    }

                    else if (fieldNumber == SATELLITES_COUNT)
                    {   
                        TxData[GPS_SATELLITE_COUNT] = atoi(token);
                    }

                    else if (fieldNumber == ALTITUDE)
                    {   
                        uint16_t altitude = atoi(token);
                        TxData[GPS_ALTITUDE_LSB] = (altitude);
                        TxData[GPS_ALTITUDE_MSB] = (altitude >> 8);
                    }

                    token = strtok(NULL, ",");
                }
                fieldNumber = 0;
            }
            
            else if (gpsData[4] == 'T') //GPVTG
            {   
                while (token != NULL)
                {
                    fieldNumber++;
                    if (fieldNumber == GROUND_SPEED_KMPH)
                    {   
                        TxData[GPS_GROUND_SPEED_KMPH] = atoi(token);
                    }
                    token = strtok(NULL, ",");
                }
                fieldNumber = 0;
            }   
            
           i = 0;
        }
}


void uart_init(void) 
{   
    // Set baud rate to 9600
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;

    // Enable receiver and transmitter and also interrupt on RX pin
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);

    // Set frame format: 8 data bits, 1 stop bit, no parity
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}























