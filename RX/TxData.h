#ifndef _TXDATA_H_
#define _TXDATA_H_

//TxData[0] and TxData[1] is used by the NRF24L01

//ATMEGA328P           TxData[X]
#define BAT_VOLTAGE_WHOLE     2   //Airplane battery voltage
#define BAT_VOLTAGE_DECIMAL   3

//GY_271
#define MAGNETIC_X_LSB        4   //Magnetic field intensity or magnetic flux density along the X-axis
#define MAGNETIC_X_MSB        5    
#define MAGNETIC_Y_LSB        6   //Magnetic field intensity or magnetic flux density along the Y-axis
#define MAGNETIC_Y_MSB        7

//BMP280
#define PRESSURE_LSB_0        8   //4 byte airpressure in Pascal
#define PRESSURE_MID_8        9
#define PRESSURE_MID_16       10   
#define PRESSURE_MSB_24       11

//GY_NEO6MV2
#define GPS_FIX               12  //GPS Data
#define GPS_SATELLITE_COUNT   13
#define GPS_ALTITUDE_LSB      14
#define GPS_ALTITUDE_MSB      15
#define GPS_GROUND_SPEED_KMPH 16
#define GPS_LATITUDE_DEGREES  17
#define GPS_LATITUDE_MIN      18
#define GPS_LATITUDE_SEC      19
#define GPS_LATITUDE_DIR      20
#define GPS_LONGITUDE_DEGREES 21
#define GPS_LONGITUDE_MIN     22
#define GPS_LONGITUDE_SEC     23
#define GPS_LONGITUDE_DIR     24


#endif /* _TXDATA_H_ */