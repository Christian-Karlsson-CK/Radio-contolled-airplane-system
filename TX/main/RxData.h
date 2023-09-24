#ifndef _RXDATA_H_
#define _RXDATA_H_

//RxData[0] and RxData[1] is used by the NRF24L01

//ATMEGA328P           RxData[X]
#define BAT_VOLTAGE_WHOLE     3   //Airplane battery voltage
#define BAT_VOLTAGE_DECIMAL   4

//GY_271
#define MAGNETIC_X_LSB        5   //Magnetic field intensity or magnetic flux density along the X-axis
#define MAGNETIC_X_MSB        6    
#define MAGNETIC_Y_LSB        7   //Magnetic field intensity or magnetic flux density along the Y-axis
#define MAGNETIC_Y_MSB        8

//BMP280
#define PRESSURE_LSB_0        9   //4 byte airpressure in Pascal
#define PRESSURE_MID_8        10
#define PRESSURE_MID_16       11   
#define PRESSURE_MSB_24       12

#define TEMPERTURE_LSB_0        26   //4 byte temperature in degree celsius
#define TEMPERTURE_MID_8        27
#define TEMPERTURE_MID_16       28   
#define TEMPERTURE_MSB_24       29

//GY_NEO6MV2
#define GPS_FIX               13  //GPS Data
#define GPS_SATELLITE_COUNT   14
#define GPS_ALTITUDE_LSB      15
#define GPS_ALTITUDE_MSB      16
#define GPS_GROUND_SPEED_KMPH 17
#define GPS_LATITUDE_DEGREES  18
#define GPS_LATITUDE_MIN      19
#define GPS_LATITUDE_SEC      20
#define GPS_LATITUDE_DIR      21
#define GPS_LONGITUDE_DEGREES 22
#define GPS_LONGITUDE_MIN     23
#define GPS_LONGITUDE_SEC     24
#define GPS_LONGITUDE_DIR     25


#endif /* _RXDATA_H_ */