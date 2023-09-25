#ifndef _RX_H_
#define _RX_H_

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "uart.h"
#include "analogRead.h"
#include "millis.h"
#include "servo.h"

#include "NRF24L01.h"
#include "UnoR3Pins.h"
#include "lcd.h"
#include "BMP280.h"
#include "GY_271.h"
#include "TxData.h"
#include "RxData.h"
//#include "GY_NEO6MV2.h"

#define SWITCH_TO_TX_COMMAND   1

#define COMMAND_BYTE           6

#define REFERENCE_VOLTAGE      4.7
#define VOLTAGE_UPSCALE_FACTOR 3

#define BIT_SET(a, b) ((a) |= (1ULL << (b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1ULL<<(b)))
#define BIT_FLIP(a,b) ((a) ^= (1ULL<<(b)))
#define BIT_CHECK(a,b) (!!((a) & (1ULL<<(b))))

void init_RX();

void ConvertToPercentage(double* reading);

uint8_t ReceiveData(uint8_t *RxData);
void ActOnReceivedData(uint8_t *RxData);

void TransmitData(uint8_t *TxData);

ReadBatteryVoltage(uint8_t *TxData);

void setHomeLocationInDecimalDegrees();
void updateCurrentLocationInDecimalDegrees();
void returnToHome();

double getBearingToHome();
double calculateHeading();

#endif /* _RX_H_ */