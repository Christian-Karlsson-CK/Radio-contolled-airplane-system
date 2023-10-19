#ifndef __READBATTERYVOLTAGE_H
#define __READBATTERYVOLTAGE_H

#include <stdint.h>
#include "TxData.h"

#define REFERENCE_VOLTAGE      4.7
#define VOLTAGE_UPSCALE_FACTOR 3
#define BATTERY_MONITOR_PIN    0

void ReadBatteryVoltage(uint8_t *TxData);

#endif