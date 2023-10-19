#include "ReadBatteryVoltage.h"
#include "analogRead.h"

void ReadBatteryVoltage(uint8_t *TxData){

    float voltage = (analogRead(BATTERY_MONITOR_PIN) / 1023.0) 
                    * REFERENCE_VOLTAGE * VOLTAGE_UPSCALE_FACTOR;

    TxData[BAT_VOLTAGE_WHOLE]   = voltage; //integer number (Heltal)
    TxData[BAT_VOLTAGE_DECIMAL] = (voltage - TxData[BAT_VOLTAGE_WHOLE]) * 10; //Decimal number
}