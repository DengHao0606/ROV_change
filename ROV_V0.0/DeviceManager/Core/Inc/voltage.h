#ifndef __VOLTAGE_H
#define __VOLTAGE_H

#include "adc.h"

uint16_t Dev_Voltage_Read(ADC_HandleTypeDef hadc,uint32_t ch);
float Dev_Voltage_Refresh(ADC_HandleTypeDef hadc,uint32_t ch,uint8_t times);

#endif