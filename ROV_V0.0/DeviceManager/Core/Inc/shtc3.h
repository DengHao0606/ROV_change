#ifndef __SHTC3_H
#define __SHTC3_H

#include "i2c.h"

struct SHTC3_MeasureData
{
    uint8_t HumidityMSB;
    uint8_t HumidityLSB;
    uint8_t HumidityCRC;
    uint8_t temperatureMSB;
    uint8_t temperatureLSB;
    uint8_t temperatureCRC;
};

struct SHTC31000
{
    uint8_t temp;
    uint8_t hum;
};

extern struct SHTC3_MeasureData shtc3Read;
extern struct SHTC31000 SHTC3_Data;

void Dev_Shtc3_init(); // void SHTC3_GoSleep();//休眠
void Dev_Shtc3_Read();
void Dev_Shtc3_Refresh(); //实时刷新当前温度湿度

#endif