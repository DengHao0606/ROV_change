#include "shtc3.h"

#define SHTC3_Address      0x07
#define SHTC3_Aaddress_W   0xE0
#define SHTC3_Aaddress_R   0xE1
#define SHTC3_WakeUp       0X3517
#define SHTC3_Sleep        0xB098
#define SHTC3_MeasuringCmd 0x5C24

#pragma pack(1)

struct SHTC3_MeasureData shtc3Read;
struct SHTC31000 SHTC3_Data;

/*
 * 函数名: Dev_Shtc3_init
 * 描述  : 温湿度计初始化
 * 输入  : /
 * 输出  : /
 * 备注  : /
 */
void Dev_Shtc3_init(void)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);
    uint8_t cmdTemp[2];

    cmdTemp[0] = (uint8_t)(SHTC3_WakeUp >> 8);
    cmdTemp[1] = (uint8_t)(SHTC3_WakeUp);
    HAL_I2C_Master_Transmit(&hi2c1, SHTC3_Aaddress_W, (uint8_t *)&cmdTemp, sizeof(cmdTemp), 1000);
    HAL_Delay(10);
    cmdTemp[0] = (uint8_t)(SHTC3_MeasuringCmd >> 8);
    cmdTemp[1] = (uint8_t)(SHTC3_MeasuringCmd);
    HAL_I2C_Master_Transmit(&hi2c1, SHTC3_Aaddress_W, (uint8_t *)&cmdTemp, sizeof(cmdTemp), 1000);
    HAL_Delay(10);
}

// void SHTC3_GoSleep()
//{
//	uint8_t cmdTemp[2];
//
//	cmdTemp[0] = (uint8_t)(SHTC3_Sleep>>8);
//	cmdTemp[1] = (uint8_t)(SHTC3_Sleep);
//	HAL_I2C_Master_Transmit(&hi2c1, SHTC3_Aaddress_W, (uint8_t *)&cmdTemp, sizeof(cmdTemp), 1000);
//}

/*
 * 函数名: Dev_Shtc3_Read
 * 描述  : 温湿度计读取
 * 输入  : /
 * 输出  : /
 * 备注  : /
 */
void Dev_Shtc3_Read()
{
    float hum, temp;

    HAL_I2C_Master_Receive(&hi2c1, SHTC3_Aaddress_R, (uint8_t *)&shtc3Read, sizeof(shtc3Read), 1000);

    hum             = (shtc3Read.HumidityMSB << 8) | (shtc3Read.HumidityLSB);
    temp            = (shtc3Read.temperatureMSB << 8) | shtc3Read.temperatureLSB;
    SHTC3_Data.hum  = (uint8_t)(100 * ((float)hum / 65535));         //湿度
    SHTC3_Data.temp = (uint8_t)((175 * ((float)temp / 65535)) - 45); //温度
    //
    //printf(" SHTC3_Data.hum is %d",SHTC3_Data.hum);
    //printf(" SHTC3_Data.temp is %d",SHTC3_Data.temp);
}

/*
 * 函数名: Dev_Shtc3_Refresh
 * 描述  : 温湿度计数据刷新,并存入全局变量中
 * 输入  : /
 * 输出  : /
 * 备注  : /
 */
void Dev_Shtc3_Refresh(void)
{
    Dev_Shtc3_init();
    Dev_Shtc3_Read();
}
