#include "voltage.h"

/*
 * 函数名: Dev_Voltage_Read
 * 描述  : 采样电压
 * 输入  : ADC_HandleTypeDef hadc 指定ADC, uint32_t ch　指定ＡＤＣ通道
 * 输出  : uint16_t　电压
 * 备注  : 输出值０～４０９６对应０～３．３ｖ
 */
uint16_t Dev_Voltage_Read(ADC_HandleTypeDef hadc, uint32_t ch)
{
    ADC_ChannelConfTypeDef ADC_ChanConf;

    ADC_ChanConf.Channel      = ch;                       //通道
    ADC_ChanConf.Rank         = 1;                        //第1个序列，序列1
    ADC_ChanConf.SamplingTime = ADC_SAMPLETIME_480CYCLES; //采样时间
    ADC_ChanConf.Offset       = 0;
    HAL_ADC_ConfigChannel(&hadc, &ADC_ChanConf); //通道配置

    HAL_ADC_Start(&hadc); //开启ADC

    HAL_ADC_PollForConversion(&hadc, 10); //轮询转换

    return (uint16_t)HAL_ADC_GetValue(&hadc); //返回最近一次ADC1规则组的转换结果
}

/*
 * 函数名: Dev_Voltage_Refresh
 * 描述  : 采集电压信息并作转化
 * 输入  : ADC_HandleTypeDef hadc 指定ADC, uint32_t ch　指定ＡＤＣ通道, uint8_t times　采样次数
 * 输出  : 电压
 * 备注  : 输出值０～２５６对应０～１２ｖ
 */
float Dev_Voltage_Refresh(ADC_HandleTypeDef hadc, uint32_t ch, uint8_t times)
{
    uint32_t temp_val = 0;
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);

    for (int t = 0; t < times; t++)
    {
        temp_val += Dev_Voltage_Read(hadc, ch);
        HAL_Delay(5);
    }
    return 12.0 * ((float)temp_val / times) / 3083;
}
