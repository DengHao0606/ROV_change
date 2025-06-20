#include "ws2812.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#define Code0  30
#define Code1  60
#define CodeReset 0

uint8_t color[LED_COUNT][3];

void WS2812_Set(uint8_t index, uint8_t r, uint8_t g, uint8_t b) 
{ 
	color[index][0] = r; 
	color[index][1] = g; 
	color[index][2] = b; 
}
void WS2812_SetAll(uint8_t r, uint8_t g, uint8_t b)
{ 
	for (uint8_t i = 0; i < LED_COUNT; i++) 
	{
		WS2812_Set(i, r, g, b) ;
	}
}
void WS2812_Update() 
{ 
	static uint16_t data[LED_COUNT * 3 * 8 + 1]; 
	for (int i = 0; i < LED_COUNT; i++)
	{ 
		uint8_t r = color[i][0]; 
		uint8_t g = color[i][1]; 
		uint8_t b = color[i][2]; 

		for (int j = 0; j < 8; j++) 
		{ 
			data[24 * i + j] = (g & (0x80 >> j)) ? Code1 : Code0;
			data[24 * i + 8 + j] = (r & (0x80 >> j)) ? Code1 : Code0;
			data[24 * i + 16 + j] = (b & (0x80 >> j)) ? Code1 : Code0; 
		}
	}
		
		data[LED_COUNT * 24] = CodeReset; 
		
		HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_1); 
		HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_3);
		HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_4); 

		__HAL_TIM_SetCounter(&htim3, 0);
		
		HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_1, (uint32_t*)data, sizeof(data)/sizeof(uint16_t));
		HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_3, (uint32_t*)data, sizeof(data)/sizeof(uint16_t));
		HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_4, (uint32_t*)data, sizeof(data)/sizeof(uint16_t));
}
//TIM3的通道2无DMA
void WS2812_SetFixedColor(uint8_t r, uint8_t g, uint8_t b) 
{
    // WS2812 数据格式：G-R-B（先发高位 MSB）
    uint32_t grb = (g << 16) | (r << 8) | b;

    // 只需要第一个 bit 是 1（60%占空比），后续 bit 可以不管（因为只点亮一个 LED）
    TIM3->CCR2 = 60;  // 第一个 bit=1（60%占空比）
    HAL_Delay(1);      // 保持一段时间（模拟 WS2812 数据）
    TIM3->CCR2 = 0;    // 复位信号（低电平）
    HAL_Delay(1);      // 保持至少 50µs
}
// LED颜色
// uint32_t ws2812_color[WS2812_NUM] = {0};

// // 当前LED颜色
// static uint32_t _ws2812_color_current[WS2812_NUM];

// /**
//  * @brief  直接更新LED颜色
//  */
// void ws2812_update(void)
// {
// 	// 数据缓冲，每个LED占用24个字节，共10个LED，前100个字节用于复位信号
// 	static uint16_t ws2812_data[RST_PERIOD_NUM + WS2812_NUM * 24];

// 	for (uint8_t led_id = 0; led_id < WS2812_NUM; led_id++)
// 	{
// 		_ws2812_color_current[led_id] = ws2812_color[led_id];
// 		static uint8_t r, g, b;
// 		color_to_rgb(_ws2812_color_current[led_id], &r, &g, &b);
// 		uint16_t *p = ws2812_data + RST_PERIOD_NUM + led_id * 24;
// 		for (uint8_t i = 0; i < 8; i++)
// 		{
// 			p[i] = (r << i) & (0x80) ? CODE_ONE_DUTY : CODE_ZERO_DUTY;
// 			p[i + 8] = (g << i) & (0x80) ? CODE_ONE_DUTY : CODE_ZERO_DUTY;
// 			p[i + 16] = (b << i) & (0x80) ? CODE_ONE_DUTY : CODE_ZERO_DUTY;
// 		}
// 	}
// 	HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_1);
// 	HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_3);
// 	HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_4);
// 	__HAL_TIM_SetCounter(&htim3, 0);
// 	HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_1, (uint32_t *)ws2812_data,
// 						  RST_PERIOD_NUM + WS2812_NUM * 24);	
// 	HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_3, (uint32_t *)ws2812_data,
// 						  RST_PERIOD_NUM + WS2812_NUM * 24);	
// 	HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_4, (uint32_t *)ws2812_data,
// 						  RST_PERIOD_NUM + WS2812_NUM * 24);
// }

// /**
//  * @brief  通过渐变方式更新LED颜色（线性插值）
//  * @param  steps: 渐变步数
//  * @param  delay_ms: 每步之间的延迟时间（毫秒）
//  */
// void ws2812_gradient(uint8_t steps, uint16_t delay_ms)
// {
// 	static uint8_t start_r[WS2812_NUM], start_g[WS2812_NUM], start_b[WS2812_NUM];
// 	static float r_step[WS2812_NUM], g_step[WS2812_NUM], b_step[WS2812_NUM];

// 	// 提取初始颜色，并计算每步的渐变步长
// 	for (uint8_t i = 0; i < WS2812_NUM; i++)
// 	{
// 		color_to_rgb(_ws2812_color_current[i], &start_r[i], &start_g[i], &start_b[i]);
// 		uint8_t target_r, target_g, target_b;
// 		color_to_rgb(ws2812_color[i], &target_r, &target_g, &target_b);

// 		r_step[i] = (float)(target_r - start_r[i]) / steps;
// 		g_step[i] = (float)(target_g - start_g[i]) / steps;
// 		b_step[i] = (float)(target_b - start_b[i]) / steps;
// 	}

// 	// 逐步渐变
// 	for (uint8_t step = 1; step <= steps; step++)
// 	{
// 		for (uint8_t led_id = 0; led_id < WS2812_NUM; led_id++)
// 		{
// 			// 计算当前步的颜色
// 			uint8_t r = (uint8_t)(start_r[led_id] + r_step[led_id] * step);
// 			uint8_t g = (uint8_t)(start_g[led_id] + g_step[led_id] * step);
// 			uint8_t b = (uint8_t)(start_b[led_id] + b_step[led_id] * step);

// 			ws2812_set_rgb(led_id, r, g, b);
// 		}

// 		ws2812_update();
// 		HAL_Delay(delay_ms);
// 	}
// }

// /**
//  * @brief  设置LED颜色(RGB格式)
//  * @param  led_id: LED编号（学习板一共有10个LED，编号范围0-9）
//  * @param  r: 红色亮度（0-255）
//  * @param  g: 绿色亮度（0-255）
//  * @param  b: 蓝色亮度（0-255）
//  */
// void ws2812_set_rgb(uint8_t led_id, uint8_t r, uint8_t g, uint8_t b)
// {
// 	ws2812_color[led_id] = rgb_to_color(r, g, b);
// }

// /**
//  * @brief  设置LED颜色（24bit颜色格式）
//  * @param  led_id: LED编号（学习板一共有10个LED，编号范围0-9）
//  * @param  color: 24bit颜色
//  */
// void ws2812_set(uint8_t led_id, uint32_t color)
// {
// 	ws2812_color[led_id] = color;
// }

// /**
//  * @brief  设置所有LED颜色（24bit颜色格式）
//  * @param  color: 24bit颜色
//  */
// void ws2812_set_all(uint32_t color)
// {
// 	for (uint8_t led_id = 0; led_id < WS2812_NUM; led_id++)
// 	{
// 		ws2812_color[led_id] = color;
// 	}
// }

// /**
//  * @brief  RGB转换为24bit颜色
//  * @param  r: 红色亮度（0-255）
//  * @param  g: 绿色亮度（0-255）
//  * @param  b: 蓝色亮度（0-255）
//  * @retval 24bit颜色
//  */
// uint32_t rgb_to_color(uint8_t r, uint8_t g, uint8_t b)
// {
// 	return (r << 16) | (g << 8) | b;
// }

// /**
//  * @brief  24bit颜色转换为RGB
//  * @param  color: 24bit颜色
//  * @param  r: 红色亮度（0-255）
//  * @param  g: 绿色亮度（0-255）
//  * @param  b: 蓝色亮度（0-255）
//  */
// void color_to_rgb(uint32_t color, uint8_t *r, uint8_t *g, uint8_t *b)
// {
// 	*r = (color >> 16) & 0xFF;
// 	*g = (color >> 8) & 0xFF;
// 	*b = color & 0xFF;
// }

// // =============== 以下为额外的效果演示函数 ================

// uint32_t rainbow_color(float frequency, int phase, int center, int width)
// {
// 	float r = sinf(frequency * phase + 0) * width + center;
// 	float g = sinf(frequency * phase + 2) * width + center;
// 	float b = sinf(frequency * phase + 4) * width + center;
// 	return rgb_to_color((uint8_t)r, (uint8_t)g, (uint8_t)b);
// }

// void rainbow_effect(uint8_t steps, uint16_t delay_ms)
// {
// 	float frequency = 0.1;
// 	int center = 128;
// 	int width = 127;

// 	for (int i = 0; i < steps; i++)
// 	{
// 		for (uint8_t led_id = 0; led_id < WS2812_NUM; led_id++)
// 		{
// 			uint32_t color = rainbow_color(frequency, i + led_id * 2, center, width);
// 			ws2812_set(led_id, color);
// 		}
// 		ws2812_update();
// 		HAL_Delay(delay_ms);
// 	}
// }


