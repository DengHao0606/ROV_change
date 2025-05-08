#include "json_process.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

RecBuf uart8rec = {0};

/* 私有变量 */
static uint8_t rx_buffer[512];
static uint16_t rx_index = 0;
static uint8_t rx_char;

/* 私有函数 */
static void parse_json_data(uint8_t *json_str);

/*
 * 函数名: JSON_Process_Init
 * 描述  : 初始化JSON处理器，启动串口接收中断，并初始化接收缓冲区
 * 输入  : 无
 * 输出  : 无
 * 备注  : 调用HAL_UART_Receive_IT函数启动串口接收中断，将接收缓冲区rx_buffer清零，并重置接收索引rx_index，
 *         同时打印初始化成功的提示信息。
 */
void JSON_Process_Init(void) 
{
    HAL_UART_Receive_IT(&huart8, &rx_char, 1);  // 启动接收中断
    memset(rx_buffer, 0, sizeof(rx_buffer));
    rx_index = 0;
    printf("JSON init success\r\n");
}

/*
 * 函数名: JSON_Process_Data
 * 描述  : 处理接收到的JSON数据，调用parse_json_data函数进行解析
 * 输入  : json_str - 指向接收到的JSON数据的指针
 * 输出  : 无
 * 备注  : 
 */
void JSON_Process_Data(uint8_t *json_str)
{
    parse_json_data(json_str);
}

/*
 * 函数名: HAL_UART_RxCpltCallback
 * 描述  : 串口中断处理
 * 输入  : UART_HandleTypeDef *huart 串口地址
 * 输出  : /
 * 备注  : /
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{
    // 串口接收控制指令
    if (huart == &huart8) 
    {
        if (uart8rec.buf[uart8rec.cnt - 1] == '{' && uart8rec.buf[uart8rec.cnt] == '\"' && uart8rec.cnt > 0)
        {
            uart8rec.cnt = 1;
            uart8rec.buf[0] = '{';
            uart8rec.buf[1] = '\"';
        }
        // 检查帧尾（换行符作为结束）
        else if (uart8rec.buf[uart8rec.cnt] == '\n' && uart8rec.cnt > 0)
        {
            uart8rec.buf[uart8rec.cnt] = '\0'; // 确保字符串终止
            JSON_Process_Data((uint8_t *)uart8rec.buf);
            uart8rec.cnt = 501; // 使缓冲计数归零
        }
        if (uart8rec.cnt >= 500)// 防止缓冲区溢出
            uart8rec.cnt = 0; 
        else
            uart8rec.cnt++;
        
        HAL_UART_Receive_IT(&huart8, uart8rec.buf + uart8rec.cnt, 1);
    }
}

/*
 * 函数名: parse_json_data
 * 描述  : 解析接收到的JSON数据，提取其中的字段并打印解析结果，调用时先在启动文件里修改Heap_Size
 * 输入  : json_str - 指向接收到的JSON数据的指针
 * 输出  : 无
 * 备注  :   
 */
static void parse_json_data(uint8_t *json_str)
{
  cJSON *root = cJSON_Parse((char *)json_str);
  if (!root) 
  {
      const char *error_ptr = cJSON_GetErrorPtr();
      if (error_ptr != NULL) 
      {
          printf("Error before: %s\n", error_ptr);
      }
      printf("Error\r\n");
      return;
  }

  /* 提取各字段 */
  JSON_Command_t command = {0};

  cJSON *x_item = cJSON_GetObjectItem(root, "x");
    if  (x_item) command.x = x_item->valuedouble;

  cJSON *y_item = cJSON_GetObjectItem(root, "y");
    if  (y_item) command.y = y_item->valuedouble;

  cJSON *z_item = cJSON_GetObjectItem(root, "z");
    if  (z_item) command.z = z_item->valuedouble;

  cJSON *yaw_item = cJSON_GetObjectItem(root, "yaw");
    if  (yaw_item) command.yaw = yaw_item->valuedouble;
    
  cJSON *servo0_item = cJSON_GetObjectItem(root, "servo0");
    if  (servo0_item) command.servo0 = servo0_item->valuedouble;

  /* 不需要解析 */
  // cJSON *roll_item = cJSON_GetObjectItem(root, "roll");
  //   if  (roll_item) command.roll = roll_item->valuedouble;

  // cJSON *pitch_item = cJSON_GetObjectItem(root, "pitch");
  //   if  (pitch_item) command.pitch = pitch_item->valuedouble;

  // cJSON *servo1_item = cJSON_GetObjectItem(root, "servo1");
  //   if  (servo1_item) command.servo1 = servo1_item->valuedouble;

  // cJSON *state_item = cJSON_GetObjectItem(root, "state");
  //   if  (state_item) command.state = state_item->valueint;

  /* 打印解析结果 */
  printf("x: %.2f  ",command.x);
  printf("y: %.2f  ",command.y);
  printf("z: %.2f  ",command.z);
  printf("yaw: %.2f  ",command.yaw);
  printf("servo0: %.3f\r\n",command.servo0);
  cJSON_Delete(root);
}