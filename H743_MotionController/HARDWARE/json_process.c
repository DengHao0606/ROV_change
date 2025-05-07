#include "json_process.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

RecBuf uart8rec = {0};

/* 私有变量 */
static uint8_t rx_buffer[512];
static uint16_t rx_index = 0;
static uint8_t rx_char;

/* 私有函数 */
static void parse_json_data(uint8_t *json_str);

/* 初始化JSON处理器 */
void JSON_Process_Init(void) 
{
    HAL_UART_Receive_IT(&huart8, &rx_char, 1);  // 启动接收中断
    memset(rx_buffer, 0, sizeof(rx_buffer));
    rx_index = 0;
    printf("JSON init success\r\n");
}

/* 处理JSON数据 */
void JSON_Process_Data(uint8_t *json_str)
{
    printf("start process...\r\n");
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
        //uint8_t current_byte = uart8rec.buf[uart8rec.cnt];
        
        // 检查帧头（例如"{"作为JSON开始）
        // if ((prev_byte_uart8 == '{') && (current_byte == '\"') && (uart8rec.cnt > 0))
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
        
        // prev_byte_uart8 = current_byte;
        HAL_UART_Receive_IT(&huart8, uart8rec.buf + uart8rec.cnt, 1);
    }
}

/* 解析JSON数据 */
static void parse_json_data(uint8_t *json_str)
{
  printf("JSON string: %s\r\n", json_str); // 打印传入的 JSON 字符串
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
    if (x_item) command.x = x_item->valuedouble;

  cJSON *y_item = cJSON_GetObjectItem(root, "y");
    if (y_item) command.y = y_item->valuedouble;

  cJSON *z_item = cJSON_GetObjectItem(root, "z");
    if (z_item) command.z = z_item->valuedouble;

  /* 打印解析结果 */
printf("x: %f ,y: %f, z: %f ,\r\n", command.x, command.y, command.z);
printf("1111\r\n");
//   /* 执行命令 */
// if (strcmp(command.cmd, "led") == 0) 
// {
//     HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_6);
// }

cJSON_Delete(root);
}