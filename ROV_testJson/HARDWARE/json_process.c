#include "json_process.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

/* 私有变量 */
static uint8_t rx_buffer[256];
static uint16_t rx_index = 0;
static uint8_t rx_char;

/* 私有函数 */
static void parse_json_data(uint8_t *json_str);

/* 初始化JSON处理器 */
void JSON_Process_Init(void) 
{
    HAL_UART_Receive_IT(&huart5, &rx_char, 1);  // 启动接收中断
    memset(rx_buffer, 0, sizeof(rx_buffer));
    rx_index = 0;
    printf("JSON处理器初始化完成\r\n");
}

/* 处理JSON数据 */
void JSON_Process_Data(uint8_t *json_str)
{
    // printf("开始处理JSON数据...\r\n");
    parse_json_data(json_str);
}

/* 串口接收回调函数 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{
    if (huart->Instance == UART5)
    {
        if (rx_char == '\n') 
        {  // 检测到换行符
        rx_buffer[rx_index] = '\0';
        printf("Full JSON: %s\r\n", rx_buffer);
        JSON_Process_Data(rx_buffer);
        rx_index = 0;
        } 
        else 
        {
            if (rx_index < sizeof(rx_buffer) - 1) 
            {
                rx_buffer[rx_index++] = rx_char;
            }
            else
            {
                printf("错误：接收缓冲区已满!\r\n");
                rx_index = 0;
            }
        }
    HAL_UART_Receive_IT(&huart5, &rx_char, 1);
    }
}

/* 解析JSON数据 */
static void parse_json_data(uint8_t *json_str)
{
  cJSON *root = cJSON_Parse((char *)json_str);
    if (!root) 
    {
        printf("Error\r\n");
        return;
    }

  /* 提取各字段 */
    JSON_Command_t command = {0};

  cJSON *cmd_item = cJSON_GetObjectItem(root, "cmd");
    if (cmd_item) strncpy(command.cmd, cmd_item->valuestring, sizeof(command.cmd)-1);

  cJSON *value_item = cJSON_GetObjectItem(root, "value");
    if (value_item) command.value = value_item->valueint;

  cJSON *name_item = cJSON_GetObjectItem(root, "name");
    if (name_item) strncpy(command.name, name_item->valuestring, sizeof(command.name)-1);

  cJSON *number_item = cJSON_GetObjectItem(root, "number");
    if (number_item) command.number = number_item->valueint;

  /* 打印解析结果 */
printf("cmd: %s\r\n", command.cmd);
printf("value: %d\r\n", command.value);
printf("name: %s\r\n", command.name);
printf("number: %d\r\n", command.number);

  /* 执行命令 */
if (strcmp(command.cmd, "led") == 0) 
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 
                    command.value ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

cJSON_Delete(root);
}