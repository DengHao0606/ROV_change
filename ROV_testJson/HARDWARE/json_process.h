#ifndef __JSON_PROCESS_H
#define __JSON_PROCESS_H

#include "cJSON.h"
#include "main.h"

/* 函数声明 */
void JSON_Process_Init(void);
void JSON_Process_Data(uint8_t *json_str);
void JSON_UART_Callback(UART_HandleTypeDef *huart);

/* 命令数据结构体 */
typedef struct {
  char cmd[16];
  int value;
  char name[32];
  int number;
  float ps2;
} JSON_Command_t;

#endif /* __JSON_PROCESS_H */