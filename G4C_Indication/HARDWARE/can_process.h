#ifndef __CAN_PROCESS_H
#define __CAN_PROCESS_H

#include "fdcan.h"

void FDCAN1_Config(void);
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
void Canfd1Transmit64(uint8_t id, uint8_t *can1_txbuf);

#endif
