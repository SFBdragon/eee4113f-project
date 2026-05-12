#ifndef SD_IO_H
#define SD_IO_H

#include "stm32l4xx_hal.h"  // adjust to your MCU family if different
#include <stdint.h>

/* ── Public API ─────────────────────────────────────────────── */
void             SD_IO_Init(UART_HandleTypeDef *uartHandle);
HAL_StatusTypeDef SD_WriteUptime(uint32_t uptime);
uint32_t          SD_ReadUptime(void);
 

#endif /* SD_IO_H */