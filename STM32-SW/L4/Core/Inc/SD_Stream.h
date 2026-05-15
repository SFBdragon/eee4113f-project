#ifndef SD_STREAM_H
#define SD_STREAM_H

#include "stm32l4xx_hal.h"

/* ── Buffer / Block geometry ────────────────────────────────── */
#define RX_BUF_SIZE       (12 * 1024)
#define SD_SECTOR_SIZE    512
#define SD_BLOCK_SIZE     1024
#define BLOCK_HEADER_SIZE 4
#define BLOCK_DATA_SIZE   (SD_BLOCK_SIZE - BLOCK_HEADER_SIZE)  // 1020
#define NUM_BLOCKS        ((RX_BUF_SIZE / 2) / BLOCK_DATA_SIZE)
#define REMAINDER_BYTES   ((RX_BUF_SIZE / 2) % BLOCK_DATA_SIZE)

/* ── SD addressing ──────────────────────────────────────────── */
#define DATA_START_SECTOR 5000

/* ── Public API ─────────────────────────────────────────────── */
void              SD_Stream_Init             (UART_HandleTypeDef *uartHandle);
HAL_StatusTypeDef SD_Stream_WriteBlock       (uint8_t *block);
void              SD_Stream_ReadDebug        (uint32_t start_sector, uint32_t num_blocks);
uint32_t          SD_Stream_GetCurrentSector (void);
void                SD_BruteSpeedTest       (void);
void SD_MassAccuracyTest                    (uint32_t iterations);

#endif