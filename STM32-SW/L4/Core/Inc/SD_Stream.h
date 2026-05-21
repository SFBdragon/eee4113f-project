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

#define FAKE_NUM_BLOCKS 1024

/* ── SD addressing ──────────────────────────────────────────── */
#define DATA_START_BLOCK 6000
#define DATA_START_SECTOR (DATA_START_BLOCK * (SD_BLOCK_SIZE / SD_SECTOR_SIZE))


extern uint32_t current_sector;


/* ── Public API ─────────────────────────────────────────────── */
uint32_t          SD_Stream_BlockCount       ();
void              SD_Stream_Init             (UART_HandleTypeDef *uartHandle);
HAL_StatusTypeDef SD_Stream_WriteBlock       (uint8_t *block);
void              SD_Stream_ReadDebug        (uint32_t start_sector, uint32_t num_blocks);
uint32_t          SD_Stream_GetCurrentSector (void);
void                SD_BruteSpeedTest       (void);
void SD_Stream_ReadDebug_Last_Line(uint32_t start_sector, uint32_t num_blocks);
void SD_MassAccuracyTest                    (uint32_t iterations);
void  SD_Stream_ReadAndForward(uint32_t start_sector, uint32_t num_blocks);
HAL_StatusTypeDef SD_Stream_ReadSectors(uint32_t sector, uint32_t count, uint8_t* buffer);


uint64_t SD_Stream_GetReadBase(void);
uint64_t SD_Stream_GetProtBase(void);
uint64_t SD_Stream_GetWriteHead(void);
void SD_Stream_SetProtHead(uint64_t from_block);
void SD_Stream_SetOverwritePolicy(uint32_t policy);

#endif