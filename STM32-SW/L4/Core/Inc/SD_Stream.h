#ifndef SD_STREAM_H
#define SD_STREAM_H

#include "stm32l4xx_hal.h"


#define RX_BUF_SIZE       (12*1024)      // Size of DMA buffer in physical SRAM

#define DATA_START_SECTOR 2048   

#define SD_BURST_SIZE     (RX_BUF_SIZE / 2) 

#define SD_BURST_BLOCKS   (SD_BURST_SIZE / 512)  
#define SD_BLOCK_SIZE     512


/* Public API */
void SD_Stream_Init(UART_HandleTypeDef *uartHandle);
void SD_Stream_WriteHalf(uint8_t *src, uint16_t len);
void SD_Stream_WriteSecondHalf(uint8_t *src, uint16_t len);
void SD_Stream_Flush(void);
void SD_Stream_ReadDebug(uint32_t start_sector, uint32_t num_blocks);
uint32_t SD_Stream_GetCurrentSector(void); 
#endif