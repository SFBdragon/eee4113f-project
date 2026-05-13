/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

// OLED Display
#include "ssd1306.h"
#include "ssd1306_fonts.h"

#include <string.h>
#include <stdio.h>

#include "SD_IO.h"
#include "SD_Stream.h"

#include "control.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

uint8_t ring_buffer[RX_BUF_SIZE];
uint8_t dma_transfer_buffer[SD_SECTOR_SIZE] __attribute__((aligned(32))); 

volatile uint32_t head = 0; // Written by UART ISR
volatile uint32_t tail = 0; // Read by Main Loop

volatile uint8_t wake_up_flag = 0;

static uint8_t writeBuf[512];
static uint8_t readBuf[512];//

// lora

uint8_t  lora_rx_buf[250];
uint16_t lora_rx_len   = 0;
volatile uint8_t lora_rx_ready = 0; 

uint8_t lora_tx_buf[250];
uint16_t lora_tx_len   = 0;

// For packing blocks
static uint8_t packedBlocks[NUM_BLOCKS][SD_BLOCK_SIZE];



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// all for for testing
#define LARGE_TEST_TOTAL_SIZE (16 * 1024 * 1024) // 16 MB
#define LARGE_TEST_BURST_SIZE (16 * 1024)        // 16 KB bursts (32 blocks)
#define LARGE_TEST_BLOCKS     (LARGE_TEST_BURST_SIZE / 512)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;

RTC_HandleTypeDef hrtc;

SD_HandleTypeDef hsd1;
DMA_HandleTypeDef hdma_sdmmc1_rx;
DMA_HandleTypeDef hdma_sdmmc1_tx;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

// STATE FLAGS

uint8_t WIFI_MODE = 1;    // Indicates if WIFI is to be active or not.
uint8_t WIFI_MODE_SETUP = 0;

uint32_t total_uptime; // Total time SD card has been active, in seconds
const char total_uptime_filename[] = "UPTIME.txt";
uint8_t rxBuf[512];

volatile uint8_t wake_source = 0;
volatile uint8_t halfTransferFired = 0;

// USART From SHarc BUOY data
volatile uint8_t cb_write_first_half  = 0;  
volatile uint8_t cb_write_second_half = 0;
volatile uint8_t uart_active          = 0;   /* set by EXTI, cleared on done */
extern UART_HandleTypeDef huart1;


// SHARC BUOY DATA WRITINGS
uint8_t rxBuffer[RX_BUF_SIZE];
volatile uint16_t rxLen = 0;
volatile uint8_t dataReady = 0;

// For cheeky packet spacing
static uint8_t accumBuffer[RX_BUF_SIZE * 4];   // size to taste
static uint16_t accumHead = 0;
static uint16_t lastSize = 0;

static uint8_t largeTestBuf[LARGE_TEST_BURST_SIZE] __attribute__((aligned(4)));

// WIFI
// 1. The Buffer: Adjust the size (64, 128, 256) based on your largest expected packet
#define WIFI_BUFFER_SIZE 128
volatile uint8_t wifi_rx_buf[WIFI_BUFFER_SIZE];

// 2. The Length: Stores how many bytes actually arrived in the last packet
volatile uint16_t wifi_rx_len = 0;

// 3. The Flag: A simple '0' or '1' so your main loop knows new data is ready
volatile uint8_t wifi_rx_ready = 0;


// DATA BLOCK NUMBER , PERSIST ACROSS HALF-BUFFER CALLBACKS
static uint16_t g_block_num = 0;   // persists across half-buffer callbacks

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_RTC_Init(void);
static void MX_I2C2_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void UART_SendString(UART_HandleTypeDef *huart, char *str);
static uint32_t get_dma_head(void);
static void Print_SD_Details(void);
static void SD_RawWriteTest(void);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void SD_SpeedTest(void);
void UART_DumpBuffer(uint8_t *data, uint32_t len);

void lora_send(uint8_t *pData, uint16_t size);

void PackBlocks(const uint8_t *src, uint16_t startBlockNum) ;
uint16_t CRC16(const uint8_t *data, uint16_t length);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

  #include <stdio.h>

  // SETUP FOR SERIAL COMS TO LAPTOP
  #ifdef __GNUC__
    #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
  #else
    #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
  #endif

  PUTCHAR_PROTOTYPE
  {
    HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
  }


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  // BEFORE SDMMC FAILS


  // LPUART CLOCK OVERRIDE TO KEEP HSI ON IN STOP MODE (NB THIS IS ALSO DONE IN MX_LPUART1_UART_Init BUT WE NEED IT EARLY TO KEEP HSI ALIVE)
      // Set LPUART1 clock to HSI16 and keep HSI alive in Stop mode
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART1;
    PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_HSI;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
    // Replace __HAL_RCC_HSIKERON_ENABLE() with:
    SET_BIT(RCC->CR, RCC_CR_HSIKERON);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_LPUART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  MX_RTC_Init();
  MX_I2C2_Init();
  MX_SDMMC1_SD_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

// SHARC BUOR RX -> DMA setup
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rxBuffer, RX_BUF_SIZE);

  // LPUART Trigger
    HAL_UARTEx_ReceiveToIdle_IT(&hlpuart1, lora_rx_buf, sizeof(lora_rx_buf));


  
  // This allows the debugger to keep communicating even if the CPU enters SLEEP, STOP, or STANDBY
  HAL_DBGMCU_EnableDBGSleepMode();
  HAL_DBGMCU_EnableDBGStopMode();
  HAL_DBGMCU_EnableDBGStandbyMode();

  //fHAL_GPIO_WritePin(Lo_PWR_CTRL_GPIO_Port, Lo_PWR_CTRL_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(Wi_EN_GPIO_Port, Wi_EN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(Wi_GPIO_0_GPIO_Port, Wi_GPIO_0_Pin, GPIO_PIN_SET);

  UART_SendString(&huart3, "SETUP\r\n");

  // Start UART RX in DMA Circular mode
  // The DMA hardware will automatically wrap around to the start of rXuffer
  //SD_RawWriteTest();    // NB KEEP THIS COMMENTED OUT worked BECAUSE IT DESTROYS SECTOR 100 BOOT
  Print_SD_Details();
  SD_IO_Init(&huart3); 
  SD_RawWriteTest();

  total_uptime = SD_ReadUptime();
  SD_Stream_Init(&huart3);
  HAL_Delay(500);

    // SPEED TEST
  //SD_SpeedTest();
  //HAL_GPIO_WritePin(Wi_GPIO_0_GPIO_Port, Wi_GPIO_0_Pin, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int i = 0;

  while (i < 10000000)
  { 
    /*
    // WIFI TESST RECIEVE:
    uint8_t wi_rxBuffer[20]; 
    HAL_StatusTypeDef rxStatus = HAL_UART_Receive(&huart2, wi_rxBuffer, 10, 1000);
      if (rxStatus == HAL_OK) {
      UART_SendString(&huart3, "WIFI DATA RECEIVED:\r\n"); 
      
      char hex[8];
      for (uint16_t i = 0; i < 10; i++) // 10 is the number of bytes we asked for
      {
          snprintf(hex, sizeof(hex), "%02X ", wi_rxBuffer[i]);
          UART_SendString(&huart3, hex);
      }
      UART_SendString(&huart3, "\r\n");
  } 
  else if (rxStatus == HAL_TIMEOUT) {
      UART_SendString(&huart3, "WIFI RX TIMEOUT (No data received)\r\n");
  }
\

*\
  
  

  
  // example of lora send usage:
  uint8_t lora_tx_buf[] = {'G', 'Ge', 'G'};
  lora_send(lora_tx_buf, sizeof(lora_tx_buf));
    */

    i++;
      // ================   WIFI MANAGEMENT =======================

      if (WIFI_MODE){
        // Wifi Active
        // RE-assert Power to power
        if(WIFI_MODE_SETUP == 0){
          // setup wifi
          UART_SendString(&huart3, "STARTING WIFI\r\n"); 
          HAL_GPIO_WritePin(Wi_EN_GPIO_Port, Wi_EN_Pin, GPIO_PIN_SET);
          HAL_Delay(100);
          HAL_UARTEx_ReceiveToIdle_IT(&huart2, (uint8_t *)wifi_rx_buf, sizeof(wifi_rx_buf));  // settign up recieve to internal buffer
           // TAM: ANY WIFI SETUP DONE HERE
           WIFI_MODE_SETUP = 1; // set flag after setup
        }
        // TODO : POWER DOWN MODE
        
        // WIFI INCOMING DATA - LIKELY SIMPLE PING IMPLEMENTED HERE
        if (wifi_rx_ready == 1) 
        {
          wifi_rx_ready = 0; // Reset the flag so we don't process the same data twice

          // TAM / SHAUN TO PROCESS INCOMING WIFI DATA HERE 

          // BELOW IS DEBUG CODE TO OUTPUT THE INCOMIGN DATA
          UART_SendString(&huart3, "WIFI DATA RECEIVED:\r\n"); 
          wifi_rx_ready = 0; // Reset the flag so we don't process the same data twice

          // DEBUG START
          UART_SendString(&huart3, (char*)wifi_rx_buf);
          UART_SendString(&huart3, "\r\n");
          HAL_UARTEx_ReceiveToIdle_IT(&huart2, (uint8_t *)wifi_rx_buf, sizeof(wifi_rx_buf));


          // TODO: IMPLEMENT NEATER WIFI SEND 
        char *msg = "UART Test: Hello from STM32!\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);

        SD_Stream_ReadDebug(DATA_START_SECTOR,100 );
          // DEBUG END
        }   

      }



      // ================== LORA ======================
      if (wake_source == 3){
        UART_SendString(&huart3, "Woke from LPUART!\r\n"); 
      }

      if (lora_rx_ready){
        // ONLY OCCURS WHEN END OF LORA PACKET. Technically the Lora Wake source is useles...... 
        lora_rx_ready = 0;

        // DEBUG START
        UART_SendString(&huart3, "LORA DATA: \r\n"); 
       char hex[8];
        for (uint16_t i = 0; i < lora_rx_len; i++)
        {
            snprintf(hex, sizeof(hex), "%02X ", lora_rx_buf[i]);
            UART_SendString(&huart3, hex);
        }
        UART_SendString(&huart3, "\r\n"); 


        // DEBUG END

      // GLENS TEMP PACKET ROUTING:
      // IF FIRST TWO BYTES SPELL YES_WIFI OR SOMETHING set WIFI_MODE = 1, IF NOW_WIFI, THEN SET WIFI_MODE = 0.
      // TAM / SHAUN TO DECODE THIS INCOMING DATA PACKET
    }


// ================= SHARC BUOY ================
      if (wake_source == 2){
      UART_SendString(&huart3, "Woke from GPIO!\r\n"); 
      // Check time 
      }
    // Chain of Device data incoming checks
    // SHARC BOUY DATA INCOMING

    uint16_t current_block_length = 0;  // how to read this... it is live incoming data ... But I decide where the end of the send is so I could shift the pointer
    uint16_t block_size;
    uint16_t last_packet_start; // Keeps track of start of prev packet for PL maths and PL placement. PL at last_packet_start -2.



    // RX BUFFER is the pointer to the end of teh DMA input
    // I WANT TO STRUCTURE DATA IN BLOCKS BEFORE SENDING INTO THE SD CARD. EACH BLOCK WILL BE: 
    // BLOCK NUMBER (16 BIT) | crc (make dummy for now) | PL (Packet length of upcoming packet) | PACKET | packet length | PACKET .....
      // First have a check to indicate that it was a DMA_IDLE interrupt here
             if (cb_write_first_half) {
              
              cb_write_first_half = 0;
              UART_SendString(&huart3, "First half of buffer ready\r\n");
              
              // Storing firstHalf in memory
              static uint8_t firstHalfCopy[RX_BUF_SIZE / 2];
             memcpy(firstHalfCopy, rxBuffer, RX_BUF_SIZE / 2);
              PackBlocks(firstHalfCopy, RX_BUF_SIZE / 2);


             // UART_SendString(&huart3, "--- BEGIN RAW ACCUM DUMP ---\r\n");

            for (uint32_t i = 0; i < accumHead; i++) {
                char hex[4];
                snprintf(hex, sizeof(hex), "%02X ", accumBuffer[i]);
              //  UART_SendString(&huart3, hex);

                // Every 32 bytes, start a new line for readability
                if ((i + 1) % 32 == 0) {
               //     UART_SendString(&huart3, "\r\n");
                }
            }
                char pbg[60];
           snprintf(pbg, sizeof(pbg), "PackBlocks: accumHead=%u\r\n", accumHead);
           // UART_SendString(&huart3, pbg);



             // PackBlocks(accumBuffer, accumHead);

              accumHead = 0;

              
              // Visualizing what the "Packed Blocks" look like before SD Write
              UART_SendString(&huart3, "Packed Block 0 Preview:\r\n");
              UART_DumpBuffer(packedBlocks[0], 512); // Assuming standard 512-byte SD blocks

              UART_SendString(&huart3, "Packed Block 1 Preview:\r\n");
               UART_DumpBuffer(packedBlocks[1], 512); // Assuming standard 512-byte SD blocks
                 
              UART_SendString(&huart3, "Packed Block 2 Preview:\r\n");
              UART_DumpBuffer(packedBlocks[2], 512); // Assuming standard 512-byte SD blocks
                
                UART_SendString(&huart3, "Packed Block 3 Preview:\r\n");
              UART_DumpBuffer(packedBlocks[3], 512); // Assuming standard 512-byte SD blocks
                    
                UART_SendString(&huart3, "Packed Block 4 Preview:\r\n");
              UART_DumpBuffer(packedBlocks[4], 512); // Assuming standard 512-byte SD blocks
                    
                UART_SendString(&huart3, "Packed Block 5 Preview:\r\n");
              UART_DumpBuffer(packedBlocks[5], 512); // Assuming standard 512-byte SD blocks
                    
            

                // RE-ENIT SD CARD
                HAL_SD_DeInit(&hsd1);       // cleanly tear down
                MX_SDMMC1_SD_Init();        // re-init SDMMC peripheral
                //HAL_Delay(100);
                
                // Process first half of the buffer
                //SD_Stream_WriteHalf(rxBuffer, RX_BUF_SIZE / 2);
                //HAL_GPIO_TogglePin(Lo_GPIO_0_GPIO_Port, Lo_GPIO_0_Pin);

                for (uint16_t i = 0; i < NUM_BLOCKS; i++) {
                    SD_Stream_WriteBlock(packedBlocks[i]);
                }

//SD_Stream_Flush();
              }
              else if (cb_write_second_half){
                cb_write_second_half = 0;
                UART_SendString(&huart3, "Second half of buffer ready\r\n");
                // RE-INIT SD CARD
                HAL_SD_DeInit(&hsd1);       // cleanly tear down
                MX_SDMMC1_SD_Init();        // re-init SDMMC peripheral
                //HAL_Delay(100);

                // Process second half of the buffer
             //   SD_Stream_WriteSecondHalf(rxBuffer + RX_BUF_SIZE / 2, RX_BUF_SIZE / 2);
                //HAL_GPIO_TogglePin(Lo_GPIO_0_GPIO_Port, Lo_GPIO_0_Pin);
               // SD_Stream_Flush();
              }
              else if (dataReady)
              {
                dataReady = 0;
                // End of transmissoin
                char hdr[48];
                sprintf(hdr, "Received %d bytes:\r\n", rxLen);
                UART_SendString(&huart3, hdr);

                // Figure out length of packet
                // Append PL to start of packet (Not sure how to do the first one because I cant force open space)
                // Append two open spaces for future PL after this packet

                // Write remaining data (if you want to - maybe only in prep for offload)
                //SD_Stream_WriteHalf(rxBuffer, rxLen);

             //    SD_Stream_Flush();
              }
     


    __HAL_GPIO_EXTI_CLEAR_IT(D_WAKE_Pin); // Clear wakeup pin interrupt flag
  
         
    HAL_Delay(500); // DElay to allow completion before resleep concerend why this is needded.. maybe dma isnt working





    // ===================== RTC FOR LORA WINDOW SETUP  =================

    // RTC WAKEUP (Lora window start / stop)
    // TOOD: MAKE THIS WAKE_SOURCE A FLAG THAT GETS SET RATHER
    if (wake_source == 1){
      UART_SendString(&huart3, "Woke from RTC!\r\n"); 

      // Check time 
      // Re-configure RTC wakeup
      HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 10, RTC_WAKEUPCLOCK_CK_SPRE_16BITS);
    }

    // TEMP OUTSIDE TO FORCE INTERRUTP 
    HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 10, RTC_WAKEUPCLOCK_CK_SPRE_16BITS);
    
    //============================= WATCH DOG TIMER ==========================
    // TODO
    // Other interrupt?
    

  

    

// ====================== GOING TO SLEEP =========================
    // ENTER STOP MODE
    wake_source = 0; // reset wake source for next loop
    // UART_SendString(&huart3, "Going to bed\r\n");
    HAL_SuspendTick();    // Need to sleep clock because systick triggers interrupt too
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);  //SLEEP NOT STOP 
    //HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI); // LPUART ISNT WORKING IN THIS MODE ATM
    SystemClock_Config();
    HAL_ResumeTick();
  /*
    
    UART_SendString(&huart3, "DEBUG: Checking wake source\r\n");

    switch (wake_source) {
      case 1: UART_SendString(&huart3, "Woke from RTC\r\n");      break;
      case 2: UART_SendString(&huart3, "Woke from GPIO\r\n");     break;
      default: UART_SendString(&huart3, "Wake source unknown\r\n"); break;
    }

    */
     /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }


  // POST WHILE LOOP
  //SD_Stream_Flush();
  HAL_Delay(500);  // give card time to settle
  UART_SendString(&huart3, "While exit\n");
  uint32_t sectors_written_read = SD_Stream_GetCurrentSector() - DATA_START_SECTOR;


  SD_Stream_ReadDebug(DATA_START_SECTOR, DATA_START_SECTOR + sectors_written_read);
  //SD_Stream_ReadDebug(DATA_START_SECTOR, sectors_written_read);


  

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
 
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10D19CE4;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE;
  hsd1.Init.ClockDiv = 0;
  if (HAL_SD_Init(&hsd1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_SD_ConfigWideBusOperation(&hsd1, SDMMC_BUS_WIDE_4B) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDMMC1_Init 2 */

  // First init with 1B bust, SD card not initifalising with 4 bits
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_1B;
  if (HAL_SD_Init(&hsd1) != HAL_OK) {
     Error_Handler();
  }
  
  // Now switch to 4 bit mode
 if (HAL_SD_ConfigWideBusOperation(&hsd1, SDMMC_BUS_WIDE_4B) != HAL_OK) {
    Error_Handler();
 }

  UART_SendString(&huart3, "SD CARD INIT\r\n");
  


  /* USER CODE END SDMMC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 14, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA2_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel4_IRQn);
  /* DMA2_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Wi_EN_Pin|Wi_RST_Pin|Wi_WAKE_Pin|Wi_GPIO_0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Wi_PWR_CTRL_Pin|Lo_PWR_CTRL_Pin|SD0_PWR_CTRL_Pin|Lo_GPIO_0_Pin
                          |Lo_GPIO_1_Pin|Lo_GPIO_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Wi_EN_Pin Wi_RST_Pin Wi_WAKE_Pin Wi_GPIO_0_Pin */
  GPIO_InitStruct.Pin = Wi_EN_Pin|Wi_RST_Pin|Wi_WAKE_Pin|Wi_GPIO_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_GPIO_INPUT_Pin */
  GPIO_InitStruct.Pin = SD_GPIO_INPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(SD_GPIO_INPUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Wi_PWR_CTRL_Pin Lo_PWR_CTRL_Pin SD0_PWR_CTRL_Pin Lo_GPIO_0_Pin
                           Lo_GPIO_1_Pin Lo_GPIO_2_Pin */
  GPIO_InitStruct.Pin = Wi_PWR_CTRL_Pin|Lo_PWR_CTRL_Pin|SD0_PWR_CTRL_Pin|Lo_GPIO_0_Pin
                          |Lo_GPIO_1_Pin|Lo_GPIO_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : D_WAKE_Pin */
  GPIO_InitStruct.Pin = D_WAKE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(D_WAKE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : D_GPIO0_Pin */
  GPIO_InitStruct.Pin = D_GPIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(D_GPIO0_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 14, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  // SET SD CARD HIGH FOR TESTING : TODO REMOVE
  HAL_GPIO_WritePin(SD0_PWR_CTRL_GPIO_Port, SD0_PWR_CTRL_Pin, GPIO_PIN_SET);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * @brief Sends a string over the specified UART peripheral.
 * @param huart: Pointer to the UART handle (e.g., &huart3)
 * @param str: The string to be sent
 */
void UART_SendString(UART_HandleTypeDef *huart, char *str) {
    HAL_UART_Transmit(huart, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}



void Print_SD_Details(void) {
    char msg[128]; // Increased buffer size to fit all details

    // Formatting multiple variables into one string
    // %lu is used for 32-bit unsigned integers (uint32_t)
    int len = sprintf(msg, 
        "\r\n--- SD Card Info ---\r\n"
        "Version:   %lu\r\n"
        "Block Size: %lu bytes\r\n"
        "Block Nbr:  %lu\r\n"
        "Total Size: %lu KB\r\n"
        "--------------------\r\n",
        (uint32_t)hsd1.SdCard.CardVersion,
        (uint32_t)hsd1.SdCard.BlockSize,
        (uint32_t)hsd1.SdCard.BlockNbr,
        (uint32_t)(hsd1.SdCard.BlockNbr * hsd1.SdCard.BlockSize / 1024) // Calculated Size
    );

    // Send the formatted string via your UART function
    UART_SendString(&huart3, msg);
    UART_SendString(&huart3, "end of setup test\r\n");

}

// Direct SDMMC write/read-back test — no FatFS involved
void SD_RawWriteTest(void) {
    char uart[64];

    // Fill write buffer with known pattern
    for (int i = 0; i < 512; i++) {
        writeBuf[i] = (uint8_t)(i & 0xFF);
    }
    memset(readBuf, 0xAA, sizeof(readBuf)); // fill read buf with garbage first

    UART_SendString(&huart3, "\r\n--- RAW HAL SD TEST ---\r\n");

    // --- WRITE ---
    HAL_StatusTypeDef wStat = HAL_SD_WriteBlocks(&hsd1, writeBuf, 100, 1, 5000);
    snprintf(uart, sizeof(uart), "WriteBlocks: %s (%d)\r\n",
             wStat == HAL_OK ? "HAL_OK" : "FAILED", wStat);
    UART_SendString(&huart3, uart);

    if (wStat != HAL_OK) {
        // Dump SD error state for more detail
        HAL_SD_CardStateTypeDef state = HAL_SD_GetCardState(&hsd1);
        snprintf(uart, sizeof(uart), "Card state after write: %d\r\n", (int)state);
        UART_SendString(&huart3, uart);

        uint32_t sdErr = hsd1.ErrorCode;    // FAILS WITH SDMMC_ERROR_CMD_CRC_FAIL implying signal integrity issue. 
        snprintf(uart, sizeof(uart), "SD ErrorCode: 0x%08lX\r\n", (unsigned long)sdErr);
        UART_SendString(&huart3, uart);
        return;
    }


    // --- WAIT for card to finish ---
    uint32_t t = HAL_GetTick();
    while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER) {
        if (HAL_GetTick() - t > 2000) {
            UART_SendString(&huart3, "Timeout waiting for card after write!\r\n");
            return;
        }
    }
    UART_SendString(&huart3, "Card ready after write.\r\n");

    // --- READ BACK ---
    HAL_StatusTypeDef rStat = HAL_SD_ReadBlocks(&hsd1, readBuf, 100, 1, 2000);
    snprintf(uart, sizeof(uart), "ReadBlocks:  %s (%d)\r\n",
             rStat == HAL_OK ? "HAL_OK" : "FAILED", rStat);
    UART_SendString(&huart3, uart);

    if (rStat != HAL_OK) return;

    // Wait for card again
    t = HAL_GetTick();
    while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER) {
        if (HAL_GetTick() - t > 2000) {
            UART_SendString(&huart3, "Timeout waiting for card after read!\r\n");
            return;
        }
    }

    // --- VERIFY ---
    int mismatches = 0;
    for (int i = 0; i < 512; i++) {
        if (readBuf[i] != writeBuf[i]) mismatches++;
    }

    snprintf(uart, sizeof(uart), "Verify: %s (%d mismatches)\r\n",
             mismatches == 0 ? "PASS" : "FAIL", mismatches);
    UART_SendString(&huart3, uart);
    UART_SendString(&huart3, "--- END RAW TEST ---\r\n\r\n");
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{


    if (huart->Instance == LPUART1)
    {
        // lora_rx_buf already contains the data, Size bytes valid
        lora_rx_buf[Size] = '\0';  // null-terminate for easy string use

        // Set a flag for main loop to process — keep ISR short
        lora_rx_len   = Size;
        lora_rx_ready = 1;
        // Trigger interrupt ot process data
        NVIC_SetPendingIRQ(EXTI0_IRQn);


        // Re-arm — matching call for this callback
        HAL_UARTEx_ReceiveToIdle_IT(&hlpuart1, lora_rx_buf, sizeof(lora_rx_buf));
    }


// Fires  at N/2, N, or idle
    // FOR SHARC BOUY ROUTING
    if (huart->Instance == USART1)
    {
        HAL_UART_RxEventTypeTypeDef event = HAL_UARTEx_GetRxEventType(huart);

        switch (event)
        {
            case HAL_UART_RXEVENT_HT:
                // First half of rxBuffer ready
                // rxBuffer[0] to rxBuffer[RX_BUF_SIZE/2 - 1]
                  char dbg[80];
               // snprintf(dbg, sizeof(dbg), "HT: first bytes: %02X %02X %02X %02X\r\n", rxBuffer[0], rxBuffer[1], rxBuffer[2], rxBuffer[3]);
                UART_SendString(&huart3, dbg);
                cb_write_first_half = 1;
                break;

            case HAL_UART_RXEVENT_TC:
                // Second half of rxBuffer ready
                // rxBuffer[RX_BUF_SIZE/2] to rxBuffer[RX_BUF_SIZE - 1]
                cb_write_second_half = 1;
                break;

          case HAL_UART_RXEVENT_IDLE:
              if (Size == lastSize) break; // No new data

              uint16_t newBytes;
              if (Size > lastSize) {
                  // Linear case: data is in one continuous chunk
                  newBytes = Size - lastSize;
              }

              // DEBUG: PRINT OUT NEW BYTES

              // Print out RXbuffer[head-size ; head]
              // --- DIRECT RX BUFFER DUMP ---
              UART_SendString(&huart3, "RX DATA: ");
              
              //for (uint16_t i = 0; i < newBytes; i++) {
              for (uint16_t i = 0; i < newBytes; i++) {
                  char hex[4];
                  // Accessing the buffer linearly from the last known position
                  snprintf(hex, sizeof(hex), "%02X ", rxBuffer[lastSize + i]);
                  UART_SendString(&huart3, hex);
              }
              //UART_SendString(&huart3, "\r\n");
              // ^^ is outputting the correct data.
              // Now load that data into the accumBuffer
              
              accumBuffer[accumHead++] = 0x00; // This will be size of value
              accumBuffer[accumHead++] = 0x00;

              // 2. Copy the data from rxBuffer into accumBuffer
              memcpy(&accumBuffer[accumHead], &rxBuffer[lastSize], newBytes);
              
              // 3. DEBUG: Print the AccumBuffer to see the result
              //UART_SendString(&huart3, "ACCUM CONTENTS: ");
              for (uint16_t i = 0; i < (newBytes + 2)*2; i++) {
                  char hex[4];
                  // We look back at what we just wrote (starting 2 bytes before the new data)
                  snprintf(hex, sizeof(hex), "%02X ", accumBuffer[(accumHead - 2) + i]);
                //  UART_SendString(&huart3, hex);
              }
             // UART_SendString(&huart3, "\r\n");

              // 4. Update the pointers
            accumHead += newBytes;
            lastSize = Size;
            }

            
    }


    // WIFI ROUTING
    // Check if the interrupt came from USART2
    if (huart->Instance == USART2)
    {
        // 1. Size = number of bytes received until IDLE or Buffer Full
        wifi_rx_len = Size;
        wifi_rx_ready = 1;

        // 2. Safety: Null-terminate for string handling
        if (Size < sizeof(wifi_rx_buf)) {
            wifi_rx_buf[Size] = '\0';
        }

        // 3. Print the formatted Hex output to your PC/Debug port (huart3)
       //UART_SendString(&huart3, "USART2 DATA RECEIVED:\r\n"); 
        
        char hex[8];
        for (uint16_t i = 0; i < Size; i++)
        {
            snprintf(hex, sizeof(hex), "%02X ", wifi_rx_buf[i]);
           // UART_SendString(&huart3, hex);
        }
       // UART_SendString(&huart3, "\r\n");

        // 4. Trigger the software interrupt for further processing logic
        NVIC_SetPendingIRQ(EXTI0_IRQn);

        // 5. IMPORTANT: Re-arm the listener for the next packet
        HAL_UARTEx_ReceiveToIdle_IT(&huart2, (uint8_t *)wifi_rx_buf, sizeof(wifi_rx_buf));
    }
}



void lora_send(uint8_t *pData, uint16_t size) {
  // Wrapper function for sending data over LORA
    // RE-ASSERT Power to LORA
    HAL_GPIO_WritePin(Lo_PWR_CTRL_GPIO_Port, Lo_PWR_CTRL_Pin, SET);
    HAL_UART_Transmit(&hlpuart1, pData, size, 100);
}




void SD_SpeedTest(void) {
    char uart[128];
    uint32_t startTime, endTime, totalTime;
    uint32_t currentSector = 10000; // Start far away from headers
    HAL_StatusTypeDef status;

    UART_SendString(&huart3, "\r\n--- STARTING 16MB STRESS TEST (Blocking) ---\r\n");
    UART_SendString(&huart3, "Preparing 16KB pattern...\r\n");

    // Initialize with a simple pattern
    for (int i = 0; i < LARGE_TEST_BURST_SIZE; i++) {
        largeTestBuf[i] = (uint8_t)(i % 256);
    }

    startTime = HAL_GetTick();

    // 1024 iterations * 16KB = 16MB
    uint32_t iterations = LARGE_TEST_TOTAL_SIZE / LARGE_TEST_BURST_SIZE;

    for (uint32_t i = 0; i < iterations; i++) {
        // Periodically print progress
        if (i % 128 == 0 && i != 0) {
            sprintf(uart, "Progress: %lu / %lu MB written...\r\n", (i * LARGE_TEST_BURST_SIZE) / (1024 * 1024), LARGE_TEST_TOTAL_SIZE / (1024 * 1024));
            UART_SendString(&huart3, uart);
        }

        // WRITE
        status = HAL_SD_WriteBlocks(&hsd1, largeTestBuf, currentSector, LARGE_TEST_BLOCKS, 5000);
        
        if (status != HAL_OK) {
            sprintf(uart, "\r\nFATAL: Write failed at Iteration %lu | Code: 0x%08lX\r\n", i, hsd1.ErrorCode);
            UART_SendString(&huart3, uart);
            return;
        }

        // WAIT for card to settle
        while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER) {
            if (HAL_GetTick() - startTime > 60000) { // 60s total timeout
                UART_SendString(&huart3, "Timeout waiting for card!\r\n");
                return;
            }
        }

        currentSector += LARGE_TEST_BLOCKS;
    }

    endTime = HAL_GetTick();
    totalTime = endTime - startTime;
    
    // Calculate final stats
    float totalMB = (float)LARGE_TEST_TOTAL_SIZE / (1024.0f * 1024.0f);
    float seconds = (float)totalTime / 1000.0f;
    float speedKBps = ((float)LARGE_TEST_TOTAL_SIZE / 1024.0f) / seconds;

    sprintf(uart, "\r\n--- TEST COMPLETE ---\r\n");
    UART_SendString(&huart3, uart);
    sprintf(uart, "Total Data: %.2f MB\r\n", totalMB);
    UART_SendString(&huart3, uart);
    sprintf(uart, "Total Time: %.2f seconds\r\n", seconds);
    UART_SendString(&huart3, uart);
    sprintf(uart, "Final Speed: %.2f KB/s\r\n", speedKBps);
    UART_SendString(&huart3, uart);
}

void PackBlocks(const uint8_t *src, uint16_t dataLen) {
    // Determine how many bytes go into each block based on total dataLen
    uint16_t remaining = dataLen;

    for (uint16_t i = 0; i < NUM_BLOCKS; i++) {
        uint8_t *block = packedBlocks[i];
        const uint8_t *data = src + (i * BLOCK_DATA_SIZE);
        
        // Calculate length for THIS specific block
        uint16_t currentBlockLen = (remaining > BLOCK_DATA_SIZE) ? BLOCK_DATA_SIZE : remaining;
        
        // Calculate CRC only for the actual data present in this block
        uint16_t crc = CRC16(data, currentBlockLen);

        // --- NEW HEADER STRUCTURE ---
        // Byte 0-1: Block Length (High byte, then Low byte)
        block[0] = (currentBlockLen >> 8) & 0xFF; 
        block[1] =  currentBlockLen       & 0xFF; 
        
        // Byte 2-3: CRC of the data
        block[2] = (crc >> 8) & 0xFF;
        block[3] =  crc            & 0xFF;

        // Write data
        if (currentBlockLen > 0) {
            memcpy(&block[4], data, currentBlockLen);
        }

        // Optional: Zero out the rest of the 512-byte block if currentBlockLen < BLOCK_DATA_SIZE
        if (currentBlockLen < BLOCK_DATA_SIZE) {
            memset(&block[4 + currentBlockLen], 0, BLOCK_DATA_SIZE - currentBlockLen);
        }

        // Update remaining count
        if (remaining >= currentBlockLen) {
            remaining -= currentBlockLen;
        } else {
            remaining = 0;
        }


      
    }
}

uint16_t CRC16(const uint8_t *data, uint16_t length) {
    return 69;
}

void UART_DumpBuffer(uint8_t *data, uint32_t len) {
    char msg[64];
    for (uint32_t i = 0; i < len; i += 16) {
        sprintf(msg, "%04X: ", (unsigned int)i);
        UART_SendString(&huart3, msg);
        
        for (int j = 0; j < 16; j++) {
            if (i + j < len) {
                sprintf(msg, "%02X ", data[i + j]);
            } else {
                sprintf(msg, "   ");
            }
            UART_SendString(&huart3, msg);
        }
        UART_SendString(&huart3, "\r\n");
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
