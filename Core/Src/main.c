/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "liquidcrystal_i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SIZE_OF_BUFFER 	500
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim17;
DMA_HandleTypeDef hdma_tim17_ch1_up;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
FATFS       FatFs;                //Fatfs handle
FIL         fil;                  //File handle
DIR 		dp;
FILINFO 	fno;
DWORD       fileSize;
FRESULT     fres;                 //Result after operations
char        buf[100];
//FIL old_fil_data;
DWORD	old_fptr;			/* File read/write pointer (Zeroed on file open) */
DWORD	old_fsize;			/* File size */
uint8_t buff_music[SIZE_OF_BUFFER];

struct
{
	int var_recording : 1;
	int sampling_timer : 4;
}general;

struct
{
	int page : 4;
	/*   Page number description
	 * 0: Start menu
	 * 1: Music
	 * 2: Recording
	 */
	uint8_t cursor_line;
	uint8_t file_number;
	char music_name[10]
}display_status;

uint16_t tim17_counter;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void start_play_music(const TCHAR* path);
void stop_play_music();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	memset(buff_music, 0, sizeof(buff_music));
	general.var_recording = 0;
	general.sampling_timer = 0;
	tim17_counter = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_FATFS_Init();
  MX_DMA_Init();
  MX_TIM17_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  process_SD_card();

	HD44780_Init(2);
	HD44780_Clear();
	HD44780_SetCursor(0,0);
	HD44780_PrintStr("     HELLO");
	HD44780_SetCursor(10,1);
	HAL_Delay(2000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_TIM17;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
  PeriphClkInit.Tim17ClockSelection = RCC_TIM17CLK_PLLCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10707DBC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 64;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 30;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 255;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */
  HAL_TIM_MspPostInit(&htim17);

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB10 PB11 PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC10 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
#ifdef __GNUC__
  /* With GCC, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
int __io_putchar(int ch)
#else
int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the UART3 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

void process_SD_card( void )
{
	f_mount(NULL, "", 0);
	fres = f_mount(&FatFs, "", 1);
	fres = f_open(&fil, "0:/data/data_file.txt", FA_WRITE | FA_READ | FA_CREATE_ALWAYS);
	fil.fsize = old_fsize;
	if (fres == FR_OK) {
	    UINT bytesWritten;
	    f_lseek(&fil, f_size(&fil) /*sizeof(fil)*/);
	    DWORD size_of_fil;
	    size_of_fil = f_size(&fil);
	    printf("size of file 1 : %d \r\n", size_of_fil);
		memset(buff_music, 'A', sizeof(buff_music));
		fres = f_write(&fil, buff_music, sizeof(buff_music), &bytesWritten);
		size_of_fil = f_size(&fil);
		printf("size of file 2: %d \r\n", size_of_fil);
		 if (fres == FR_OK) {
				// Data written successfully
				f_close(&fil);  // Close the file
				printf("Data written to file: data_file.txt \r\n");
				//printf("strlen(buff_adc1) == %d   	sizeof(buff_adc1) == %d \r\n", strlen(buff_adc1), sizeof(buff_adc1));
			} else {
				// Error occurred while writing data
				f_close(&fil);  // Close the file
				printf("Error writing data to file: data_file.txt \r\n");
			}
		    } else {
		        // Error occurred while opening the file
		        printf("Error opening file: data_file.txt \r\n");
		    }
		  f_close(&fil);
		  old_fsize = fil.fsize;
		  //f_mount(NULL, "", 0);
}

HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_13)  //Esc
	{
		//Start menu
		HD44780_Clear();
		HD44780_SetCursor(0,0);
		HD44780_PrintStr("1. Music");
		HD44780_SetCursor(0,1);
		HD44780_PrintStr("2. Recording");
	    HD44780_Cursor();
	    HD44780_Blink();
	    HD44780_SetCursor(0,0);
	    display_status.page = 0;
	    display_status.cursor_line = 0;

	    stop_play_music(&fil);
	}
	else if(GPIO_Pin == GPIO_PIN_10)  // Enter
	{
		if(display_status.page == 0 && display_status.cursor_line ==0) // Music
		{
			HD44780_Clear();
			HD44780_SetCursor(0,0);
			display_status.page = 1;
			display_status.cursor_line = 0;
			fres = f_mount(&FatFs, "", 1);
			fres=f_opendir(&dp,"0:/music");
			printf("fres = %d \r\n", fres);
			display_status.file_number = 0;
			do
			{
				fres=f_readdir(&dp, &fno);
				printf("file name: %s \r\n", fno.fname);
				sprintf(buff_music,"%d. %s", display_status.file_number+1, fno.fname);
				if(display_status.file_number<2)
				{
					HD44780_SetCursor(0,display_status.file_number);
					HD44780_PrintStr(buff_music);
				}
				display_status.file_number++;
			}
			while (fres == FR_OK && fno.fname[0] !='\0');
			display_status.file_number =1; //Display can show only 2 lines
			f_mount(NULL, "", 0);
			HD44780_SetCursor(0,0);
		}
		else if(display_status.page == 1 && display_status.cursor_line ==0)
		{
			fres = f_mount(&FatFs, "", 1);
			fres=f_opendir(&dp,"0:/music");
			uint8_t n = display_status.file_number;
			display_status.file_number=0;
			do
			{
				fres=f_readdir(&dp, &fno);
				printf("file name: %s \r\n", fno.fname);
				sprintf(display_status.music_name,"%s", fno.fname);
				display_status.file_number++;
			}while(fno.fname[0] !='\0' &&  display_status.file_number<=n-1);
			display_status.file_number=n;
			f_mount(NULL, "", 0);
			char path[30];
			sprintf(path, "0:/music/%s", display_status.music_name);
			start_play_music(path);
		}
		else if(display_status.page == 1 && display_status.cursor_line ==1)
		{
			fres = f_mount(&FatFs, "", 1);
			fres=f_opendir(&dp,"0:/music");
			uint8_t n = display_status.file_number;
			display_status.file_number=0;
			do
			{
				fres=f_readdir(&dp, &fno);
				printf("file name: %s \r\n", fno.fname);
				sprintf(display_status.music_name,"%s", fno.fname);
				display_status.file_number++;
			}while(fno.fname[0] !='\0' &&  display_status.file_number<=n);
			display_status.file_number=n;
			f_mount(NULL, "", 0);
			char path[30];
			sprintf(path, "0:/music/%s", display_status.music_name);
			start_play_music(path);
		}

		if(display_status.page == 0 && display_status.cursor_line ==1) // Recording
		{
			HD44780_Clear();
			HD44780_SetCursor(0,0);
			HD44780_PrintStr("1.New Recording");
			HD44780_SetCursor(0,1);
			HD44780_PrintStr("2.Listen Record");
			display_status.page = 2;
			display_status.cursor_line = 0;
			HD44780_SetCursor(0,0);

		}
		else if(display_status.page == 2 && display_status.cursor_line ==0) //New recording
		{
			GPIOC->ODR |= GPIO_ODR_3;
			HAL_TIM_Base_Start_IT(&htim6);
		}
		else if(display_status.page == 2 && display_status.cursor_line ==1) //Listen record
		{
			GPIOB->ODR |= GPIO_ODR_0;
			HAL_Delay(10);
			GPIOB->ODR &= ~GPIO_ODR_0;
		}
	}
	else if(GPIO_Pin == GPIO_PIN_11)  // ->
	{
		if(display_status.page == 0) {HD44780_SetCursor(0,1); display_status.cursor_line =1;}
		if(display_status.page == 1)
		{
			if(display_status.cursor_line == 1)
			{
				fres = f_mount(&FatFs, "", 1);
				fres=f_opendir(&dp,"0:/music");
				display_status.file_number++;
				uint8_t n = display_status.file_number;
				display_status.file_number=0;
				HD44780_Clear();
				do
				{
					fres=f_readdir(&dp, &fno);
					printf("file name: %s \r\n", fno.fname);
					sprintf(buff_music,"%d. %s", display_status.file_number+1, fno.fname);
					if(display_status.file_number == n-1)
					{
						HD44780_SetCursor(0,0);
						HD44780_PrintStr(buff_music);
					}
					else if(display_status.file_number == n)
					{
						HD44780_SetCursor(0,1);
						HD44780_PrintStr(buff_music);
					}
					display_status.file_number++;
				}while(fno.fname[0] !='\0' &&  display_status.file_number<=n);
				display_status.file_number=n;
				//f_mount(NULL, "", 0);
			}
			HD44780_SetCursor(0,1);
			display_status.cursor_line =1;
		}
		if(display_status.page == 2)
		{
			HD44780_SetCursor(0,1);
			display_status.cursor_line =1;
		}

	}
	else if(GPIO_Pin == GPIO_PIN_12)  // <-
	{
		if(display_status.page == 0) {HD44780_SetCursor(0,0); display_status.cursor_line =0;}
		if(display_status.page == 1)
		{
			if(display_status.cursor_line == 0)
			{
				fres = f_mount(&FatFs, "", 1);
				fres=f_opendir(&dp,"0:/music");
				display_status.file_number--;
				uint8_t n = display_status.file_number;
				display_status.file_number=0;
				HD44780_Clear();
				do
				{
					fres=f_readdir(&dp, &fno);
					printf("file name: %s \r\n", fno.fname);
					sprintf(buff_music,"%d. %s", display_status.file_number+1, fno.fname);
					if(display_status.file_number == n-1)
					{
						HD44780_SetCursor(0,0);
						HD44780_PrintStr(buff_music);
					}
					else if(display_status.file_number == n)
					{
						HD44780_SetCursor(0,1);
						HD44780_PrintStr(buff_music);
					}
					display_status.file_number++;
				}while(fno.fname[0] !='\0' &&  display_status.file_number<=n);
				display_status.file_number=n;
				//f_mount(NULL, "", 0);
			}
			HD44780_SetCursor(0,0);
			display_status.cursor_line =0;
		}
		if(display_status.page == 2)
		{
			HD44780_SetCursor(0,0);
			display_status.cursor_line =0;
		}
	}
}

void start_play_music(const TCHAR* path)
{
	fres = f_mount(&FatFs, "", 1);    //1=mount now  "" == '\0' == 0
  fres = f_open(&fil, path, FA_READ );
  if (fres == FR_OK) {
	  if((TIM17->DIER &(1<<0)) == 0)
	  {
		  TIM17->DIER |=1;
		  HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
	  }
  }
}

void stop_play_music()
{
	TIM17->DIER &=~1;
	HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
	tim17_counter = 0;
	f_close(&fil);
	fres = f_mount(NULL, "", 0);
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

#ifdef  USE_FULL_ASSERT
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
