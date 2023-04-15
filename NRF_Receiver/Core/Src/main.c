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
#include "adc.h"
#include "i2c.h"
#include "iwdg.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "NRF2401/nrf2401.h"
#include "NRF2401/nrf2401_defs.h"
#include "SSD1306_OLED.h"
#include "GFX_BW.h"
#include "fonts/fonts.h"
#include "drv8835.h"
#include "lsm303dlhc.h"
#include "clock.h"
#include "frame.h"
#include "putchar.h"
#include "lights.h"
#include "battery.h"
#include "watchdog.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MESSAGE_LENGTH 3

#define ACCELERATION_VALUE 0
#define VEER_VALUE 1
#define LIGHTS_STATE 2

#define INTERLINE 10

#define MS_500 500
#define MS_1000 1000
#define MS_200 200
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
T_Frame frame = MOTORS;

uint8_t rx_data[MESSAGE_LENGTH];

char oled_message[32];
uint8_t line = 0;

double battery_voltage;


float acc_x, acc_y, acc_z;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void Scan_I2C();
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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_IWDG_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  WDG_ResetInfo();

  Clock_Init(&htim1);
  NRF_Init(&hspi1, 'r');
  DRV8835_Init(&htim3, TIM_CHANNEL_1, TIM_CHANNEL_2);
  Battery_Init(&hadc1, ADC_CHANNEL_10);
  LSM303DLHC_Init(&hi2c2);

  SSD1306_Init(&hi2c2);
  GFX_SetFont(font_8x5);
  SSD1306_Clear(BLACK);
  SSD1306_Display();

  uint32_t display_refresh = HAL_GetTick();
  //Scan_I2C();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  NRF_process(rx_data);
	  if(NRF_IsMessageReceived())
	  {
		  switch(frame)
		  {
		  	  case MOTORS:
		  		  	  DRV8835_Move(rx_data[ACCELERATION_VALUE], rx_data[VEER_VALUE]);
					  frame = LIGHTS;
		  		  continue;

		  	  case LIGHTS:
		  		  	  Lights_SetState(LIGHTS_GPIO_Port, LIGHTS_Pin, rx_data[LIGHTS_STATE]);
		  		  	  frame = MOTORS;
		  		  break;
		  }

		  if(HAL_GetTick() - display_refresh >= MS_1000)
		  {
			  SSD1306_Clear(BLACK);

			  battery_voltage = Battery_GetVoltage();
			  LSM303DLHC_ReadValues(&acc_x, &acc_y, &acc_z);

			  sprintf(oled_message, "acceleration:[%d]", rx_data[ACCELERATION_VALUE]);
			  GFX_DrawString(0, line, oled_message, WHITE, 0);

			  sprintf(oled_message, "veer:[%d]", rx_data[VEER_VALUE]);
			  GFX_DrawString(0, line+=INTERLINE, oled_message, WHITE, 0);

			  sprintf(oled_message, "lights:[%d]", rx_data[LIGHTS_STATE]);
			  GFX_DrawString(0, line+=INTERLINE, oled_message, WHITE, 0);

			  sprintf(oled_message, "Vbat = %.2f", battery_voltage);
			  GFX_DrawString(0, line+=INTERLINE, oled_message, WHITE, 0);

			  sprintf(oled_message, "X %4s Y %4s Z", "", "");
			  GFX_DrawString(0, line+=INTERLINE, oled_message, WHITE, 0);

			  sprintf(oled_message, "%.2f | %.2f | %.2f", acc_x, acc_y, acc_z);
			  GFX_DrawString(0, line+=INTERLINE, oled_message, WHITE, 0);

			  line = 0;
			  SSD1306_Display();

			  display_refresh = HAL_GetTick();
		  }

		  //printf("ACC: %d \t", rx_data[0]);
		  //printf("VEER: %d \t", rx_data[1]);
		  //printf("LIGHTS: %d \n", rx_data[2]);
		  NRF_ReceiveNextMessage();
	  }
	  WDG_Refresh(&hiwdg);
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM1_BRK_UP_TRG_COM_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
}

/* USER CODE BEGIN 4 */
void Scan_I2C()
{
	printf("Scanning I2C bus:\r\n");
	HAL_StatusTypeDef result;
	uint8_t i;
	for (i=1; i<128; i++)
	{
		/*
		* the HAL wants a left aligned i2c address
		* &hi2c1 is the handle
		* (uint16_t)(i<<1) is the i2c address left aligned
		* retries 2
		* timeout 2
		*/
		result = HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(i<<1), 2, 2);
		if (result != HAL_OK) // HAL_ERROR or HAL_BUSY or HAL_TIMEOUT
		{
		  printf("."); // No ACK received at that address
		}
		if (result == HAL_OK)
		{
		  printf("0x%X", i); // Received an ACK at that address
		}
	}
	printf("\r\n");
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
