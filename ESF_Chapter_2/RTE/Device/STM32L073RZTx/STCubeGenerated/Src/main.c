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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32l073xx.h"
#include "switch.h"
#include "delay.h"
#include "field_access.h"
#include "gpio.h"
#include "rgb.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*	Activate-High LED control definitions
*		0 out = LED off
*		1 out = LED on		Inverse to what is is the book due to my led being common cathode
*/
#define W_DELAY_SLOW			150
#define W_DELAY_FAST			70
#define RGB_DELAY_SLOW		400
#define RGB_DELAY_FAST		100


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//void RGB_Flasher_Init(void);
//void RGB_Flasher_Sequential(void);
//void RGB_Flasher_Parallel(void);
void Flasher(void);
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
  /* USER CODE BEGIN 2 */
	//RGB_Flasher_Init();
	// Comment out the unwanted flasher version
	//RGB_Flasher_Sequential();
	//RGB_Flasher_Parallel();
	Flasher();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void Flasher(void) {
			uint32_t w_delay = W_DELAY_SLOW;
	uint32_t RGB_delay = RGB_DELAY_SLOW;

	Init_GPIO_RGB();
	Init_GPIO_Switches();
	while (1) {
		if (SWITCH_PRESSED(SW1_POS)) {	// flash white
			Control_RGB_LEDs(1, 1, 1);
			Delay(w_delay);
			Control_RGB_LEDs(0, 0, 0);
			Delay(w_delay);
		} else {										// sequence R, G, B
			Control_RGB_LEDs(1, 0, 0);
			Delay(RGB_delay);
			Control_RGB_LEDs(0, 1, 0);
			Delay(RGB_delay);
			Control_RGB_LEDs(0, 0, 1);
			Delay(RGB_delay);
		}
		if (SWITCH_PRESSED(SW2_POS)) {
			w_delay = W_DELAY_FAST;
			RGB_delay = RGB_DELAY_FAST;
		} else {
			w_delay = W_DELAY_SLOW;
			RGB_delay = RGB_DELAY_SLOW;
		}
	}
}

/*
void RGB_Flasher_Init(void) {
	// Enable peripheral clock of GPIOA (for LD2)	RCC->IOPENR
	RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
	// Configure pins in output mode (01=1) 
	MODIFY_FIELD(GPIOA->MODER, GPIO_MODER_MODE5, ESF_GPIO_MODER_OUTPUT);
	MODIFY_FIELD(GPIOA->MODER, GPIO_MODER_MODE6, ESF_GPIO_MODER_OUTPUT);
	MODIFY_FIELD(GPIOA->MODER, GPIO_MODER_MODE7, ESF_GPIO_MODER_OUTPUT);
	
	// Turn on LED's
	GPIOA->BSRR = LED_B_ON_MSK | LED_G_ON_MSK | LED_R_ON_MSK;

}

void RGB_Flasher_Sequential(void) {
	unsigned int num = 0;

	while (1) {
		num++;
		if (num & 1)
			GPIOA->BSRR = LED_R_ON_MSK;
		else
			GPIOA->BSRR = LED_R_OFF_MSK;
		if (num & 2)
			GPIOA->BSRR = LED_G_ON_MSK;
		else
			GPIOA->BSRR = LED_G_OFF_MSK;
		if (num & 4)
			GPIOA->BSRR = LED_B_ON_MSK;
		else
			GPIOA->BSRR = LED_B_OFF_MSK;
		Delay(400);
	}
}

void RGB_Flasher_Parallel(void) {
	unsigned int num = 0;

	while (1) {
		num++;
		GPIOA->ODR &= ~((0x07) << 5);	// Clear all bits in field
		GPIOA->ODR |= (num & 0x07) << 5;	// Set given bits in field
		Delay(400);
	}
}
*/
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
