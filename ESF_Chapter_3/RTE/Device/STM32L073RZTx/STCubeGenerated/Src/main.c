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
#define W_DELAY_SLOW		150
#define W_DELAY_FAST		70
#define RGB_DELAY_SLOW		1000
#define RGB_DELAY_FAST		150


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t g_flash_LED = 0;
volatile uint8_t g_flash_LED_changed = 1;
volatile uint32_t g_w_delay = W_DELAY_SLOW;
volatile uint32_t g_RGB_delay = RGB_DELAY_SLOW;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//void RGB_Flasher_Init(void);
//void RGB_Flasher_Sequential(void);
//void RGB_Flasher_Parallel(void);
void Flasher(void);
void Task_Read_Switches(void);
//void Task_Flash(void);
//void Task_RGB(void);
void Task_Flash_FSM_Timer(void);
void Task_RGB_FSM_Timer(void);
void EXTI4_15_IRQHandler(void);
unsigned int TIM_Expired(void);
void Stop_TIM(void);
void Start_TIM(uint32_t delay);
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
void EXTI4_15_IRQHandler(void) {
	if (EXTI->PR & MASK(SW1_POS)) {
		// Acknowledge interrupt by writing 1 to bit, clearing it (!) 
		EXTI->PR = MASK(SW1_POS);
		// Process interrupt
		if (SWITCH_PRESSED(SW1_POS)) {
			g_flash_LED = 1;					// Flash white
		} else {
			g_flash_LED = 0;					// RGB sequence
		}
	}
	if (EXTI->PR & MASK(SW2_POS)) {
		// Acknowledge interrupt by writing 1 to bit, clearing it (!) 
		EXTI->PR = MASK(SW2_POS);
		// Process interrupt
		if (SWITCH_PRESSED(SW2_POS)) {	// Short delays
			g_w_delay = W_DELAY_FAST;
			g_RGB_delay = RGB_DELAY_FAST;
		} else {										// Long delays
			g_w_delay = W_DELAY_SLOW;
			g_RGB_delay = RGB_DELAY_SLOW;
		}
	}
	// NVIC Acknowledge interrupt
	NVIC_ClearPendingIRQ(EXTI4_15_IRQn);
}

void Task_RGB_FSM_Timer(void) {
	enum State { ST_RED, ST_RED_WAIT, ST_GREEN, ST_GREEN_WAIT,
			ST_BLUE, ST_BLUE_WAIT
	};
	static enum State next_state;
	
	if (g_flash_LED == 0) {
		if (g_flash_LED_changed) {
			next_state = ST_RED;
			g_flash_LED_changed = 0;
		}
		switch (next_state) {
			case ST_RED:
				Control_RGB_LEDs(1, 0, 0);
				Start_TIM(g_RGB_delay);
				next_state = ST_RED_WAIT;
				break;
			case ST_RED_WAIT:
				if (TIM_Expired()) {
					Stop_TIM();
					next_state = ST_GREEN;
				}
				break;
			case ST_GREEN:
				Control_RGB_LEDs(0, 1, 0);
				Start_TIM(g_RGB_delay);
				next_state = ST_GREEN_WAIT;
				break;
			case ST_GREEN_WAIT:
				if (TIM_Expired()) {
					Stop_TIM();
					next_state = ST_BLUE;
				}
				break;
			case ST_BLUE:
				Control_RGB_LEDs(0, 0, 1);
				Start_TIM(g_RGB_delay);
				next_state = ST_BLUE_WAIT;
				break;
			case ST_BLUE_WAIT:
				if (TIM_Expired()) {
					Stop_TIM();
					next_state = ST_RED;
				}
				break;
			default:
				next_state = ST_RED;
				break;
		}
	}
}

void Task_Flash_FSM_Timer(void) {
	enum State { ST_WHITE, ST_WHITE_WAIT, ST_BLACK, ST_BLACK_WAIT };
	static enum State next_state = ST_WHITE;
	
	if (g_flash_LED == 1) {
		switch (next_state) {
			case ST_WHITE:
				Control_RGB_LEDs(1, 1, 1);
				Start_TIM(g_w_delay);
				next_state = ST_WHITE_WAIT;
				break;
			case ST_WHITE_WAIT:
				if (TIM_Expired()) {
					Stop_TIM();
					next_state = ST_BLACK;
				}
				break;
			case ST_BLACK:
				Control_RGB_LEDs(0, 0, 0);
				Start_TIM(g_w_delay);
				next_state = ST_BLACK_WAIT;
				break;
			case ST_BLACK_WAIT:
				if (TIM_Expired()) {
					Stop_TIM();
					next_state = ST_WHITE;
				}
				break;
			default:
				next_state = ST_WHITE;
				break;
		}
	} else {
		next_state = ST_WHITE;
	}
}

unsigned int TIM_Expired(void) {
	return TIM3->SR & TIM_SR_UIF;
}

void Stop_TIM(void) {
	TIM3->SR &= ~TIM_SR_UIF; // must clear manually!
	TIM3->CR1 &= ~TIM_CR1_CEN;
}

#define TIM_PRESCALER (4800)

void Start_TIM(uint32_t delay) {
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	// Stop timer in case it was already running
	Stop_TIM();
	// Initialize timer
	TIM3->SMCR = 0;
	TIM3->CR1 |= TIM_CR1_DIR | TIM_CR1_OPM | TIM_CR1_ARPE; // count down, one-pulse mode, enable preload
	TIM3->CR2 = 0;
	TIM3->EGR = 0;
	TIM3->ARR = delay;
	TIM3->PSC = TIM_PRESCALER - 1;
	TIM3->CR1 |= TIM_CR1_CEN;
}

void Flasher(void) {
	Init_GPIO_RGB();
	Init_GPIO_Switches_Interrupts();
	while (1) {
		Task_Flash_FSM_Timer();
		Task_RGB_FSM_Timer();
	}
}

/*
void Task_Read_Switches(void) {
	if (SWITCH_PRESSED(SW1_POS)) {
		g_flash_LED = 1;		// Flash white
	} else {
		g_flash_LED = 0;		// RGB Sequence
	}
	if (SWITCH_PRESSED(SW2_POS)) {
		g_w_delay = W_DELAY_FAST;
		g_RGB_delay = RGB_DELAY_FAST;
	} else {
		g_w_delay = W_DELAY_SLOW;
		g_RGB_delay = RGB_DELAY_SLOW;
	}
}
*/

void Task_Flash(void) {
	if (g_flash_LED == 1) {
		Control_RGB_LEDs(1, 1, 1);
		Delay(g_w_delay);
		Control_RGB_LEDs(0, 0, 0);
		Delay(g_w_delay);
	}		
}

void Task_RGB(void) {
	if (g_flash_LED == 0) {
		Control_RGB_LEDs(1, 0, 0);
		Delay(g_RGB_delay);
		Control_RGB_LEDs(0, 1, 0);
		Delay(g_RGB_delay);
		Control_RGB_LEDs(0, 0, 1);
		Delay(g_RGB_delay);
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
