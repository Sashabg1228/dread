/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "NRF24L01.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

const uint8_t rx_address[] = {0xEE, 0xDD, 0xCC, 0xBB, 0xAA};

Payload payload = {0, 0, 0, 0};

// 0 = disk;	1 = right;	2 = left
volatile uint8_t cycles[3] = {0, 0, 0};
volatile uint8_t enable[3] = {1, 1, 1};
// 0 = right(down)	1 = left(up)
volatile uint8_t hand_enable[2] = {1, 1};
volatile uint8_t hand_counter = 0;
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
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  NRF24_init();
  NRF24_RX_mode(rx_address, 120);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // TODO: replace the pooling
    if (0 == is_data_received(1))
    {
      if (0 == NRF24_receive(&payload))
      {
	  unload_payload();
      }
    }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
}

/* USER CODE BEGIN 4 */
/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* TODO: sadly this interrupt still does not work as I want to
  if (GPIO_Pin == NRF24L01_IRQ_Pin)
  {
    // payload_recieved = 1;
  }
  */

  if (GPIO_Pin == OPT_END_DISK_Pin)
  {
    cycles[0]++;
  }
  if (GPIO_Pin == OPT_END_WHEEL_RIGHT_Pin)
  {
    cycles[1]++;
  }
  if (GPIO_Pin == OPT_END_WHEEL_LEFT_Pin)
  {
    cycles[2]++;
  }



  if (GPIO_Pin == HAND_END_RIGHT_Pin)
  {
    hand_enable[0] = !hand_enable[0];
  }
  if (GPIO_Pin == HAND_END_LEFT_Pin)
  {
    hand_enable[1] = !hand_enable[1];
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
  if (&htim2 == htim)
  {
    if (2 * payload.hand_position != hand_counter)
    {
      hand_counter++;
      HAL_GPIO_TogglePin(HAND_PWM_GPIO_Port, HAND_PWM_Pin);
    }
    else
    {
      payload.hand_position = 0;
      hand_counter = 0;
    }
  }

  if (&htim4 == htim)
  {
    for (int i = 0; i < 3; i++)
    {
      if (enable[i] && 0 == cycles[i])
      {
	enable[i] = 0;
	cycles[i] = 0;
      }
      else if (!enable[i] && 4 > cycles[i])
      {
	cycles[i]++;
      }
      else
      {
	enable[i] = 1;
	cycles[i] = 0;
      }
    }
  }
}

void get_brushed_DIR(void)
{
  HAL_GPIO_WritePin(DISK_DIR_GPIO_Port, DISK_DIR_Pin, SET);

  if (payload.left_speed < 0)
  {
    payload.left_speed *= -1;
    HAL_GPIO_WritePin(WHEEL_LEFT_DIR_GPIO_Port, WHEEL_LEFT_DIR_Pin, RESET);
  }
  else
  {
    HAL_GPIO_WritePin(WHEEL_LEFT_DIR_GPIO_Port, WHEEL_LEFT_DIR_Pin, SET);
  }

  if (payload.right_speed < 0)
  {
    payload.right_speed *= -1;
    HAL_GPIO_WritePin(WHEEL_RIGHT_DIR_GPIO_Port, WHEEL_RIGHT_DIR_Pin, RESET);
  }
  else
  {
    HAL_GPIO_WritePin(WHEEL_RIGHT_DIR_GPIO_Port, WHEEL_RIGHT_DIR_Pin, SET);
  }
}

void get_brushed_PWM(void)
{
  // ARR => TOP
  TIM3->CCR2 = enable[0] * payload.weapon * TIM3->ARR;
  TIM3->CCR3 = enable[2] * payload.left_speed * TIM3->ARR;
  TIM3->CCR4 = enable[1] * payload.right_speed * TIM3->ARR;
}

void get_hand_DIR(void)
{
  if (hand_enable[0] && payload.hand_position < 0)
  {
    payload.hand_position *= -1;
    HAL_GPIO_WritePin(HAND_DIR_GPIO_Port, HAND_DIR_Pin, RESET);
  }
  else if (hand_enable[1])
  {
    HAL_GPIO_WritePin(HAND_DIR_GPIO_Port, HAND_DIR_Pin, SET);
  }
  else
  {
    payload.hand_position = 0; // stop
  }
}

void unload_payload(void)
{
  get_brushed_DIR();
  get_brushed_PWM();

  get_hand_DIR();
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
