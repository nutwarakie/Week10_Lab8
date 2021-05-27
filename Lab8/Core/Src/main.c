/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>//sprintf
#include <string.h>//strlenght
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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

char LEDFrequency[20] = { 0 };
char TxDataBuffer[32] = { 0 };
char RxDataBuffer[32] = { 0 };
enum _StateDisplay {
	State_Start = 0,
	State_Mainmenu_Print = 10,
	State_Mainmenu_Wait,
	State_LEDMenu_Print,
	State_LEDMenu_Wait,
	State_ButtonMenu_Print = 20,
	State_ButtonMenu_Wait
};

uint8_t STATE = 0;
float f = 1;
uint32_t LED_HalfPeriod = 500 ;
uint32_t TimeStamp = 0;
uint8_t LEDBlink = 0;

uint8_t Freq = 0;
uint8_t Interrupt = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void UARTRecieveAndResponsePolling();
int16_t UARTRecieveIT();
int16_t UARTRecieveIT();
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */


	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		/*Method 2 Interrupt Mode*/
		HAL_UART_Receive_IT(&huart2, (uint8_t*) RxDataBuffer, 32);

		/*Method 2 W/ 1 Char Received*/
		int16_t inputchar = UARTRecieveIT();


		switch (STATE) {

		case State_Start:
			STATE = State_Mainmenu_Print;
			break;

		case State_Mainmenu_Print:

		{
			char Mainmenu[] =
					"Main Menu\r\n 0.LED Control\r\n 1.Button Status\r\n";
			HAL_UART_Transmit(&huart2, (uint8_t*) Mainmenu, strlen(Mainmenu),
					100);
		}

			STATE = State_Mainmenu_Wait;
			break;

		case State_Mainmenu_Wait:

			if(inputchar != -1){

				switch (inputchar)
				{
				case 0:
					break;
				case '0':
					STATE = State_LEDMenu_Print;
					break;
				case '1':
					STATE = State_ButtonMenu_Print;
					break;
				default:
					HAL_UART_Transmit(&huart2, (uint8_t*) "unidentified input\r\n", strlen("unidentified input\r\n"),1000);
					STATE = State_Mainmenu_Print;
					break;

				}

			}

			break;

		case State_LEDMenu_Print:

		{
			char LEDmenu[] =
					"LED Control\r\n a.Speed Up +1Hz\r\n s.SpeedDown-1Hz\r\n d.ON/OFF\r\n x.back\r\n";
			HAL_UART_Transmit(&huart2, (uint8_t*) LEDmenu, strlen(LEDmenu),
					100);
		}
			STATE = State_LEDMenu_Wait;

			break;

		case State_LEDMenu_Wait:

			if(inputchar != -1){
				switch (inputchar)
				{
					case 0:
						break;
					case 'a':
						f+=1;
						LED_HalfPeriod = ((1/f)/2)/0.001;

						Freq = f ;
						sprintf(LEDFrequency, "LEDFrequency:[%u]Hz\r\n",Freq);

						HAL_UART_Transmit(&huart2, (uint8_t*) LEDFrequency, strlen(LEDFrequency),1000);

						STATE = State_LEDMenu_Wait;
						break;
					case 's':
						if (f>0)
						{
						  f-=1;
						  LED_HalfPeriod= (((1/f)/2)/0.001);

						 }

						Freq = f ;
						sprintf(LEDFrequency, "LEDFrequency:[%u]Hz\r\n",Freq);
						HAL_UART_Transmit(&huart2, (uint8_t*) LEDFrequency, strlen(LEDFrequency),1000);
						STATE = State_LEDMenu_Wait;
						break;
					case 'd':
						if(LEDBlink == 0 )
						{
							LEDBlink = 1;
							HAL_UART_Transmit(&huart2, (uint8_t*) "LED ON\r\n", strlen("LED ON\r\n"),1000);
						}
						else if (LEDBlink == 1)
						{
							HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,RESET);
							HAL_UART_Transmit(&huart2, (uint8_t*) "LED OFF\r\n", strlen("LED OFF\r\n"),1000);
							LEDBlink = 0;
						}
						STATE = State_LEDMenu_Wait;
						break;
					case 'x':
						STATE = State_Mainmenu_Print;
						break;
					default:
						HAL_UART_Transmit(&huart2, (uint8_t*) "unidentified input\r\n", strlen("unidentified input\r\n"),1000);
						STATE = State_LEDMenu_Print;
						break;
				}
			}
			break;


			case State_ButtonMenu_Print:

						HAL_UART_Transmit(&huart2, (uint8_t*) "x.back\r\n", strlen("x.back\r\n"),
								100);
					STATE = State_ButtonMenu_Wait;
				break;

			case State_ButtonMenu_Wait:

				if(inputchar != -1){
					switch (inputchar)
					{
					case 0:
						break;
					case 'x':
						STATE = State_Mainmenu_Print;
						break;
					default:
						HAL_UART_Transmit(&huart2, (uint8_t*) "unidentified input\r\n", strlen("unidentified input\r\n"),1000);
						STATE = State_ButtonMenu_Print;
						break;
					}
				}
				if(Interrupt ==1)
				{
					if(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == 1)
					{
						char stateB[]="Status of button:Unpress\r\n";
						HAL_UART_Transmit(&huart2, (uint8_t*)stateB, strlen(stateB),1000);
					}
					else
					{
						char stateB[]="Status of button:Press\r\n";
						HAL_UART_Transmit(&huart2, (uint8_t*)stateB, strlen(stateB),1000);
					}
					Interrupt = 0;
				}
				break;
		}

		if (LEDBlink == 1)
		{
			if (HAL_GetTick() - TimeStamp >= LED_HalfPeriod)

				{
					HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
					TimeStamp = HAL_GetTick();

				}
		}

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
int16_t UARTRecieveIT() {
	//store data last position
	static uint32_t dataPos = 0;

	//create dummy data
	int16_t data = -1;

	//check pos in buffer vs last position
	if (huart2.RxXferSize - huart2.RxXferCount != dataPos)
			{
		//read data from buffer
		data = RxDataBuffer[dataPos];
		//move to next pos
		dataPos = (dataPos + 1) % huart2.RxXferSize;
	}
	return data;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_13)
	{
			Interrupt = 1;
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
	while (1) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
