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
#include "string.h"
#include "math.h"
#include "stdio.h"
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
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_TIM3_Init(void);
void MX_TIM4_Init(void);
void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t receive_data[25];
uint8_t send_data[16];
//bien Encoder
int16_t encoder1 =0 ,encoder2 = 0,encoder3 = 0;
int16_t encoder1_prv = 0, encoder2_prv = 0, encoder3_prv = 0;
int16_t e1_prv = 0, e2_prv = 0, e3_prv = 0;
int16_t denta = 0;
int16_t vantoc = 0;
//bien goc quay , xung quay
double goc_quay1 = 0, goc_quay2 = 0, goc_quay3 = 0;
double set_point1 = 0, set_point2 = 0, set_point3 = 0;
int16_t xung_quay1 = 0, xung_quay2 = 0, xung_quay3 = 0;
//dong co 1
int16_t e1 = 0, tong_e1 = 0;
double P1 = 0, I1 = 0, D1 = 0;
double duty1 = 0;
double kp1 =1.2, ki1 = 0, kd1 = 0.03;
//dong co 2
int16_t e2 = 0, tong_e2 = 0;
double P2 = 0, I2 = 0, D2 = 0;
double duty2 = 0;
double kp2 = 1.3, ki2 = 0, kd2 = 0.03;
//dong co 3
int16_t e3 = 0, tong_e3 = 0;
double P3 = 0, I3 = 0, D3 = 0;
double duty3 = 0;
double kp3 = 1.5, ki3 = 0, kd3 = 0.05;

int16_t t_truyen = 0;
int16_t t_nhan = 0;
int16_t t_kich = 0;
int16_t temp = 0;
void Receive_data(void);
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
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */


  HAL_TIM_Base_Start_IT(&htim3);

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);


  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
   __HAL_UART_ENABLE_IT(&huart2, UART_IT_TC);

   HAL_UART_Receive_IT(&huart2, receive_data, 22);

    receive_data[0] = 'A';			//A la goc theta1

     receive_data[1] = '0';			//0la +, 1 la-

     receive_data[2] = '0';
     receive_data[3] = '0';
     receive_data[4] = '0';
     receive_data[5] = '0';
     receive_data[6] = '0';

     receive_data[7] = 'B';			//B goc theta2

     receive_data[8] = '0';			//0la +, 1 la-

     receive_data[9] = '0';
     receive_data[10] = '0';
     receive_data[11] = '0';
     receive_data[12] = '0';
     receive_data[13] = '0';

     receive_data[14] = 'C';			//C goc theta3

     receive_data[15] = '0';			//0la +, 1 la-

     receive_data[16] = '0';
     receive_data[17] = '0';
     receive_data[18] = '0';
     receive_data[19] = '0';
     receive_data[20] = '0';

     receive_data[21] = 'N';			//chuoi he thong khi nen
     receive_data[22] = '0';
     receive_data[23] = '0';
     receive_data[24] = '0';

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  /////tinh xung cua cac dong co
	  	  	  xung_quay1 = goc_quay1/360;		//xung quay 1
//	  	  	  xung_quay1 = set_point1*998.4/360;
	  	  	  xung_quay1 = round(xung_quay1);

	  	  	  xung_quay2 = goc_quay2/360;		//xung quay 2
//	  	  	  xung_quay2 = set_point1*998.4/360;
	  	  	  xung_quay2 = round(xung_quay2);

	  	  	  xung_quay3 = goc_quay3/360;		//xung quay3
//	  	  	  xung_quay3 = set_point3*998.4/360;
	  	  	  xung_quay3 = round(xung_quay3);

	 t_nhan++;
	 if(t_nhan > 1000)
	 {
		 HAL_UART_Receive_IT(&huart2, receive_data, 25);
		 t_nhan = 0;
	 }
	 else if(receive_data[21] == 'E' || receive_data[21] == 'H' || receive_data[21] == 'S')
	 {
		 Receive_data();
	 }
	 else
	 {
		 if(receive_data[22] == '0' && receive_data[23] == '0' && receive_data[24] == '0')
		 {
			 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
			 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
			 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
		 }
		 if(receive_data[22]!='0')
		 {
			 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
		 }
		 else HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);

		 if(receive_data[23]!='0')
		 {
			 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
		 }
		 else HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);

		 if(receive_data[24]!='0')
		 {
			 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
		 }
		 else HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
	 }
  }
  /* USER CODE END 3 */
}



void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	if(&huart->Instance == &huart2.Instance)
	{
		HAL_UART_Receive_IT(&huart2, receive_data, 25);
	}
}
void Receive_data()
{
	goc_quay1 = (double)((receive_data[2] - 48)*10000 + (receive_data[3] - 48)*1000 +
			(receive_data[4] - 48)*100 + (receive_data[5] - 48)*10 + (receive_data[6] - 48))/100.0;
	goc_quay2 = (double)((receive_data[9] - 48)*10000 + (receive_data[10] - 48)*1000 +
			(receive_data[11] - 48)*100 + (receive_data[12] - 48)*10 + (receive_data[13] - 48))/100.0;
	goc_quay3 = (double)((receive_data[16] - 48)*10000 + (receive_data[17] - 48)*1000 +
				(receive_data[18] - 48)*100 + (receive_data[19] - 48)*10 + (receive_data[20] - 48))/100.0;
	if((receive_data[1]-48) == 1) goc_quay1 = -goc_quay1;
	if((receive_data[8]-48) == 1) goc_quay2 = -goc_quay2;
	if((receive_data[15]-48) == 1) goc_quay3 = -goc_quay3;
}
///A0abcdeB0abcdeC0abcdeE000
///E/H/S->dong co				N-> khi nen		000: tat het,100: bat bom 110: hut 101:tha

/////dong co: E -> end vi tri dong co		H-> home	S-> send data	vi tri dong co
/// khi nen: G-> gap 	T-> tha  B-> kich bom	-> O:tat bom	1-> valve 1		2-> valve 2

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == htim3.Instance)
	{
		/*Encoder read*/
		/*Set Home*/
		if(receive_data[21] == 'H')
		{
			if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0) == 0)
			{

				__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1,0);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
			}

			else
			{
				__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1,25);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
			}

			if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1) == 0)
			{

				__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2,0);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
			}

			else
			{
				__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2,25);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
			}

			if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3) == 0)
			{

				__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3,0);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
			}

			else
			{
				__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3,25);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
			}
		}
		//PID vitri
		else
		{
			if((receive_data[1] - 48) == '1')
			{
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
			}
			else
			{
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
			}
			__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1, 25);

			if((receive_data[8] - 48) == '1')
			{
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
			}
			else
			{
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
			}
			__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_2, 25);

			if((receive_data[15] - 48) == '1')
			{
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
			}
			else
			{
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
			}
			__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3, 25);

		}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 24000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 24;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 99;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
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
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC6 PC7 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA9 PA10 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 PD3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
