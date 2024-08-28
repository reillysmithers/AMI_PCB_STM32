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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int encoder_position = 0; //Encoder position
int encoder_zero = 0; //Position of encoder when we toggle (the new zero point)

//GPIO Pins for leds in index order (so LED 1 equals index 0)
GPIO_TypeDef *GPIO_Ports[] = { GPIOC, GPIOB, GPIOB, GPIOB, GPIOC, GPIOC, GPIOA };
uint16_t GPIO_Pins[] = { GPIO_PIN_13, GPIO_PIN_9, GPIO_PIN_8, GPIO_PIN_6,
GPIO_PIN_11, GPIO_PIN_10, GPIO_PIN_10 };

//For LED pulsing case
int led_cursor_idx = 1; // Which pin to pulse LED on?
int led_cursor_flag = 0; // Are we pulsing an LED (0 - no, 1 - slow, 2 - fast)

//Modes
int selection_mode = 0;
int selection_allowed = 1;

//Current led index (index of solid LED, must start at 1)
int led_idx = 1;

//Last interrupt time for debouncing button
volatile uint32_t last_interrupt_time = 0;

//The current mission selected, only the Jetson can update this
uint8_t missionSelected = 1;

//The current AS state, only other devices on the CAN bus can update this
uint8_t globalASState = 0;

int jetson_wait_flag = 0;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void updateEncoders(void);
void switchLED(int ledId, int state);
void sendMission(int mission);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	MX_CAN1_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */
	//Set systick priority
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING)
			!= HAL_OK) {
		Error_Handler();
	}

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	// CAN message structure
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t TxMailbox;
	uint8_t TxData[8];

	TxHeader.DLC = 8;  // Data length: 8 bytes
	TxHeader.ExtId = 0x01;  // Example extended ID
	TxHeader.IDE = CAN_ID_EXT;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.TransmitGlobalTime = DISABLE;

	// Example data to send
	TxData[0] = 0xDE;
	TxData[1] = 0xAD;
	TxData[2] = 0xBE;
	TxData[3] = 0xEF;
	TxData[4] = 0x12;
	TxData[5] = 0x34;
	TxData[6] = 0x56;
	TxData[7] = 0x78;

	// Transmit the message
	if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
		// Transmission error
		Error_Handler();
	}

	while (1) {
		//Update encoders
		updateEncoders();
		//Check if we are selecting
		if (selection_mode == 1 && jetson_wait_flag == 0) {
			//We are using a cursor
			led_cursor_flag = 1;

			//Set cursor index
			int normalised_encoder_pos = encoder_position - encoder_zero;
			if (normalised_encoder_pos < 10) {
				led_cursor_idx = 1;
				//Stop the encoder from spinning crazy one way
				//and taking ages to recover
				if (normalised_encoder_pos < 0) {
					encoder_zero = encoder_position;
				}
			} else if (normalised_encoder_pos < 20) {
				led_cursor_idx = 2;
			} else if (normalised_encoder_pos < 30) {
				led_cursor_idx = 3;
			} else if (normalised_encoder_pos < 40) {
				led_cursor_idx = 4;
			} else if (normalised_encoder_pos < 50) {
				led_cursor_idx = 5;
			} else if (normalised_encoder_pos < 60) {
				led_cursor_idx = 6;
			} else {
				led_cursor_idx = 7;
				if (normalised_encoder_pos > 70) {
					encoder_zero = encoder_position - 70;
				}
			}

			//So long as the selected LED is not the cursor, turn it on solid
			if (led_idx != led_cursor_idx) {
				switchLED(led_idx, 1);
			}

			//Turn everything but the cursor and selected led
			for (int i = 1; i <= 7; i++) {
				if (i != led_idx && i != led_cursor_idx) {
					switchLED(i, 0);
				}
			}
		} else if (jetson_wait_flag == 0) {
			//Turn off cursor
			led_cursor_flag = 0;
			//Turn the current LED on solid
			switchLED(led_idx, 1);
			//Turn everything else off
			for (int i = 1; i <= 7; i++) {
				if (i != led_idx) {
					switchLED(i, 0);
				}
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
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 80;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief CAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN1_Init(void) {

	/* USER CODE BEGIN CAN1_Init 0 */

	/* USER CODE END CAN1_Init 0 */

	/* USER CODE BEGIN CAN1_Init 1 */

	/* USER CODE END CAN1_Init 1 */
	hcan1.Instance = CAN1;
	hcan1.Init.Prescaler = 4;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_8TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
	hcan1.Init.TimeTriggeredMode = DISABLE;
	hcan1.Init.AutoBusOff = DISABLE;
	hcan1.Init.AutoWakeUp = DISABLE;
	hcan1.Init.AutoRetransmission = ENABLE;
	hcan1.Init.ReceiveFifoLocked = DISABLE;
	hcan1.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CAN1_Init 2 */

	/* USER CODE END CAN1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 8000;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 1400;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */
	// Start timer 2 (LED Pulse timer)
	HAL_TIM_Base_Start_IT(&htim2);
	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */
	// Start timer (encoder mode)
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 | GPIO_PIN_10 | GPIO_PIN_11,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_8 | GPIO_PIN_9,
			GPIO_PIN_RESET);

	/*Configure GPIO pins : PC13 PC10 PC11 */
	GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_10 | GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PA10 */
	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB6 PB8 PB9 */
	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PB7 */
	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	CAN_RxHeaderTypeDef rxHeader;
	uint8_t rxData[8];
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK) {
		Error_Handler();
	}
	while (1) {
		switchLED(6, 1);
	}
	if ((rxHeader.StdId == 1298)) {
		// Read in Mission_Selected
		missionSelected = (rxData[0] >> 3) & ((1 << 3) - 1);
	} else if (rxHeader.StdId == 1282) {
		// Read in AS_State
		globalASState = (rxData[0] >> 0) & ((1 << 3) - 1);
	}
}

//Timer callback
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	// Check which version of the timer triggered this callback and toggle LED
	if (htim == &htim2) {
		//Check if we are using a cursor (at speed 1, the slower speed, trigged on Timer 2)
		if (led_cursor_flag == 1) {
			//Toggle the cursor LED on/off
			HAL_GPIO_TogglePin(GPIO_Ports[led_cursor_idx - 1],
					GPIO_Pins[led_cursor_idx - 1]);
		}
	}
}

//Button press callback, theres only one of these so we don't need to check pin

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	//Toggle mode if allowed too or already in mode
	if (HAL_GetTick() - last_interrupt_time > 200) {
		last_interrupt_time = HAL_GetTick();
		if (selection_allowed == 1 || selection_mode == 1) {
			//Toggle selection mode
			if (selection_mode == 1) {
				//We are turning off selection mode
				//Send mission to Jetson
				sendMission(led_cursor_idx);
				//Wait until mission selected is confirmed
				jetson_wait_flag = 1;
				led_cursor_flag = 0;
				while (missionSelected != led_cursor_idx) {
					HAL_GPIO_TogglePin(GPIO_Ports[led_cursor_idx - 1],
							GPIO_Pins[led_cursor_idx - 1]);
					HAL_Delay(50);
				}
				jetson_wait_flag = 0;
				selection_mode = 0; //Turn off selection mode
				led_idx = led_cursor_idx; //Lock in the new solid LED index

			} else {
				//We are turning on selection mode
				//Zero out encoder (taking into consideration current LED index)
				encoder_zero = encoder_position - 10 * (led_idx - 1);
				selection_mode = 1;
			}
		}
	}
}

void CAN1_FilterConfig(void) {
	CAN_FilterTypeDef sFilterConfig;

	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
		// Filter configuration error
		Error_Handler();
	}
}

//Reads latest encoder values
void updateEncoders(void) {
	//Deal with unsigned int wraparound, it would be
	//genuinely impressive for monkey to spin this thing high
	//enough for me to care
	if (TIM3->CNT > 32768) {
		encoder_position = TIM3->CNT - 65535;
	} else {
		encoder_position = TIM3->CNT;
	}
}

//Switch led taking index 1-7 and state 0 or 1
void switchLED(int ledId, int state) {
	if (ledId >= 1 && ledId <= 7) {
		HAL_GPIO_WritePin(GPIO_Ports[ledId - 1], GPIO_Pins[ledId - 1],
				state == 1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
	}
}

// Function to send the mission data over CAN
void sendMission(int mission) {
	// Ensure the mission value fits within 3 bits (0 to 7)
	uint8_t selected_mission = mission & 0x07; // Mask to 3 bits

	// Create a CAN message
	CAN_TxHeaderTypeDef TxHeader;
	uint8_t TxData[1];  // Message length is 1 byte
	uint32_t TxMailbox;

	// Configure the CAN message ID and DLC
	TxHeader.StdId = 0x0510;        // Standard ID (message ID from DBC)
	TxHeader.ExtId = 0x00;          // Extended ID not used
	TxHeader.IDE = CAN_ID_STD;      // Standard identifier
	TxHeader.RTR = CAN_RTR_DATA;    // Data frame
	TxHeader.DLC = 1;               // Data length code (1 byte)
	TxHeader.TransmitGlobalTime = DISABLE;

	// Prepare the data to send
	TxData[0] = selected_mission;

	// Send the CAN message
	if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK) {

		// Transmission error
		Error_Handler();
	} else {
		while (1) {
			switchLED(5, 1);
		}

	}
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	switchLED(7, 1);
	jetson_wait_flag = 1;
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
