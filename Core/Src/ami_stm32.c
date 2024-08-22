/*
 * ami_stm32.c
 *
 *  Created on: Aug 21, 2024
 *  Author: reillysmithers
 */

#include "stm32f4xx_hal.h"

int encoder_position = 0; //Encoder position
int encoder_zero = 0; //Position of encoder when we toggle (the new zero point)

//GPIO Pins for leds in index order (so LED 1 equals index 0)
GPIO_TypeDef* GPIO_Ports[] = {GPIOC, GPIOB, GPIOB, GPIOB, GPIOC, GPIOC, GPIOA};
uint16_t GPIO_Pins[] = {GPIO_PIN_13, GPIO_PIN_9, GPIO_PIN_8, GPIO_PIN_6, GPIO_PIN_11, GPIO_PIN_10, GPIO_PIN_10};

//For LED pulsing case
int led_pulsing_idx = 0; // Which pin to pulse LED on?
int led_pulsing_flag = 0; // Are we pulsing an LED (0 - no, 1 - slow, 2 - fast)

//Modes
int selection_mode = 0;
int selection_allowed = 1;

//The main loop
void ami_main(void) {
	//Check if we are selecting
	while (1) {
		if (selection_mode == 1) {
			led_pulsing_flag = 1;

		}
	}
}

//Timer callback
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback and toggle LED
  if (htim == &htim2)
  {
	//Check if we are pulsing (at speed 1, the slower speed, trigged on Timer 2)
	if (led_pulsing_flag == 1) {
		//Toggle the LED on/off
		HAL_GPIO_TogglePin(GPIO_Ports[led_pulsing_idx-1], GPIO_Pins[led_pulsing_idx-1]);
	}
  }
}

//Button press callback, theres only one of these so we don't need to check pin
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// Debounce delay
	HAL_Delay(50);
	// Check if button is still pressed (optional)
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET) // Replace GPIOx and GPIO_PIN with actual values
	{
		//Toggle mode if allowed too or already in mode
		if (selection_allowed == 1 || selection_mode == 1) {
			//Toggle selection mode and zero out encoder
			selection_mode = !selection_mode;
			encoder_zero = encoder_position;
		}
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

//LED states
//0 - Off
//1 - On
//2 - Flash slow (Selection mode)
//3 - flash fast (Pending Jetson Confirmation)
void switchLED(int ledId, int state) {
	if (state > 1) {
		led_pulsing_idx = ledId;
		led_pulsing_flag = 0;
	}
	else if (ledId >= 1 && ledId <= 7) {
        HAL_GPIO_WritePin(GPIO_Ports[ledId - 1], GPIO_Pins[ledId - 1], state == 1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}


