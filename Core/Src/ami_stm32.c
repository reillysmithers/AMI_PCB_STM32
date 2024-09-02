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
int led_cursor_idx = 1; // Which pin to pulse LED on?
int led_cursor_flag = 0; // Are we pulsing an LED (0 - no, 1 - slow, 2 - fast)

//Modes
int selection_mode = 0;
int selection_allowed = 1;

//Current led index (index of solid LED, must start at 1)
int led_idx = 1;

void updateEncoders(void);
void switchLED(int ledId, int state);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

//The main loop
void ami_update(void) {
	//Update encoders
	updateEncoders();
	//Check if we are selecting
	if (selection_mode == 1) {
		//We are using a cursor
		led_cursor_flag = 1;

		//Set cursor index
		int normalised_encoder_pos = encoder_position - encoder_zero;
		if (normalised_encoder_pos < 20) {
			led_cursor_idx = 1;
		} else if (normalised_encoder_pos < 40) {
			led_cursor_idx = 2;
		} else if (normalised_encoder_pos < 60) {
			led_cursor_idx = 3;
		} else if (normalised_encoder_pos < 80) {
			led_cursor_idx = 4;
		} else if (normalised_encoder_pos < 100) {
			led_cursor_idx = 5;
		} else if (normalised_encoder_pos < 120) {
			led_cursor_idx = 6;
		} else {
			led_cursor_idx = 7;
		}

		//So long as the selected LED is not the cursor, turn it on solid
		if (led_idx != led_cursor_idx) {
			switchLED(led_idx,1);
		}

		//Turn everything but the cursor and selected led
		for (int i = 1; i <= 7; i++) {
			if (i != led_idx && i != led_cursor_idx) {
				switchLED(i,0);
			}
		}
	}
	else {
		//Turn off cursor
		led_cursor_flag = 0;
		//Turn the current LED on solid
		switchLED(led_idx,1);
		//Turn everything else off
		for (int i = 1; i <= 7; i++) {
			if (i != led_idx) {
				switchLED(i,0);
			}
		}
	}
}

//Timer callback
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback and toggle LED
  if (htim == &htim2)
  {
	//Check if we are using a cursor (at speed 1, the slower speed, trigged on Timer 2)
	if (led_cursor_flag == 1) {
		//Toggle the cursor LED on/off
		HAL_GPIO_TogglePin(GPIO_Ports[led_cursor_idx-1], GPIO_Pins[led_cursor_idx-1]);
	}
  }
}

//Button press callback, theres only one of these so we don't need to check pin
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// Debounce delay
	HAL_Delay(30);
	// Check if button is still pressed (optional)
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET) // Replace GPIOx and GPIO_PIN with actual values
	{
		//Toggle mode if allowed too or already in mode
		if (selection_allowed == 1 || selection_mode == 1) {
			//Toggle selection mode
			if (selection_mode == 1) {
				//We are turning off selection mode
				//Fast blink (add this)
				//Wait for Jetson (add this)
				led_idx = led_cursor_idx; //Lock in the new solid LED index
				selection_mode = 0; //Turn off selection mode
			}
			else {
				//We are turning on selection mode
				//Zero out encoder (taking into consideration current LED index)
				encoder_zero = encoder_position - 20*(led_idx-1);
				selection_mode = 1;
			}

		}
	}
}

//Reads latest encoder values
void updateEncoders(void)
{
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
void switchLED(int ledId, int state)
{
	if (ledId >= 1 && ledId <= 7) {
        HAL_GPIO_WritePin(GPIO_Ports[ledId - 1], GPIO_Pins[ledId - 1], state == 1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}


