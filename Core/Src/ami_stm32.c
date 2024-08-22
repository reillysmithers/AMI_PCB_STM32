/*
 * ami_stm32.c
 *
 *  Created on: Aug 21, 2024
 *  Author: reillysmithers
 */

#include "stm32f4xx_hal.h"

int encoder_position = 0;

//GPIO Pins for leds in index order (so LED 1 equals index 0)
GPIO_TypeDef* GPIO_Ports[] = {GPIOC, GPIOB, GPIOB, GPIOB, GPIOC, GPIOC, GPIOA};
uint16_t GPIO_Pins[] = {GPIO_PIN_13, GPIO_PIN_9, GPIO_PIN_8, GPIO_PIN_6, GPIO_PIN_11, GPIO_PIN_10, GPIO_PIN_10};

//For LED pulsing case
int led_pulsing_idx = 0; // Which pin to pulse LED on?
int led_pulsing_flag = 0; // Are we pulsing an LED (0 - no, 1 - slow, 2 - fast)

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

void ami_main(void) {
	//First check if we have pushed the button
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


