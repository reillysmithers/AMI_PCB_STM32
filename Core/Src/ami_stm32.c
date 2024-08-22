/*
 * ami_stm32.c
 *
 *  Created on: Aug 21, 2024
 *  Author: reillysmithers
 */

#include "stm32f4xx_hal.h"

encoder_position = 0;

HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

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
//1
//2 - Flash slow (Selection mode)
//3 - flash fast (Pending Jetson Confirmation)

void switchLED(int ledId, int state) {
    GPIO_TypeDef* GPIO_Ports[] = {GPIOC, GPIOB, GPIOB, GPIOB, GPIOC, GPIOC, GPIOA};
    uint16_t GPIO_Pins[] = {GPIO_PIN_13, GPIO_PIN_9, GPIO_PIN_8, GPIO_PIN_6, GPIO_PIN_11, GPIO_PIN_10, GPIO_PIN_10};

    if (ledId >= 1 && ledId <= 7) {
        HAL_GPIO_WritePin(GPIO_Ports[ledId - 1], GPIO_Pins[ledId - 1], state == 1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}


