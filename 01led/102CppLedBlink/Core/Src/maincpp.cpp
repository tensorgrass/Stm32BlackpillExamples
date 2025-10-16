/*
 * maincpp.cpp
 *
 *  Created on: May 24, 2025
 *      Author: froilan
 */

#include "maincpp.hpp"
#include "main.h"

void usercode2()
{
    while( 1 )
    {
    	for (int i = 0; i < 4; i++) {
			HAL_GPIO_TogglePin(GPIOC, LED_GREEN_Pin);
			HAL_Delay(200);
	    	HAL_GPIO_TogglePin(GPIOC, LED_GREEN_Pin);
	    	HAL_Delay(200);
    	}
    	HAL_GPIO_TogglePin(GPIOC, LED_GREEN_Pin);
    	HAL_Delay(500);
    	HAL_GPIO_TogglePin(GPIOC, LED_GREEN_Pin);
    	HAL_Delay(200);
    }
}
