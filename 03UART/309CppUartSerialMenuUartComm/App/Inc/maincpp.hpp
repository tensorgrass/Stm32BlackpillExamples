/*
 * maincpp.hpp
 *
 *  Created on: May 24, 2025
 *      Author: froilan
 */

#ifndef INC_MAINCPP_HPP_
#define INC_MAINCPP_HPP_

#ifdef __cplusplus
extern "C" {
#endif

#include <stm32f4xx_hal.h>

void main_cpp(UART_HandleTypeDef *huart1, UART_HandleTypeDef *huart2);
void HAL_UART_RxCpltCallback_cpp(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* INC_MAINCPP_HPP_ */
