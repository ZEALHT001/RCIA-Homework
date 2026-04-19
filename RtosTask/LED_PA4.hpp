#ifndef LED_PA4_HPP   
#define LED_PA4_HPP

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "freertos.h"
#include "cmsis_os.h"
#include "../Function/Input_PA2.hpp"

void LED_PA4(void *argument);

#ifdef __cplusplus
}
#endif



#endif