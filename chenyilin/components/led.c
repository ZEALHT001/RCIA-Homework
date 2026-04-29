#include "led.h"

void LED_BLUE(void)
{ 
  HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
}

void LED_RED(void)
{ 
  HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
}

void LED_GREEN(void)
{ 
  HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
}