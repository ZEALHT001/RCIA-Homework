#include "LED_PA3.hpp"

extern "C" void LED_PA3(void *argument)
{
  for(;;)
  {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
    osDelay(500);
  }
}
    