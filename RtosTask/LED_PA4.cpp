#include "LED_PA4.hpp"

extern "C" void LED_PA4(void *argument)
{
  for(;;)
  {
    Input_PA2();
    osDelay(1000);
  }
}