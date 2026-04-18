#include "LED_PA5.hpp"
extern "C" void LED_PA5(void *argument)
{
  for(;;)
  {
    PA5();
    osDelay(1000);
  }
}