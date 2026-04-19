#include "LED_TASK.h"
#include "main.h"           
#include "cmsis_os.h"

LED_t leds[3] = {
  {LED_0_GPIO_Port, GPIO_PIN_0, NULL, 500},
  {LED_1_GPIO_Port, GPIO_PIN_1, NULL, 1000},
  {LED_2_GPIO_Port, GPIO_PIN_2, NULL, 800}
};

void LED_On(LED_t* led) {
  HAL_GPIO_WritePin(led->port, led->pin, GPIO_PIN_RESET);
}
void LED_Off(LED_t* led) {
  HAL_GPIO_WritePin(led->port, led->pin, GPIO_PIN_SET);
}
void LED_Toggle(LED_t* led) {
  HAL_GPIO_TogglePin(led->port, led->pin);
}

void LED_Task(void* argument) {
  LED_t* led = (LED_t*)argument;
  for(;;) {
    LED_Toggle(led);
    osDelay(led->delay);
  }
}
static const char* ledTaskNames[] = {"led0Task", "led1Task", "led2Task"};
void LED_Task_Init(void) {
for(int i = 0; i < 3; i++) {
  const osThreadAttr_t ledTask_attributes = {
    .name = ledTaskNames[i],
    .stack_size = 128 * 4,
    .priority = (osPriority_t) osPriorityNormal,
  };
  leds[i].handle = osThreadNew(LED_Task, &leds[i], &ledTask_attributes);
}
}