#include "LED.h"

void Led_Control(led_t *led,GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin,uint8_t GPIO_Mode)
{
    led->GPIOx = GPIOx;
    led->GPIO_Pin = GPIO_Pin;
    if(GPIO_Mode == 1)
    {
        HAL_GPIO_WritePin(led->GPIOx, led->GPIO_Pin, GPIO_PIN_SET);
    }
    else if(GPIO_Mode == 0)
    {
        HAL_GPIO_WritePin(led->GPIOx, led->GPIO_Pin, GPIO_PIN_RESET);
    }
}