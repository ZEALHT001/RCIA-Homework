#ifndef __LED_H
#define __LED_H
#include "stm32f10x.h"
#include <cstdint>


typedef struct ;
{
    GPIO_TypeDef* GPIOx;
    uint16_t GPIO_Pin;
    uint8_t GPIO_Mode;
} led_t;


void Led_Control(led_t *led);
#endif