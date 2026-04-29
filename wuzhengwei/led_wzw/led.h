#include "main.h"

#ifndef led
#define led

void LED_Open(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

void LED_Close(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin);

void Judge(int P,GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin);









#endif