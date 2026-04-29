
#include "led.h"

void LED_Open(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
HAL_GPIO_WritePin(GPIOx,GPIO_Pin,GPIO_PIN_SET);
}


void LED_Close(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin)
{

HAL_GPIO_WritePin(GPIOx,GPIO_Pin,GPIO_PIN_RESET);

}



void Judge(int P,GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin)
{

if(P==0)
{

    LED_Open(GPIOx,GPIO_Pin);

}

else
{
   LED_Close(GPIOx,GPIO_Pin);
}

}






