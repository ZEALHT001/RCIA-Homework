#include "LED.h"


void LEDinit(BASE_HandleTypeDef *hled, GPIO_TypeDef* GPIOx, uint16_t GPIO_PIN, uint8_t ActiveLevel, uint8_t CONTORL_form){

 hled -> GPIOx = GPIOx;
 hled -> GPIO_PIN = GPIO_PIN;
 hled -> ActiveLevel =  ActiveLevel; 
 hled -> CONTORL_form = CONTORL_form;
}

void LED_ON (BASE_HandleTypeDef *hled){
if(hled->CONTORL_form == 0){
 HAL_GPIO_WritePin(hled->GPIOx, hled-> GPIO_PIN,(hled->ActiveLevel == 0) ? GPIO_PIN_SET : GPIO_PIN_RESET) ;
}else{
 HAL_GPIO_WritePin(hled->GPIOx, hled-> GPIO_PIN,(hled->ActiveLevel == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET) ;
}

}

void KEY_CONTROL(BASE_HandleTypeDef*hled){
// 读取引脚的电平状态
GPIO_PinState pin_state = HAL_GPIO_ReadPin(hled->GPIOx, hled-> GPIO_PIN);

if (pin_state == GPIO_PIN_SET) {
   hled -> ActiveLevel =  1;
}else {
   hled -> ActiveLevel =  0;
}

}

