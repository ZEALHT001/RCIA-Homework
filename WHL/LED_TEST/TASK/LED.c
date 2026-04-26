#include "LED.h"


void LED_init(BASE_HandleTypeDef *hled, GPIO_TypeDef* GPIOx, uint16_t GPIO_PIN, uint8_t LED_State, uint8_t Control_mode){

 hled -> GPIOx = GPIOx;
 hled -> GPIO_PIN = GPIO_PIN;
 hled -> LED_State =  LED_State; 
 hled -> Control_mode = Control_mode;
}

void LED_On (BASE_HandleTypeDef *hled){
   if (hled == NULL) return;
if(hled->Control_mode == 0){
 HAL_GPIO_WritePin(hled->GPIOx, hled-> GPIO_PIN,(hled->LED_State == 0) ? GPIO_PIN_SET : GPIO_PIN_RESET) ;
}else{
 HAL_GPIO_WritePin(hled->GPIOx, hled-> GPIO_PIN,(hled->LED_State  == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET) ;
}

}

void Key_Control(BASE_HandleTypeDef*hled){
   if (hled == NULL) return;
// 读取引脚的电平状态
GPIO_PinState pin_state = HAL_GPIO_ReadPin(hled->GPIOx, hled-> GPIO_PIN);

if (pin_state == GPIO_PIN_SET) {
   hled -> LED_State =  1;
}else {
   hled -> LED_State =  0;
}

}

