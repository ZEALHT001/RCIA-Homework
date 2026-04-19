#include "ControlLED.h"
#include "main.h"

void LED_Control(const LED_t* Led, uint8_t On){
	if(!Led) return;//防止空指针访问
	
	 //  三目运算符：条件 ? 真值 : 假值 （嵌套）
	GPIO_PinState target = (On) ? Led -> light_level : 
	(Led->light_level == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET);
	HAL_GPIO_WritePin(Led->GPIO_Port, Led->GPIO_Pin, target);
}

void LED_ALL_On(void){
	for(int i = 0; i < 3; i++){
		LED_Control(&leds[i], 1);
	}
}

void LED_ALL_Off(void){
	for(int i = 0; i < 3; i++){
		LED_Control(&leds[i], 0);
	}
}