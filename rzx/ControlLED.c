#include "ControlLED.h"
#include "main.h"

void LED_Control(const LED_t* Led, uint8_t On){
	if(!Led) return;//
	
	 //  
	GPIO_PinState target = (On) ? Led -> light_level : 
	(Led->light_level == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET);
	
}

void LED_ALL_On(){
	for(int i = 0; i < 3; i++){
		LED_Control(&leds[i], 1);
	}
}
void LED_ALL_Off(){
	for(int i = 0; i < 3; i++){
		LED_Control(&leds[i], 0);
	}
}