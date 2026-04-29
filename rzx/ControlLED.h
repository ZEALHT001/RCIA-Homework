#include "main.h"


typedef struct{
	GPIO_TypeDef* port;
	uint16_t pin;
	GPIO_PinState light_level;      // ��/�͵�ƽ����
}LED_t;

void LED_Control(const LED_t* Led, uint8_t On);
void LED_ALL_On();
void LED_ALL_Off();