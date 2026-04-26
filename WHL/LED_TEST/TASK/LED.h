#ifndef __LED_H
#define __LED_H

#include "stm32f1xx_hal.h"

typedef struct{

    GPIO_TypeDef * GPIOx;
    uint16_t GPIO_PIN;
    uint8_t  LED_State; //状态set/reset
    uint8_t Control_mode; //0/1对应开漏/推挽

} BASE_HandleTypeDef;

/*
** * @brief  LED初始化函数
 * @param  hled: LED句柄
 * @param  GPIOx: GPIO端口
 * @param  GPIO_PIN: GPIO引脚
 * @param  ActiveLevel: LED的有效电平（0或1）
*/
void LED_init(BASE_HandleTypeDef *hled, GPIO_TypeDef* GPIOx, uint16_t GPIO_PIN, uint8_t ActiveLevel, uint8_t Control_mode);
/*
** * @brief  LED点亮函数
 * @param  hled: LED句柄
*/
void LED_On(BASE_HandleTypeDef*hled);
 
/*
** * @brief  按键控制函数
 * @param  hled: LED句柄
*/
void Key_Control(BASE_HandleTypeDef*hled);
/**
** * @brief  LED控制函数
 * @param  hled: LED句柄
*/
void led_control(BASE_HandleTypeDef *hled);

#endif