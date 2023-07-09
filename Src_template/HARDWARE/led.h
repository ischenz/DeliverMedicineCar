#ifndef __LED_H
#define __LED_H

#include "sys.h"

//LED端口定义
#define LED1 PFout(9)
#define LED2 PFout(10)

#define set_green_led(pwm) (TIM_SetCompare3(TIM1 ,pwm))
#define set_blue_led(pwm)  (TIM_SetCompare2(TIM1 ,pwm))

void LED_Init(void);//初始化	
void set_led_yellow(void);
void led_off(void);

#endif
