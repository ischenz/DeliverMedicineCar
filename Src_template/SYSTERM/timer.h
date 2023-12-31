#ifndef __TIMER_H
#define __TIMER_H

#include "sys.h"

extern volatile uint8_t timeout;
extern float my_yaw;
void Init_Timer3(void);
void Init_Timer4(void);
void Timer1_PWM_GPIO_Init(uint16_t Psc, uint16_t Per);
void Timer6_init(void);
void Timer7_init(void);
void set_time(uint16_t time_ms);

#endif
