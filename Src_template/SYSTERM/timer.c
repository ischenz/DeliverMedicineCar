#include "timer.h"
#include "led.h"
#include "usart.h"
#include "oled.h"
#include "pid.h"
#include "mpu6050.h"
#include "motor.h"
#include "control.h"

volatile uint8_t timeout = 0;
float my_yaw;

void Timer1_PWM_GPIO_Init(uint16_t Psc, uint16_t Per)
{
	GPIO_InitTypeDef GPIO_InitStructure = {0};
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct = {0};
	TIM_OCInitTypeDef TIM_OCInitStruct = {0};
	//开启时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA ,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//使能GPIOE时钟	
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_TIM1); //GPIOA11复用为定时器1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource8, GPIO_AF_TIM1); 
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource11,GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource13,GPIO_AF_TIM1); 	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_11;  //TIM1_CH1  TIM1_CH4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_13;  //TIM1_CH2  TIM1_CH3
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	//GPIO_SetBits(GPIOE,GPIO_Pin_15);
	
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);//初始化定时器。
	TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period=Per;
	TIM_TimeBaseInitStruct.TIM_Prescaler=Psc;
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStruct);
	
	TIM_OCInitStruct.TIM_OCMode=TIM_OCMode_PWM1;//初始化输出比较,TIMx_CNT<TIM_CCRx的时候为有效电平
	TIM_OCInitStruct.TIM_OCPolarity=TIM_OCPolarity_High;//有效电平为高
	TIM_OCInitStruct.TIM_OutputState=TIM_OutputState_Enable;
	//TIM_OCInitStruct.TIM_Pulse=0;//对应CCR的值 比较值
	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);//ENABLE//OC1预装载寄存器使能
	TIM_OC4PreloadConfig(TIM1,TIM_OCPreload_Enable);//ENABLE//OC4预装载寄存器使能	
	TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);//ENABLE//OC2预装载寄存器使能
	TIM_OC3PreloadConfig(TIM1,TIM_OCPreload_Enable);//ENABLE//OC3预装载寄存器使能	
	
	TIM_OC2Init(TIM1,&TIM_OCInitStruct);
	TIM_OC3Init(TIM1,&TIM_OCInitStruct);
	TIM_OC1Init(TIM1,&TIM_OCInitStruct);
	TIM_OC4Init(TIM1,&TIM_OCInitStruct);	
	
	TIM_SetCompare2(TIM1 ,0);
	TIM_SetCompare3(TIM1 ,0);
	TIM_SetCompare1(TIM1 ,0);
	TIM_SetCompare4(TIM1 ,0);
	TIM_Cmd(TIM1,ENABLE);
	TIM_CtrlPWMOutputs(TIM1,ENABLE);
}

//Tim2-5通用定时器  TIM2/TIM5位32位
//void Init_Timer2(void)//配置为编码器模式 A0->CH1 A1->CH2 *******A0与按键冲突 TIM2用于舵机控制
//{
//	
//}

void Init_Timer3(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA ,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;  //TIM3_CH1  TIM3_CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_TIM3); 
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_TIM3); 	
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 0xffff;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 0x00;
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);
	
	//编码器模式
	TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_EncoderInterfaceConfig(TIM3,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//TIM_ICPolarity_Rising捕获发生在ICx的上升沿
    TIM_ICStructInit(&TIM_ICInitStructure); //将结构体中的内容缺省输入
    TIM_ICInitStructure.TIM_ICFilter = 0;//滤波器值
    TIM_ICInit(TIM3, &TIM_ICInitStructure);  
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1; 
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_SetCounter(TIM3, 0);
	TIM_Cmd(TIM3, ENABLE);
}

void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //溢出中断
	{
		
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update); //清除中断标志位
}

void Init_Timer4(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB ,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;  //TIM4_CH1  TIM4_CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_TIM4); 
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_TIM4); 	
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 0xffff;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 0x00;
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);
	
	//编码器模式
	TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_EncoderInterfaceConfig(TIM4,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//TIM_ICPolarity_Rising捕获发生在ICx的上升沿
    TIM_ICStructInit(&TIM_ICInitStructure); //将结构体中的内容缺省输入
    TIM_ICInitStructure.TIM_ICFilter = 0;//滤波器值
    TIM_ICInit(TIM4, &TIM_ICInitStructure);  
	
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1; 
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_SetCounter(TIM4, 0);
	TIM_Cmd(TIM4, ENABLE);
}

void TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4,TIM_IT_Update)==SET) //溢出中断
	{
		
	}
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update); //清除中断标志位
}



void Timer6_init(void)//基本定时器
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);//APB1:42Mhz 定时器6：84Mhz/8400 = 10Khz  0.0001s/count  6s
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = {0};//计时
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 60000-1;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 8400-1;
	TIM_TimeBaseInit(TIM6,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);
	
	NVIC_InitTypeDef NVIC_InitStructure = {0};
	NVIC_InitStructure.NVIC_IRQChannel=TIM6_DAC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=3;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	TIM_Cmd(TIM6, DISABLE);
}

void Timer7_init(void)//基本定时器
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);//APB1:42Mhz 定时器6：84Mhz/84 = 1Mhz 
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;//计时
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 10000-1;//10ms
	TIM_TimeBaseInitStructure.TIM_Prescaler = 84-1;
	TIM_TimeBaseInit(TIM7,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel=TIM7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=3;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	TIM_Cmd(TIM7, ENABLE);
}

void set_time(uint16_t time)
{
	timeout = 0;
	if(time > 65535){
		time = 65535;
	}
	TIM_SetAutoreload(TIM6, time -1);
	TIM_Cmd(TIM6, ENABLE);
}

void TIM6_DAC_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM6,TIM_IT_Update)==SET) //溢出中断
	{
		timeout = 1;
		TIM_Cmd(TIM6, DISABLE);
		TIM_SetCounter(TIM6, 0);
	}
	TIM_ClearITPendingBit(TIM6,TIM_IT_Update); //清除中断标志位
}

void TIM7_IRQHandler(void)//定时(10ms)读取编码器值(即速度)、mpu6050//超声波测距
{
	if(TIM_GetITStatus(TIM7,TIM_IT_Update)==SET) //溢出中断
	{
		if(motor_l.status == MOTOR_STATUS_ACTIVE){
			motor_l.coder_v = (int16_t)TIM4->CNT;
			motor_r.coder_v = (int16_t)TIM3->CNT;
		}else{
			motor_l.coder_v = 0;
		    motor_r.coder_v = 0;
		}
		TIM3->CNT = 0;
		TIM4->CNT = 0;
		mpu_dmp_get_data(&pitch, &roll, &yaw);
		my_yaw = yaw + 180;
	}
	TIM_ClearITPendingBit(TIM7,TIM_IT_Update); //清除中断标志位
}

