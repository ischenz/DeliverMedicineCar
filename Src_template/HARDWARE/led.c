#include "led.h" 
//��ʼ��PF9��PF10Ϊ�����.��ʹ���������ڵ�ʱ��		    
//LED IO��ʼ��
void LED_Init(void)
{    	 
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);//ʹ��GPIOFʱ��

	//GPIOF9,F10��ʼ������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;//LED0��LED1��ӦIO��
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_Init(GPIOF, &GPIO_InitStructure);//��ʼ��GPIO

	GPIO_ResetBits(GPIOF, GPIO_Pin_9 | GPIO_Pin_10);//GPIOF9,F10���øߣ�����
}

void set_led_yellow(void){
	GPIO_SetBits(GPIOE, GPIO_Pin_15);
	set_green_led(30);
	set_blue_led(0);
}

void led_off(void){
	GPIO_ResetBits(GPIOE, GPIO_Pin_15);
	set_green_led(0);
	set_blue_led(0);
}

