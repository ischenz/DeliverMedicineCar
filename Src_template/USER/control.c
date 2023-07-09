#include "control.h"
#include "oled.h"
#include "pid.h"
#include "usart.h"
#include "timer.h"
#include "delay.h"
#include "track.h"
#include "led.h"
#include "motor.h"
#include "delay.h"
#include "mpu6050.h"
#include "stdlib.h"

PID_TypeDef x_pid,y_pid;
uint8_t RecCoorFlag = 0; //接收成功标志
uint8_t mode = NO_SELECT_MODE;
uint8_t v831_stop = 0;

uint8_t num_value[8] = {0};
int8_t num_dir[8] = {0};

void (*mode_task)(void) = mode_0;

uint8_t get_num(uint8_t *num, uint8_t threshold)
{
	uint8_t i,ret;
	uint8_t count = 0;
	for(i = 0; i < 8; i++){
		num[i] = 0;
	}
	for(i = 0; i < 8; i++){
		if(num_value[i] > threshold){
			*(num+count) = i+1;
			count++;
		}
	}
	ret = count;
	for(i = 0; i < 8; i++){
		num_value[i] = 0;
	}
	return ret;
}

uint8_t get_num_dir(uint8_t num , uint8_t threshold)
{
	uint8_t i,ret;
	if(abs(num_dir[num]) > threshold){
		if(num_dir[num] > 0){
			ret = 0;
		}else{
			ret = 1;
		}
	}
	for(i = 0; i < 8; i++){
		num_dir[i] = 0;
	}
	return ret;
}

void mode_0(void)
{
	static uint8_t cross_num_flag = 0, setup_flag = 0;
	if(setup_flag == 0){
		setup_flag = 1;
		set_pid_target(&l_pid,20);
		set_pid_target(&r_pid,20);
		track.mode = 1;
	}
	if( track.cross_num == 1 && cross_num_flag == 0){
		cross_num_flag = 1;
		set_pid_target(&l_pid,0);
		set_pid_target(&r_pid,0);
		delay_ms(100);
		motor_set_status(&motor_l, MOTOR_STATUS_FREE);
		motor_set_status(&motor_r, MOTOR_STATUS_FREE);
		turn_pid(LEFT, 15);
		
		motor_set_status(&motor_l, MOTOR_STATUS_FREE);
		motor_set_status(&motor_r, MOTOR_STATUS_FREE);
		
		set_pid_target(&l_pid,20);
		set_pid_target(&r_pid,20);
		motor_set_status(&motor_l, MOTOR_STATUS_ACTIVE);
		motor_set_status(&motor_r, MOTOR_STATUS_ACTIVE);
		delay_ms(1000);
	}

	if(v831_stop == 1 && track.cross_num == 1){
		set_pid_target(&l_pid,0);
		set_pid_target(&r_pid,0);
		delay_ms(7000);
		motor_set_status(&motor_l, MOTOR_STATUS_FREE);
		motor_set_status(&motor_r, MOTOR_STATUS_FREE);

		turn_pid(BACK, 15);
		set_pid_target(&l_pid,0);
		set_pid_target(&r_pid,0);
		delay_ms(6000);
		motor_set_status(&motor_l, MOTOR_STATUS_FREE);
		motor_set_status(&motor_r, MOTOR_STATUS_FREE);

		while(1);
	}
	
}

void go_to_close(uint8_t room)
{
	static uint8_t step_flag = 0, setup_flag = 0;
	if(setup_flag == 0){
		setup_flag = 1;
		set_pid_target(&l_pid,20);
		set_pid_target(&r_pid,20);
		track.mode = 1;
	}
	if( track.cross_num == 1 && step_flag == 0){//左转
		step_flag += 1;
		set_pid_target(&l_pid,0);
		set_pid_target(&r_pid,0);
		delay_ms(100);
		
//		get_num(num, 5);
//		dir = get_num_dir(num[0], 5);
		
		motor_set_status(&motor_l, MOTOR_STATUS_FREE);
		motor_set_status(&motor_r, MOTOR_STATUS_FREE);
		if(room == 1){
			turn_pid(LEFT, 15);
		}else {
			turn_pid(RIGHT, 15);
		}
		
		
		motor_set_status(&motor_l, MOTOR_STATUS_FREE);
		motor_set_status(&motor_r, MOTOR_STATUS_FREE);
		
		set_pid_target(&l_pid,20);
		set_pid_target(&r_pid,20);
		motor_set_status(&motor_l, MOTOR_STATUS_ACTIVE);
		motor_set_status(&motor_r, MOTOR_STATUS_ACTIVE);
		delay_ms(1000);
	}

	if(v831_stop == 1 && track.cross_num == 1 && step_flag == 1){//停止-掉头
		step_flag++;
		set_pid_target(&l_pid,0);
		set_pid_target(&r_pid,0);
		delay_ms(7000);
		motor_set_status(&motor_l, MOTOR_STATUS_FREE);
		motor_set_status(&motor_r, MOTOR_STATUS_FREE);

		turn_pid(BACK, 15);
		delay_ms(6000);
		set_led_yellow();
		motor_set_status(&motor_l, MOTOR_STATUS_FREE);
		motor_set_status(&motor_r, MOTOR_STATUS_FREE);
		set_pid_target(&l_pid,20);
		set_pid_target(&r_pid,20);
		motor_set_status(&motor_l, MOTOR_STATUS_ACTIVE);
		motor_set_status(&motor_r, MOTOR_STATUS_ACTIVE);
		track.mode = 1;
	}
	
	if( track.cross_num == 2 && step_flag == 2){
		step_flag += 1;
		set_pid_target(&l_pid,0);
		set_pid_target(&r_pid,0);
		delay_ms(100);
		motor_set_status(&motor_l, MOTOR_STATUS_FREE);
		motor_set_status(&motor_r, MOTOR_STATUS_FREE);
		if(room == 1){
			turn_pid(RIGHT, 15);
		}else {
			turn_pid(LEFT, 15);
		}
		
		motor_set_status(&motor_l, MOTOR_STATUS_FREE);
		motor_set_status(&motor_r, MOTOR_STATUS_FREE);
		
		set_pid_target(&l_pid,20);
		set_pid_target(&r_pid,20);
		motor_set_status(&motor_l, MOTOR_STATUS_ACTIVE);
		motor_set_status(&motor_r, MOTOR_STATUS_ACTIVE);
		delay_ms(1000);
		v831_stop = 0;//打开
	}
	
	if(v831_stop == 1 && track.cross_num == 2 && step_flag == 3){
		step_flag++;
		v831_stop = 0;
		set_pid_target(&l_pid,0);
		set_pid_target(&r_pid,0);
		delay_ms(7000);
		motor_set_status(&motor_l, MOTOR_STATUS_FREE);
		motor_set_status(&motor_r, MOTOR_STATUS_FREE);

		turn_pid(BACK, 15);
		set_pid_target(&l_pid,0);
		set_pid_target(&r_pid,0);
		delay_ms(6000);
		motor_set_status(&motor_l, MOTOR_STATUS_FREE);
		motor_set_status(&motor_r, MOTOR_STATUS_FREE);
	}
}

void go_to_middle(uint8_t room)
{
//	uint8_t num[4] = {0};
	static uint8_t dir;
	static uint8_t step_flag = 0, setup_flag = 0;
	if(setup_flag == 0){
		setup_flag = 1;
		set_pid_target(&l_pid,20);
		set_pid_target(&r_pid,20);
		track.mode = 1;
	}
	if( track.cross_num == 1 && step_flag == 0){//左转
		step_flag += 1;
		set_pid_target(&l_pid,0);
		set_pid_target(&r_pid,0);
		delay_ms(100);
		
//		get_num(num, 5);
//		dir = get_num_dir(num[0], 5);
		
		motor_set_status(&motor_l, MOTOR_STATUS_FREE);
		motor_set_status(&motor_r, MOTOR_STATUS_FREE);
		if(dir == 1){
			turn_pid(LEFT, 15);
		}else {
			turn_pid(RIGHT, 15);
		}
		
		
		motor_set_status(&motor_l, MOTOR_STATUS_FREE);
		motor_set_status(&motor_r, MOTOR_STATUS_FREE);
		
		set_pid_target(&l_pid,20);
		set_pid_target(&r_pid,20);
		motor_set_status(&motor_l, MOTOR_STATUS_ACTIVE);
		motor_set_status(&motor_r, MOTOR_STATUS_ACTIVE);
		delay_ms(1000);
	}

	if(v831_stop == 1 && track.cross_num == 1 && step_flag == 1){//停止-掉头
		step_flag++;
		set_pid_target(&l_pid,0);
		set_pid_target(&r_pid,0);
		delay_ms(7000);
		motor_set_status(&motor_l, MOTOR_STATUS_FREE);
		motor_set_status(&motor_r, MOTOR_STATUS_FREE);

		turn_pid(BACK, 15);
		delay_ms(6000);
		set_led_yellow();
		motor_set_status(&motor_l, MOTOR_STATUS_FREE);
		motor_set_status(&motor_r, MOTOR_STATUS_FREE);
		set_pid_target(&l_pid,20);
		set_pid_target(&r_pid,20);
		motor_set_status(&motor_l, MOTOR_STATUS_ACTIVE);
		motor_set_status(&motor_r, MOTOR_STATUS_ACTIVE);
		track.mode = 1;
	}
	
	if( track.cross_num == 2 && step_flag == 2){
		step_flag += 1;
		set_pid_target(&l_pid,0);
		set_pid_target(&r_pid,0);
		delay_ms(100);
		motor_set_status(&motor_l, MOTOR_STATUS_FREE);
		motor_set_status(&motor_r, MOTOR_STATUS_FREE);
		if(dir == 1){
			turn_pid(RIGHT, 15);
		}else {
			turn_pid(LEFT, 15);
		}
		
		motor_set_status(&motor_l, MOTOR_STATUS_FREE);
		motor_set_status(&motor_r, MOTOR_STATUS_FREE);
		
		set_pid_target(&l_pid,20);
		set_pid_target(&r_pid,20);
		motor_set_status(&motor_l, MOTOR_STATUS_ACTIVE);
		motor_set_status(&motor_r, MOTOR_STATUS_ACTIVE);
		delay_ms(1000);
		v831_stop = 0;//打开
	}
	
	if(v831_stop == 1 && track.cross_num == 2 && step_flag == 3){
		step_flag++;
		v831_stop = 0;
		set_pid_target(&l_pid,0);
		set_pid_target(&r_pid,0);
		delay_ms(7000);
		motor_set_status(&motor_l, MOTOR_STATUS_FREE);
		motor_set_status(&motor_r, MOTOR_STATUS_FREE);

		turn_pid(BACK, 15);
		set_pid_target(&l_pid,0);
		set_pid_target(&r_pid,0);
		delay_ms(6000);
		motor_set_status(&motor_l, MOTOR_STATUS_FREE);
		motor_set_status(&motor_r, MOTOR_STATUS_FREE);
	}
}

void mode_1(void)
{

}

void mode_2(void)
{

}

void mode_3(void)
{

}


void mode_4(void)
{

}

void mode_pro_1(void)
{

}

void mode_pro_2(void)
{

}

void mode_pro_3(void)
{

}


void stop(void)
{
	TIM_Cmd(TIM10, DISABLE);//关闭PID计算
}

void start(void)
{
	TIM_Cmd(TIM10, ENABLE);
}

void analyse(uint8_t *lable, uint8_t *load)
{
	static uint8_t num;
	uint8_t _load = *load;
	switch(*lable){
		case NUM_VALUE:
			num = _load;
			num_value[_load - 1]++;
			
			break;
		case NUM_DIR:
			if(_load == 0)
				num_dir[num - 1] += 1;
			else if(_load == 1)
				num_dir[num - 1] -= 1;
			break;
		
		case 0x20:
			if(_load == 0x01){
				v831_stop = 1;
			}
		case CMD:{
			switch(*load){
				case V831_OK:
					delay_ms(3000);
					TIM_Cmd(TIM10,ENABLE);//开启pid运算
					break;
				case START:
					start();
					break;
				case STOP:
					stop();
					break;
				case REBOOT:
					NVIC_SystemReset();
					break;
				case BSP_LED:
					//LED2 = !LED2;
					break;
				}
			break;
		}
		case CHANGEMODE:{
			mode = *load;
			//SetFlag = 0;
			switch(mode){
				case MODE_1:{
					mode_task = mode_1;
					break;
				}
				case MODE_2:{
					mode_task = mode_2;
					break;
				}
				case MODE_3:{
					mode_task = mode_3;
					break;
				}
				case MODE_4:{
					mode_task = mode_4;
					break;
				}
				case MODE_5:{
					//mode_task = mode_5;
					break;
				}
				case MODE_PRO_1:
					mode_task = mode_pro_1;
					break;
				case MODE_PRO_2:
					mode_task = mode_pro_2;
					break;
				case MODE_PRO_3:
					mode_task = mode_pro_3;
					break;
			}
			start();
			break;
		}           
	}    
}                    


