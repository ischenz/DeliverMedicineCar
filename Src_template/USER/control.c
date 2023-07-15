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
volatile uint8_t v831_stop = 0;

uint8_t num_value[8] = {0};
int16_t num_dir[8] = {0};

void (*mode_task)(void) = mode_0;

uint8_t get_num_count_not_clear(uint8_t *num, uint8_t threshold)
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
	return ret;	
}

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
	uint8_t i,ret = 0xff;
	num -= 1;
	if(abs(num_dir[num]) >= threshold){
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

//uint8_t find_num_dir(uint8_t findnum, uint8_t threshold)
//{
//	uint8_t count = 0, i,dir = 0xff;
//	uint8_t num[4];
//	count = get_num(num, threshold);
//	for(i = 0; i < count; i++){
//		if(num[i] == findnum){
//			dir = get_num_dir(findnum, threshold);
//			break;
//		}
//	}
//	get_num_dir(num[0], threshold);
//	return dir;
//	
//}

void mode_0(void)
{

}

void main_task(uint8_t room)
{
	if(room <= 2){
		go_to_close(room);
	}else if(room <= 8){
		go_to_middle(room);
	}	
}

void go_to_close(uint8_t room)
{
	static uint8_t step_flag = 0, setup_flag = 0;
	if(setup_flag == 0){
		setup_flag = 1;
		track.mode = 1;
		track.status = ENABLE;
		set_pid_target(&l_pid,20);
		set_pid_target(&r_pid,20);
		
	}
	if( track.cross_num == 1 && step_flag == 0){
		step_flag += 1;
		set_pid_target(&l_pid,0);
		set_pid_target(&r_pid,0);
		delay_ms(100);
		
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

static void go_back(uint8_t arr[4])
{
    uint8_t data[4];
    for (uint8_t i = 0; i < 4; i++)
    {
        data[4-i-1] = arr[i];
		if(data[4-i-1] == 0 || data[4-i-1] == 1){
			data[4-i-1] = !data[4-i-1];
		}
    }
    for (uint8_t i = 0, j = 0; i < 4; i++)
    {
        if (data[i] == 0xff)
        {
            continue;
        }
        arr[j++] = data[i];
    }
}

/**第一个路口直行
  *
  *
  *
  *
 **/
void go_to_middle(uint8_t room)
{
	uint8_t num[4] = {0};
	static uint8_t dir, cross_num, num_count;
	static uint8_t step_flag = 0, setup_flag = 0;
	static uint8_t roadmap[4] = {0xff, 0xff, 0xff, 0xff};
	
	/************************开始----初始化*************************/
	if(setup_flag == 0){
		setup_flag = 1;
		get_num(num, 10);
		get_num_dir(num[0], 5);
		track.mode = 1;
		track.status = ENABLE;
		set_pid_target(&l_pid,20);
		set_pid_target(&r_pid,20);
		v831_stop = 0;
	}
	
	/************************第一个路口*************************/
	if( track.cross_num == 1 && step_flag == 0){//路口1 |
		step_flag += 1;
		cross_num += 1;
		roadmap[0] = 2;
		get_num(num, 5);
		get_num_dir(num[0], 5);
	}
	
	if(cross_num == 2 && step_flag == 1){
		static uint8_t turn_flag = 0, straight_flag = 0;
		uint8_t direter = 0;
		num_count = get_num_count_not_clear(num, 3);
		if(num_count >= 2 && turn_flag == 0 && straight_flag == 0){		
			set_pid_target(&l_pid,0);
			set_pid_target(&r_pid,0);
			
			set_time(20000);
			while(timeout == 0){
				uint8_t data1, data2;
				if(DataDecode(&Uart2_RingBuff,&data1,&data2) == 0){
					analyse(&data1, &data2);
				}
			}
			
			
			//printf("%3d %3d %3d %3d %3d %3d %3d %3d\r\n",num_dir[0],num_dir[1],num_dir[2],num_dir[3],num_dir[4],num_dir[5],num_dir[6],num_dir[7]);
			//printf("%3d %3d %3d %3d %3d %3d %3d %3d\r\n",num_value[0],num_value[1],num_value[2],num_value[3],num_value[4],num_value[5],num_value[6],num_value[7]);
			get_num(num, 5);
			direter = get_num_dir(room, 3);	
			//printf("dir=%d\r\n", direter);
			if(direter != 0xff){
				dir = direter;
				straight_flag = 1;
				set_pid_target(&l_pid,20);
				set_pid_target(&r_pid,20);
				return;
			}
			track.status = DISABLE;
			turn_pid(35, 15);
			turn_flag++;
			
			get_num(num, 5);
			get_num_dir(num[0], 5);
		}else if(turn_flag == 1){
			if(num_count >= 2){
				turn_flag++;
				
				set_time(20000);
				while(timeout == 0){
					uint8_t data1, data2;
					if(DataDecode(&Uart2_RingBuff,&data1,&data2) == 0){
						analyse(&data1, &data2);
					}
				}
				//printf("%3d %3d %3d %3d %3d %3d %3d %3d\r\n",num_dir[0],num_dir[1],num_dir[2],num_dir[3],num_dir[4],num_dir[5],num_dir[6],num_dir[7]);
				//printf("%3d %3d %3d %3d %3d %3d %3d %3d\r\n",num_value[0],num_value[1],num_value[2],num_value[3],num_value[4],num_value[5],num_value[6],num_value[7]);
				get_num(num, 5);
				direter = get_num_dir(room, 2);
				//printf("dir=%d\r\n", direter);
				if( direter != 0xff){
					dir  = 1;
					set_led_yellow();
				}else{
					led_off();
					set_green_led(200);
					dir = 0;
				}
				turn_pid(-35, 15);
				set_pid_target(&l_pid,20);
				set_pid_target(&r_pid,20);
				track.status = ENABLE;
			}
		}
	}

	if( track.cross_num == cross_num + 1 && step_flag == 1 && track.cross_num <= 4 ){
		cross_num++;
		
		if(track.cross_num != 3){
			get_num(num, 5);
			dir = get_num_dir(room, 5);
		}
		roadmap[track.cross_num - 1] = dir;
		set_pid_target(&l_pid,0);
		set_pid_target(&r_pid,0);
		track.mode = 0;
		if(dir == 1){
			turn_pid(LEFT, 15);
		}else if(dir == 0){
			turn_pid(RIGHT, 15);
		}else{
			roadmap[track.cross_num - 1] = 2;
		}
		track.mode = 1;
		set_pid_target(&l_pid,20);
		set_pid_target(&r_pid,20);
		set_time(14000);
		USART_Cmd(USART2, DISABLE); 
		while(timeout == 0){
			set_green_led(200);
		}
		led_off();
		get_num(num, 5);
		get_num_dir(room, 5);
		USART_Cmd(USART2, ENABLE); 
	}
	if(v831_stop == 1 && step_flag == 1){
		//printf("%d,%d,%d,%d\r\n",roadmap[0], roadmap[1], roadmap[2], roadmap[3]);
		step_flag++;
		USART_Cmd(USART2, DISABLE); 
		set_pid_target(&l_pid,0);
		set_pid_target(&r_pid,0);
		track.mode = 0;
		turn_pid(BACK, 15);	
		go_back(roadmap);
		track.mode = 1;
		track.cross_num = 0;
		cross_num = 0;

		v831_stop = 0;	
		set_pid_target(&l_pid,20);
		set_pid_target(&r_pid,20);	
		
		USART_Cmd(USART2, ENABLE); 
	}
	if(step_flag == 2){//返回
		if(track.cross_num == cross_num + 1 && track.cross_num <= 4){
			cross_num++;

			dir = roadmap[track.cross_num - 1];
			track.mode = 0;
			if(dir == 1){
				turn_pid(LEFT, 15);
			}else if(dir == 0){
				turn_pid(RIGHT, 15);
			}else{
				roadmap[track.cross_num - 1] = 2;
			}
			track.mode = 1;
			set_pid_target(&l_pid,20);
			set_pid_target(&r_pid,20);
		}
		if(v831_stop == 1 && track.cross_num >= 2){
			v831_stop = 0;
			set_pid_target(&l_pid,0);
			set_pid_target(&r_pid,0);
		}
	}
}

void mode_1(void)//右 右 掉头 左左
{
	static uint8_t step_flag = 0, setup_flag = 0;
	if(setup_flag == 0){
		setup_flag = 1;
		track.mode = 1;
		track.status = ENABLE;
		set_pid_target(&l_pid,20);
		set_pid_target(&r_pid,20);
	}
	if( track.cross_num == 1 && step_flag == 0){
		step_flag += 1;
		set_pid_target(&l_pid,0);
		set_pid_target(&r_pid,0);

			turn_pid(RIGHT, 15);
		
		
		set_pid_target(&l_pid,20);
		set_pid_target(&r_pid,20);
		motor_set_status(&motor_l, MOTOR_STATUS_ACTIVE);
		motor_set_status(&motor_r, MOTOR_STATUS_ACTIVE);
		delay_ms(1000);
	}
	
	if( track.cross_num == 2 && step_flag == 1){
		step_flag += 1;
		set_pid_target(&l_pid,0);
		set_pid_target(&r_pid,0);

			turn_pid(RIGHT, 15);
		
		
		set_pid_target(&l_pid,20);
		set_pid_target(&r_pid,20);
		motor_set_status(&motor_l, MOTOR_STATUS_ACTIVE);
		motor_set_status(&motor_r, MOTOR_STATUS_ACTIVE);
	}

	if(v831_stop == 1 && track.cross_num == 2 && step_flag == 2){//停止-掉头
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
	if( track.cross_num == 3 && step_flag == 3){
		step_flag += 1;
		set_pid_target(&l_pid,0);
		set_pid_target(&r_pid,0);

			turn_pid(LEFT, 15);
		
		
		set_pid_target(&l_pid,20);
		set_pid_target(&r_pid,20);
		motor_set_status(&motor_l, MOTOR_STATUS_ACTIVE);
		motor_set_status(&motor_r, MOTOR_STATUS_ACTIVE);
	}
	if( track.cross_num == 4 && step_flag == 4){
		step_flag += 1;
		set_pid_target(&l_pid,0);
		set_pid_target(&r_pid,0);

			turn_pid(LEFT, 15);
		
		
		set_pid_target(&l_pid,20);
		set_pid_target(&r_pid,20);
		motor_set_status(&motor_l, MOTOR_STATUS_ACTIVE);
		motor_set_status(&motor_r, MOTOR_STATUS_ACTIVE);
	}
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


