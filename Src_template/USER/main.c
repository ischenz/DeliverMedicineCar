#include "oled.h"
#include "key.h"
#include "usart.h"
#include "delay.h"
#include "led.h"
#include "timer.h"
#include "motor.h"
#include "datasave.h"
#include "stmflash.h"
#include "Kalman.h"
#include "pid.h"
#include "control.h"
#include "track.h"
#include "mpu6050.h"
#include "pid.h"
#include "gw_grayscale_sensor.h"
#include "sw_i2c.h"

float line_P;
char str[100] = {0};

int main(void)
{	
	double l_p = 25, l_i = 7, l_d = 25, r_p = 25, r_i = 7, r_d = 25;
	uint8_t data1,data2;
	uint8_t count;
	uint8_t num[6];
	uint32_t heartbeat = 0;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
	
	RingBuff_Init(&Uart1_RingBuff);
	RingBuff_Init(&Uart2_RingBuff);
	delay_init(168);
	Motor_Gpio_Init();
	LED_Init();
	uart_init(115200);
	uart2_v831_init(115200);
	
	KEY_Init();//weak不能使用 与定时器通道冲突
	
	OLED_Init();
	Kalman_Init(&kfp_line);
	delay_ms(500);
	MPU_Init();
	DMP_Init();
	
	sw_i2c_init();
	
	
	grayscale_init();
	
//	read_pid_from_flash(L_PID_FLASH_ADDR, "L_PID", &l_p, &l_i, &l_d);
//	read_pid_from_flash(R_PID_FLASH_ADDR, "R_PID", &r_p, &r_i, &r_d);
	PID_param_init(&l_pid);
	PID_param_init(&r_pid);
	PID_param_init(&veer_pid);
	set_p_i_d(&l_pid, l_p, l_i, l_d);
	set_p_i_d(&r_pid, r_p, r_i, r_d);//79.0, 8.3, 0
	set_p_i_d(&veer_pid, 2, 0, 5);
	set_pid_target(&l_pid,0);
	set_pid_target(&r_pid,0);
	set_pid_target(&veer_pid,0);
	PID_TimerInit();
	
	Init_Timer3();
	Init_Timer4();
	Timer7_init();//获取速度和yaw
	Timer6_init();
	
	printf("Hello world\r\n");
	
	TIM_Cmd(TIM10,ENABLE);
	
	
	
	set_time(20000);//2s
	while(1){
		OLED_ShowString(30, 10, "Get Room:", 12, 1);
		OLED_Refresh();
		if(timeout){
			count = get_num(num, 10);
			get_num_dir(num[0], 5);
			if(count == 1){
				OLED_ShowNum(84,10, num[0], 1, 12, 1);
				OLED_Refresh();
				break;
			}else{
				set_time(20000);
			}
			
//			count = get_num_dir(4, 10);
//			OLED_ShowNum(84,10, count, 3, 12, 1);
			OLED_Refresh();
			
		}
		if(DataDecode(&Uart2_RingBuff,&data1,&data2) == 0){
			//printf("%#4X %#4X\r\n", data1, data2);
			analyse(&data1, &data2);
		}
	}

	while(medc_scan() != 1){
		OLED_ShowString(30, 30, "Wait Med", 12, 1);
		OLED_Refresh();
	}
	OLED_Clear();
	OLED_ShowString(0, 0, "stop:", 12, 1);
	OLED_ShowString(0,12, "room:", 12, 1);
	OLED_ShowString(0,24, "cod_l:", 12, 1);
	OLED_ShowString(0,36, "cod_r:", 12, 1);
	OLED_ShowString(0,48, "Yaw:", 12, 1);
	OLED_ShowString(70, 0, "cros:", 12, 1);
	OLED_ShowString(60, 12, "dir_c:", 12, 1);
	OLED_Refresh();
	motor_set_status(&motor_l, MOTOR_STATUS_ACTIVE);
	motor_set_status(&motor_r, MOTOR_STATUS_ACTIVE);
//	track.status = ENABLE;
//	track.mode = 1;
//	set_pid_target(&l_pid,20);
//	set_pid_target(&r_pid,20);
	while(1)
	{
		if(v831_stop){
			set_green_led(200);
		}else{
			set_green_led(0);
		}
		main_task(num[0]);
		//mode_1();
		
		if(DataDecode(&Uart2_RingBuff,&data1,&data2) == 0){
			//printf("%#4X %#4X\r\n", data1, data2);
			analyse(&data1, &data2);
		}

#ifdef DEBUG		
		printf("data=%d\r\n", track.offset);
		printf("%3d %3d %3d %3d %3d %3d %3d %3d\r\n",num_dir[0],num_dir[1],num_dir[2],num_dir[3],num_dir[4],num_dir[5],num_dir[6],num_dir[7]);
		printf("%3d %3d %3d %3d %3d %3d %3d %3d\r\n",num_value[0],num_value[1],num_value[2],num_value[3],num_value[4],num_value[5],num_value[6],num_value[7]);
		count = get_num(num, 0);
		printf("count=%d, %3d, %3d, %3d, %3d \r\n", count, num[0], num[1],num[2],num[3]);
		printf("%3d %3d %3d %3d %3d %3d %3d %3d\r\n",num_value[0],num_value[1],num_value[2],num_value[3],num_value[4],num_value[5],num_value[6],num_value[7]);
		printf("%f,%d\r\n",r_pid.Target,motor_r.coder_v);
#endif		
		
#ifdef DBUG_PID		
		if(receiveJson(&Uart1_RingBuff, str)){
			printf("Rec:\r\n%s \r\n", str);
			OLED_Clear();
			if(read_json_pid(str, "L_PID", &l_p, &l_i, &l_d) == 0 || read_json_pid(str, "R_PID", &r_p, &r_i, &r_d) == 0){
				pid_flash_root_init(ADDR_FLASH_SECTOR_11);//使用扇区11,先擦除flash，后一次性写入全部pid参数
				write_pid_to_flash(L_PID_FLASH_ADDR, "L_PID", l_p, l_i, l_d);
				write_pid_to_flash(R_PID_FLASH_ADDR, "R_PID", r_p, r_i, r_d);
				set_p_i_d(&l_pid, l_p, l_i, l_d);
				set_p_i_d(&r_pid, r_p, r_i, r_d);
			}else{
				OLED_ShowString(0, 0, "Not this PID", 12, 1);
				OLED_Refresh();
			}
			sprintf(showstr, "L_P:%5.2f  R_P:%5.2f",l_p, r_p);
			OLED_ShowString(0, 0, (int8_t *)showstr, 12, 1);
			sprintf(showstr, "L_I:%5.2f  R_I:%5.2f",l_i, r_i);
			OLED_ShowString(0, 20, (int8_t *)showstr, 12, 1);
			sprintf(showstr, "L_D:%5.2f  R_D:%5.2f",l_d, r_d);
			OLED_ShowString(0, 30, (int8_t *)showstr, 12, 1);
			OLED_Refresh();
		}	
#endif

		heartbeat++;
		if(heartbeat%300 == 0){
			LED1 = !LED1;
			OLED_ShowNum(36, 0, v831_stop, 2, 12, 1);
			OLED_ShowNum(36,12, num[0], 1, 12, 1);
			OLED_ShowSNum(36,24, motor_l.coder_v, 3, 12, 1);
			OLED_ShowSNum(36,36, motor_r.coder_v, 3, 12, 1);
			OLED_ShowFNum(36,48, my_yaw, 5, 12, 1);
			OLED_ShowNum(100, 0, track.cross_num, 2, 12, 1);
			OLED_ShowSNum(96, 12, num_dir[num[0]-1], 3, 12, 1);
			OLED_Refresh();
		}
	}
}
