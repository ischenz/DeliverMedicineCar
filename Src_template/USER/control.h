#ifndef __CONTROL_H
#define __CONTROL_H

#include "sys.h"

#define NO_SELECT_MODE	0x00
#define MODE_1			0x01
#define MODE_2			0x02
#define MODE_3			0x03
#define MODE_4			0x04
#define MODE_5			0x05
#define MODE_PRO_1 		0x06
#define MODE_PRO_2 		0x07
#define MODE_PRO_3 		0x08
#define MODE_PRO_4 		0x09

#define CMD				0xEE
#define DATA			0x80
#define SETPID			0x81

#define NUM_DIR			0x00
#define NUM_VALUE		0x01

#define STOP			0xbb
#define START			0xaa
#define REBOOT  		0x00
#define CHANGEMODE 		0xab
#define BSP_LED			0xff
#define V831_OK			0x55

void main_task(uint8_t room);
void mode_0(void);
void mode_1(void);
void mode_2(void);
void mode_3(void);
void mode_4(void);
void go_to_close(uint8_t room);
void go_to_middle(uint8_t room);
void mode_pro_1(void);
void mode_pro_2(void);
void mode_pro_3(void);
void mode_select(void);
void stop(void);
void start(void);
void analyse(uint8_t *lable, uint8_t *load);

uint8_t get_num(uint8_t *num, uint8_t threshold);
uint8_t get_num_dir(uint8_t num , uint8_t threshold);
uint8_t find_num_dir(uint8_t findnum, uint8_t threshold);

extern void (*mode_task)(void);
extern uint8_t mode;
extern uint8_t num_value[8];
extern int16_t num_dir[8];
extern volatile uint8_t v831_stop;
#endif /* __CONTROL_H */
