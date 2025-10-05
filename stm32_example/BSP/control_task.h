#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "main.h"
#include "pid.h"

#define MOTOR_L 1 //运动学计算用数据

typedef struct
{
	int motor_v[4];
	int vx,vy,vz; // vz表示z轴角速度
	
} VEC_DATA;

extern VEC_DATA vecdata;
extern pid_type_def pid_x, pid_y, pid_z, pid_v1, pid_v2, pid_v3, pid_v4;
extern int init_over;

void control_task(void);
void cmd_send(int motor1, int motor2, int motor3, int motor4);
void all_pid_init(void);

#endif
