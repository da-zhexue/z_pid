#include "control_task.h"
#include "bsp_uart.h"
#include "bsp_odom.h"
#include <stdlib.h>
#include <time.h>

VEC_DATA vecdata;
pid_type_def pid_x, pid_y, pid_z, pid_v1, pid_v2, pid_v3, pid_v4;
int init_over = 0;

void cmd_send(int motor1, int motor2, int motor3, int motor4)
{
	;// 这里放电机驱动代码
}

void all_pid_init(void)
{
	PID_init(&pid_v1, PID_POSITION, PID_V, OUT_MAX_V, IOUT_MAX_V);
	PID_init(&pid_v2, PID_POSITION, PID_V, OUT_MAX_V, IOUT_MAX_V);
	PID_init(&pid_v3, PID_POSITION, PID_V, OUT_MAX_V, IOUT_MAX_V);
	PID_init(&pid_v4, PID_POSITION, PID_V, OUT_MAX_V, IOUT_MAX_V);
	
	PID_init(&pid_x, PID_POSITION, PID_P, OUT_MAX_P, IOUT_MAX_P);
	PID_init(&pid_y, PID_POSITION, PID_P, OUT_MAX_P, IOUT_MAX_P);
	PID_init(&pid_z, PID_POSITION, PID_P, OUT_MAX_P, IOUT_MAX_P);
}

void control_task(void)
{
	if(init_over == 1)
	{
		/* 假装这里获得了底盘电机编码器数据 */
			//printf("control\r\n");
			for(int i = 0; i < 4; i++)
				vecdata.motor_v[i] = rand() % 100;
			vecdata.vx = (vecdata.motor_v[0] + vecdata.motor_v[1] + vecdata.motor_v[2] + vecdata.motor_v[3])/4;
			vecdata.vy = (-vecdata.motor_v[0] + vecdata.motor_v[1] + vecdata.motor_v[2] - vecdata.motor_v[3])/4;
			vecdata.vz = (-vecdata.motor_v[0] + vecdata.motor_v[1] - vecdata.motor_v[2] + vecdata.motor_v[3])/(4 * MOTOR_L);

		/* 假装下面是控制代码 */
//		int target_x = 114, target_y = 514, target_z = 0; // 不知道从哪来的目标位置
//		int target_vx = PID_calc(&pid_x, odomdata.x, target_x);
//		int target_vy = PID_calc(&pid_y, odomdata.y, target_y);
//		int target_vz = PID_calc(&pid_z, odomdata.z, target_z);
//		
//		int target_v1 = target_vx - target_vy - target_vz * MOTOR_L;
//		int target_v2 = target_vx + target_vy + target_vz * MOTOR_L;
//		int target_v3 = target_vx + target_vy - target_vz * MOTOR_L;
//		int target_v4 = target_vx - target_vy + target_vz * MOTOR_L;
//		
//		int motor1_out = PID_calc(&pid_v1, vecdata.motor_v[0], target_v1);
//		int motor2_out = PID_calc(&pid_v2, vecdata.motor_v[1], target_v2);
//		int motor3_out = PID_calc(&pid_v3, vecdata.motor_v[2], target_v3);
//		int motor4_out = PID_calc(&pid_v4, vecdata.motor_v[3], target_v4);
//		
//		cmd_send(motor1_out, motor2_out, motor3_out, motor4_out);	
	}

}
