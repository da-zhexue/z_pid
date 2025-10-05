#ifndef __IMU_H__
#define __IMU_H__

#include "main.h"

typedef struct
{
	int ax,ay,az;
	int roll,yaw,pitch;
	
} IMU_DATA;

extern IMU_DATA imudata;

void imu_task(void);

#endif
