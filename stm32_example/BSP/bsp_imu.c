#include "bsp_imu.h"
#include "bsp_uart.h"
#include <stdlib.h>

IMU_DATA imudata;

void imu_task(void)
{
		//printf("imu\r\n");
		/* 假装这里有IMU和odom数据处理 */
		imudata.ax = rand() % 100;
		imudata.ay = rand() % 100;
		imudata.az = rand() % 100;
		imudata.roll = rand() % 100;
		imudata.yaw = rand() % 100;
		imudata.pitch = rand() % 100;

}
