#include "bsp_odom.h"
#include "bsp_uart.h"
#include <stdlib.h>

ODOM_DATA odomdata;

void odom_task(void)
{
		//printf("odom\r\n");
		/* 假装这里有里程计数据处理 */
//		odomdata.x = rand() % 100;
//		odomdata.y = rand() % 100;
//		odomdata.z = rand() % 100;
		static int i=0;
		odomdata.x = i++;
		odomdata.y = 2*i;
		odomdata.z = 4*i;
		if(i >= 100)
			i = 0;
}
