#ifndef __ODOM_H__
#define __ODOM_H__

#include "main.h"

typedef struct
{
	int x,y,z;
	
} ODOM_DATA;

extern ODOM_DATA odomdata;

void odom_task(void);

#endif
