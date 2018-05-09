#ifndef _COMMEN_H
#define _COMMEN_H

#include "stm32f4xx.h"
#include "stdint.h"
#include "string.h"
#include <stdint.h>
typedef 	u8 		byte;
typedef 	u16 	word;
typedef 	u32 	size_t;
typedef		u8      boolean;
#define true  1
#define false 0
#define HIGH  1
#define LOW   0

#define PI		3.1415926
#define PIx2	(PI*2)
#define PI1000	3142
#define PI2000	6283
#define PI_NUM	314
#define PI_DEN	100

// macro definition
#ifndef ABS
#define ABS(x)	  ((x)<0?(-(x)):(x))
#endif

#ifndef MIN
#define MIN(x,y)  ((x)<(y)?(x):(y))
#endif

#ifndef MAX
#define MAX(x,y)  ((x)>(y)?(x):(y))
#endif

#ifndef ROUND
#define ROUND(x)  (((x)<0)?(int)((x)-0.5):(int)((x)+0.5))
#endif

#define LIMIT_MIN(x,a)     x = MAX(x,a)
#define LIMIT_MAX(x,b)     x = MIN(x,b)
#define LIMIT_RANGE(x,a,b) x = MIN(MAX(x,a),b)

#define setbit(x,y) x|=(1<<y) //将X的第Y位置1
#define clrbit(x,y) x&=~(1<<y) //将X的第Y位清0

#endif
