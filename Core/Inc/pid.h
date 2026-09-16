#ifndef __PID_H__
#define __PID_H__

#include "stm32f1xx_hal.h"

void Control(void);	// 每10ms调用一次（TIM3中断）

#endif
