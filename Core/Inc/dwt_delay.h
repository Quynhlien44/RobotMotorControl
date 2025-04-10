// dwt_delay.h
#ifndef DWT_DELAY_H
#define DWT_DELAY_H

#include "stm32l1xx_hal.h"

void DWT_Init(void);
void DWT_Delay_us(volatile uint32_t microseconds);

#endif
