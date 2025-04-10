#include "dwt_delay.h"
#include "core_cm3.h"

void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}


void DWT_Delay_us(volatile uint32_t microseconds) {
  uint32_t clk_cycle_start = DWT->CYCCNT;
  microseconds *= (SystemCoreClock / 1000000);
  while ((DWT->CYCCNT - clk_cycle_start) < microseconds);
}
