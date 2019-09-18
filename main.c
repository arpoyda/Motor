#include "stm32f0xx.h"
#include "core_cm0.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_iwdg.h"
#include "usart.h"
#include "hal.h"
#include "modbus.h"
#include "adc_dma.h"


void SystemInit(void)
{
}

int main()
{
  rcc_config();
  /*
  modbus_init(1);
  adc_config(); //1
  __enable_irq();
  start_delay();
  adc_delay_config();
  */
  while(1){
    IWDG_ReloadCounter();
  }
}
