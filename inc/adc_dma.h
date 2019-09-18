#ifndef __ADCDMA__
#define __ADCDMA__

#include "stm32f0xx.h"
#include "stm32f0xx_rcc.h"
#include "gpio_macros.h"

#define ADC_DMA_RX DMA1_Channel1


static void adc_calib(); //sup
void adc_config();
void adc_dma_rx(uint16_t* ADC_array, uint32_t len);

#endif