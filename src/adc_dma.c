#include "adc_dma.h"

static void adc_calib(){
  if ((ADC1->CR & ADC_CR_ADEN) != 0)
    ADC1->CR |= ADC_CR_ADDIS;
  while ((ADC1->CR & ADC_CR_ADEN) != 0){};
  ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN;
  ADC1->CR |= ADC_CR_ADCAL;
  while ((ADC1->CR & ADC_CR_ADCAL) !=0){};
}

void adc_config(){
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
  GPIO_SET_ANALOG(GPIOA,0);
  GPIO_SET_ANALOG(GPIOA,1);
  GPIO_SET_ANALOG(GPIOA,2);
  GPIO_SET_ANALOG(GPIOA,3);
  GPIO_SET_ANALOG(GPIOA,4);
  GPIO_SET_ANALOG(GPIOA,5);
  GPIO_SET_ANALOG(GPIOA,6);
  GPIO_SET_ANALOG(GPIOA,7);
  RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
  RCC->AHBENR  |= RCC_AHBENR_DMAEN;
  adc_calib();
  
  ADC1->SMPR   |= ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1; // 71.5 clock cycles [110]
  ADC1->CHSELR  = ADC_CHSELR_CHSEL0 | ADC_CHSELR_CHSEL1 | ADC_CHSELR_CHSEL2
                | ADC_CHSELR_CHSEL3 | ADC_CHSELR_CHSEL4 | ADC_CHSELR_CHSEL5
                | ADC_CHSELR_CHSEL6 | ADC_CHSELR_CHSEL7;  
  ADC1->CFGR1  |= ADC_CFGR1_DMAEN;
  ADC1->CFGR1  |= ADC_CFGR1_CONT;
  ADC1->CFGR2  |= ADC_CFGR2_CKMODE_0;
  
  NVIC_SetPriority(DMA1_Channel1_IRQn, 3);
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

void adc_dma_rx(uint16_t* ADC_array, uint32_t len){
  ADC_DMA_RX->CCR   &= ~DMA_CCR_EN;
  ADC_DMA_RX->CPAR   = (uint32_t)(&(ADC1->DR));
  ADC_DMA_RX->CMAR   = (uint32_t)(ADC_array);
  ADC_DMA_RX->CNDTR  = (uint32_t)len; // len = 8 (adc channels)
  ADC_DMA_RX->CCR   |= DMA_CCR_MINC | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0
                     | DMA_CCR_CIRC | DMA_CCR_TCIE | DMA_CCR_HTIE;
  ADC_DMA_RX->CCR   |= DMA_CCR_EN; // enable
}

