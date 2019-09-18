#include "hal.h"


void rcc_config(){
  RCC_HSEConfig(RCC_HSE_ON);
  if (RCC_WaitForHSEStartUp() == SUCCESS){
    RCC_SYSCLKConfig( RCC_SYSCLKSource_HSE);
    while(RCC_GetSYSCLKSource() != 0x04); 
    RCC_HSICmd(DISABLE);
  } else { 
    RCC_HSICmd(ENABLE);
    RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);
    while(RCC_GetSYSCLKSource() != 0x00){};
    RCC_HSEConfig(RCC_HSE_OFF);
  }
  
  RCC_HCLKConfig(RCC_SYSCLK_Div1);
  RCC_PCLKConfig(RCC_HCLK_Div1);  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
  
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB
                      | RCC_AHBPeriph_GPIOF | RCC_AHBPeriph_GPIOC, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1| RCC_APB2Periph_SYSCFG  ,ENABLE);
  
  GPIO_SET_OUTPUT(GPIOC, 13);
  GPIO_SET_BIT(GPIOC, 13);
}

void start_delay(){
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16,ENABLE);
  TIM16->PSC = 8000 - 1;                              // HSE frequency = ???, HSI = 8 MHz
  TIM16->ARR = 50; // 0.05sec
  //TIM16->DIER |= TIM_DIER_UIE;
  TIM16->CR1 |= TIM_CR1_CEN;
  while(TIM16->CNT < (TIM16->ARR)-1);
  TIM16->CNT = 0;
  TIM16->SR &= ~TIM_SR_UIF; //???
  TIM16->CR1 &= ~TIM_CR1_CEN;
}


void adc_ON(void){
  adc_dma_rx(adc_buff, CHANNEL_NUM);
  ADC1->CR |= ADC_CR_ADEN;
  ADC1->CR |= ADC_CR_ADSTART;
}

void ADC_DMA_RX_IRQHandler(){
  static uint8_t sample_counter = 0;
  int i;
  
  sample_counter++;
  
  if (DMA1->ISR && DMA_ISR_HTIF1){
    DMA1->IFCR = DMA_IFCR_CHTIF1;
    if (sample_counter <= OVER_SAMPLE){
      for (i=0; i<CHANNEL_NUM/2; i++)
        adc_sum[i] += adc_buff[i];
    } else {
      for (i=0; i<CHANNEL_NUM/2; i++)
        ADC_data[i] = adc_sum[i] / OVER_SAMPLE;
    }
  }
  if (DMA1->ISR && DMA_ISR_TCIF1){
    DMA1->IFCR = DMA_IFCR_CTCIF1;
    if (sample_counter <= OVER_SAMPLE){
      for (i=4; i<CHANNEL_NUM; i++)
        adc_sum[i] += adc_buff[i];
    } else {
      for (i=4; i<CHANNEL_NUM; i++)
        ADC_data[i] = adc_sum[i] / OVER_SAMPLE;
    }
    sample_counter = 0;
  }
}


void EXTI_config(){
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
  GPIO_SET_PUPDOWN(GPIOA,11);
  GPIO_SET_PUPDOWN(GPIOB,1);
  GPIO_SET_PUPDOWN(GPIOB,2);
  GPIO_SET_PUPDOWN(GPIOB,10);
  GPIO_SET_PUPDOWN(GPIOB,11);
  GPIO_SET_PUPDOWN(GPIOB,12);
  
  SYSCFG->EXTICR[2] = SYSCFG_EXTICR3_EXTI1_PB | SYSCFG_EXTICR3_EXTI10_PB;
  SYSCFG->EXTICR[3] = SYSCFG_EXTICR4_EXTI12_PB;
  EXTI->RTSR |= EXTI_RTSR_TR10 | EXTI_RTSR_TR12 | EXTI_RTSR_TR12;
  EXTI->PR = EXTI_PR_PR1 | EXTI_PR_PR10 | EXTI_PR_PR12;
  EXTI->IMR |= EXTI_IMR_MR1 | EXTI_IMR_MR10 | EXTI_IMR_MR12;
  
  NVIC_SetPriority(EXTI4_15_IRQn, 2);
  NVIC_EnableIRQ(EXTI4_15_IRQn);  
}

void EXTI4_15_IRQHandler(){
  static int8_t states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  static uint8_t enc_trans = 0;
  uint8_t enc_state = 0x00;
  
  
  if (EXTI->PR && EXTI_PR_PR10){
    EXTI->PR = EXTI_PR_PR10;
    enc_state = (GPIO_READ_BIT(GPIOB, 1) |  (GPIO_READ_BIT(GPIOB, 2) << 1));    //<<== Maybe will be changed!
    enc_trans = ((0x03 & enc_trans) << 2) | enc_state;
    if (states[enc_trans] == 1){
      //INCREASE SPEED
    }
    if (states[enc_trans] == -1){
      //REDUCE SPEED
    }
  }
  
  if (EXTI->PR && EXTI_PR_PR11){
    EXTI->PR = EXTI_PR_PR11;
    enc_state = (GPIO_READ_BIT(GPIOB, 10) |  (GPIO_READ_BIT(GPIOB, 11) << 1));    //<<== Maybe will be changed!
    enc_trans = ((0x03 & enc_trans) << 2) | enc_state;
    if (states[enc_trans] == 1){
      //INCREASE SPEED
    }
    if (states[enc_trans] == -1){
      //REDUCE SPEED
    }
  }
  
  if (EXTI->PR && EXTI_PR_PR12){
    EXTI->PR = EXTI_PR_PR12;
    enc_state = (GPIO_READ_BIT(GPIOB, 12) |  (GPIO_READ_BIT(GPIOA, 11) << 1));    //<<== Maybe will be changed!
    enc_trans = ((0x03 & enc_trans) << 2) | enc_state;
    if (states[enc_trans] == 1){
      //INCREASE SPEED
    }
    if (states[enc_trans] == -1){
      //REDUCE SPEED
    }
  }
}  


void TIM1_config(){
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);  
  //GPIO pins - push-pull as reset_state

  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
  //делитель
  TIM1->PSC = 400; // 8MHz->20KHz
 
  TIM1->ARR = 1000;
  TIM1->CCR1 = 0; // Continous increasing
  
  TIM1->CCER |= TIM_CCER_CC4E | TIM_CCER_CC4P;
  //разрешим использовать выводы таймера как выходы
  TIM1->BDTR |= TIM_BDTR_MOE;
  //PWM mode 1, прямой ШИМ 4 канал
  TIM1->CCMR2 = TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1; 
  //если надо настроить первый канал, это можно сделать так
  //TIM1->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
  //считаем вверх
  TIM1->CR1 &= ~TIM_CR1_DIR;
  //выравнивание по фронту, Fast PWM
  TIM1->CR1 &= ~TIM_CR1_CMS;
  //включаем счётчик
  TIM1->CR1 |= TIM_CR1_CEN;
  
  
}  

int16_t read_bin_register(int16_t addr){
  if (addr == 0){
      return conversion_turnon;
    } else
    if (addr < CHANNEL_NUM+1){
      return data_buff[addr-1];
    } else
    if (addr < REG_NUM+CHANNEL_NUM+1) {
      return flash_read_reg(addr-CHANNEL_NUM-1);
    }
  return -1;
}

int16_t write_bin_register(int16_t addr, int16_t data){
  if (addr == 0){
    if (!conversion_turnon && data){
      adc_conversion();
      conversion_turnon = data;
    }
    return 1;
  } else
  if (addr < CHANNEL_NUM+1){
    //adc_aver is only for read
    return 0;
  } else
  if (addr < REG_NUM+CHANNEL_NUM+1){
    flash_rewrite_page((addr-CHANNEL_NUM-1), data);
    return 1;
  }
  return 0;
}


