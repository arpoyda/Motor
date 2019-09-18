#include "usart.h"

void UART_init(uint32_t boudrate)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  RCC_USARTCLKConfig(RCC_USART1CLK_SYSCLK);

  RCC_AHBPeriphClockCmd (RCC_AHBPeriph_GPIOA , ENABLE);
  GPIO_SET_AF(GPIOB, 2, 0);
  GPIO_SET_AF(GPIOB, 3, 0);

  USART1->BRR  =  (uint32_t) (F_CPU)  /  boudrate; // 
  USART1->CR1 =0;
  USART1->CR3 =0;
  USART1->CR2 =0;
  
  if (UART1_SWAP_RX_TX)
    USART1->CR2 |= USART_CR2_SWAP;
  
  USART1->CR3 |= USART_CR3_DMAT |USART_CR3_DMAR; // enable dma to tx and rx
  //USART1->CR1 |= USART_CR1_CMIE;
  //USART1->CR2 |= END_OF_LINE<<24;
  USART1->CR1 |= USART_CR1_RTOIE;
  USART1->CR2 |= USART_CR2_RTOEN;
  USART1->RTOR = 0xF; //  times wait
  
#ifdef UART1_DE
  GPIO_SET_AF__(UART1_DE_PIN);
  USART1->CR1 |=  0x08 << 21 | 0x08 << 16; // driver enable assert and de-assert time;
  USART1->CR3 |= USART_CR3_DEM ; 
#endif

  USART1->CR1 |=  USART_CR1_UE |USART_CR1_TE|USART_CR1_RE  ; 
  
  uart_dma_rx(recivebuff1,UART1_RECIVE_LEN);
    
  NVIC_EnableIRQ(USART1_IRQn);
  NVIC_SetPriority(USART1_IRQn,5);  
}


void uart_dma_tx(uint8_t *data,uint32_t len)
{
  while(uart_dma_tx_complete()==0);
  
  USART1->ICR |= 1<<6;
  DMA1->IFCR |= 1<<((UART1_TX_DMA_bit-1)*4+1);
  UART1_TX_DMA->CCR = 0<<10 | 0 << 8 | 1<<7 | 0<<6 | 1<<4;
  UART1_TX_DMA->CNDTR = (uint32_t)len;
  UART1_TX_DMA->CMAR = (uint32_t)data;
  UART1_TX_DMA->CPAR = (uint32_t)&(USART1->TDR);
  UART1_TX_DMA->CCR |= 1; // enable dma 
}

uint32_t uart_dma_tx_complete(void) //return 0 if not complete
{
  uint32_t tp;
  tp =  UART1_TX_DMA->CCR ==0 ? 1 :  DMA1->ISR & (1<<((UART1_TX_DMA_bit-1)*4+1));
  return tp;
}

void uart_dma_rx(uint8_t *data,uint32_t len)
{
  UART1_RX_DMA->CCR = 0<<10 | 0 << 8 | 1<<7 | 0<<6 | 0<<4;
  UART1_RX_DMA->CNDTR = (uint32_t)len;
  UART1_RX_DMA->CMAR = (uint32_t)data;
  UART1_RX_DMA->CPAR = (uint32_t)&(USART1->RDR);
  UART1_RX_DMA->CCR |= 1; // enable dma 
}
           
void USART1_IRQHandler(void)
{
  uint16_t n_recived;
  if (USART1->ISR & USART_ISR_RTOF)
  {  
    if  (UART1_RX_DMA->CNDTR != UART1_RECIVE_LEN) 
    {
      n_recived = UART1_RECIVE_LEN - UART1_RX_DMA->CNDTR;
      USART1_RXHandeler(recivebuff1,n_recived);  
    }
  }
  USART1->ICR = 0xFFFF;
  uart_dma_rx(recivebuff1,UART1_RECIVE_LEN);
}