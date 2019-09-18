#ifndef __USATR__
#define __USATR__

#include "gpio_macros.h"

#define F_CPU 48000000
#define USART_BOUND 115200
#define END_OF_LINE '\r'

#define UART1_RECIVE_LEN 128

#define UART1_RX_DMA DMA1_Channel3
#define UART1_TX_DMA DMA1_Channel2
#define UART1_TX_DMA_bit 2

#define UART1_SWAP_RX_TX 0
//#define UART1_DE 1
//#define UART1_DE_PIN GPIOA,1,1

static uint8_t recivebuff1[UART1_RECIVE_LEN];

#define DEFBOUND 10

static const uint32_t cboundrates[] = {110, 150, 300, 1200, 
                                       2400, 4800, 9600, 19200, 
                                       38400, 57600, 115200, 230400,
                                       460800, 921600};

void UART_init(uint32_t boudrate);
void uart_dma_tx(uint8_t *data,uint32_t len);
uint32_t uart_dma_tx_complete(void);
void uart_dma_rx(uint8_t *data,uint32_t len);


extern void USART1_RXHandeler(uint8_t *,uint32_t);
#endif