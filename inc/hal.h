#ifndef __HAL__
#define __HAL__

#include "gpio_macros.h"
#include "stm32f0xx_rcc.h"
#include "adc_dma.h"

#define CHANNEL_NUM 8
#define OVER_SAMPLE 32

#define REG_NUM     4

#define MCOUNTS_REG 2
#define MDELAY_REG  3
#define MAXFILL_REG 1
#define MINFILL_REG 0

static uint16_t adc_buff[CHANNEL_NUM];
static uint16_t ADC_data[CHANNEL_NUM];
static uint16_t adc_sum[CHANNEL_NUM];
static uint8_t conversion_turnon;
static uint16_t conver_num;
static uint32_t adc_counter;

void rcc_config();
void start_delay();
void adc_delay_config();
void adc_delay_OFF();
void adc_delay_ON(uint16_t delay_time);
int16_t read_bin_register(int16_t addr);
int16_t write_bin_register(int16_t addr, int16_t data);
void adc_conversion();

#endif
