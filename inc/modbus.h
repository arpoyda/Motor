#ifndef _MODBUS_
#define _MODBUS_

#include "usart.h"
#include "hal.h"

#define modbus_id_def 2


#define READ_COIL 0x01
#define WRITE_SINGLE_COIL 0x05
#define WRITE_MULTI_COILS 0x0F

#define READ_DISCRERE_INPUT 0x02

#define READ_HOLDING_REGISTERS 0x03
#define READ_INPUT_REGISTERS 0x04
#define WRITE_SINGLE_REGISTER 0x06
#define WRITE_MULTI_REGISTERS 0x10

#define LED_CTR_BASE 0
#define IS_LED_CTR(x) ((x >= LED_CTR_BASE) && (x < LED_CTR_BASE + 41))


static uint8_t recivcommand; //for led triggering

static uint8_t modbus_id;
static uint32_t bitrade;


void modbus_init(uint8_t mode);
void modbus_saveflash(void);
uint8_t modbus_ispacks(void);

void USART1_RXHandeler(uint8_t* data, uint32_t len);


#endif