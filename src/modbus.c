#include "modbus.h"


uint16_t CRC16_2(uint8_t *buf, uint8_t len)
{
  uint16_t crc = 0xFFFF;

  for (uint8_t pos = 0; pos < len; pos++)
  {
    crc ^= (uint8_t)buf[pos];          // XOR byte into least sig. byte of crc

    for (uint8_t i = 8; i != 0; i--) {    // Loop over each bit
      if ((crc & 0x0001) != 0) {      // If the LSB is set
        crc >>= 1;                    // Shift right and XOR 0xA001
        crc ^= 0xA001;
      }
      else                            // Else LSB is not set
        crc >>= 1;                    // Just shift right
    }
  }
  // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
  return crc;
}

void modbus_saveflash(void)
{

}

void mb_put_error_info(uint8_t *buf, uint8_t funcn,uint8_t errnum)
{
  uint16_t crc;
  buf[0] = modbus_id;
  buf[1] = funcn | 0x80;
  buf[2] = errnum;
  crc = CRC16_2(buf, 3);
  buf[3] = (crc & 0x00FF);
  buf[4] = (crc >>8);
}

void modbus_init(uint8_t mode)
{
  modbus_id = modbus_id_def;
  bitrade = DEFBOUND;
  
  UART_init(cboundrates[bitrade]);
}

uint8_t modbus_ispacks(void)
{
  if (recivcommand) {
    recivcommand = 0;
    return 1;
  } else return 0;
}

void USART1_RXHandeler(uint8_t* data, uint32_t len)
{
  uint16_t addres, size;
  uint16_t crc;
  uint8_t answ;
  static uint8_t modbus_dataTX[64];
 
  if (len<8) return;
  if ((data[0] != modbus_id) && (data[0] !=0)) return;
  
  addres = data[2] << 8 | data[3];
  size = data[4] << 8 | data[5];
  crc = data[len-2] | data[len-1]<<8; //fucking other byteorder
  answ = 0;
  
  if (CRC16_2(data, len-2) != crc) return;
    
  recivcommand = 1;  

  switch(data[1]){
  case READ_HOLDING_REGISTERS:
        modbus_dataTX[0] = modbus_id;
        modbus_dataTX[1] = READ_HOLDING_REGISTERS;
        modbus_dataTX[2] = size*2;
        int16_t tp;
        int16_t i;
        for(i=0;i<size;i++) {
              if (IS_LED_CTR(addres+i)) {
                  tp = ledctr_r_reg(addres-LED_CTR_BASE+i);
                  modbus_dataTX[i*2+4] = (uint8_t)tp;
                  modbus_dataTX[i*2+3] = (uint8_t)((uint16_t)tp >> 8);
                  answ = 1;
               }
        }
        if (answ) {
          size= size*2;
          crc = CRC16_2(modbus_dataTX, size+3);
        
          modbus_dataTX[size+3] = (crc & 0x00FF);
          modbus_dataTX[size+4] = (crc >>8);
          uart_dma_tx(modbus_dataTX,size+5);
       }
    break;
    
  case WRITE_SINGLE_REGISTER:
     if (IS_LED_CTR(addres)) {
       answ = ledctr_w_reg(addres-LED_CTR_BASE, (int16_t)size);
    }
    
    if (answ) { 
      for(uint8_t i=0; i<8; i++)
        modbus_dataTX[i] = data[i];
       uart_dma_tx(modbus_dataTX,8);
    }
    break;
    
    
  }

}

