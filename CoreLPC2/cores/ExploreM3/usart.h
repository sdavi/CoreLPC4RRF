
#ifndef _EXPLORE_M3_USART_H_
#define _EXPLORE_M3_USART_H_


#ifdef __cplusplus
extern "C" {
#endif
#include "chip.h"
#include "uart_17xx_40xx.h"
    
#include "stdint.h"
#include "gpio.h"


#include "ring_buffer.h"



typedef struct
{ 
  gpioPins_et TxPin;
  gpioPins_et RxPin;
  uint8_t PinFunSel;
  uint8_t pconBit;
  LPC_USART_T *UARTx;
} usart_channel_map;

typedef struct usart_dev {
    const usart_channel_map *channel;             /*LPC1768 UART Channel Register Mapping */
    uint32_t max_baud;
    IRQn_Type irq_NUM;  
} usart_dev;

void usart_init(const usart_dev *dev, uint32_t baud_rate);


extern const usart_dev *USART0;
//extern usart_dev *USART1;
//extern usart_dev *USART2;
//extern usart_dev *USART3;



#ifdef __cplusplus
}
#endif



#endif

