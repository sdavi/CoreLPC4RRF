
#include "uart.h"
#include "usart.h"
#include "gpio.h"
#include "stdutils.h"
#include "lpc17xx.h"
#include <inttypes.h>
#include "HardwareSerial.h"
#include "Core.h"


//SD:: only using UART0

extern usart_channel_map USART_BASE[4]; //Defined in UART.c file




void usart0_IRQHandler(void);
void usart1_IRQHandler(void);
void usart2_IRQHandler(void);
void usart3_IRQHandler(void);


static usart_dev usart0 = {
    .channel     = &USART_BASE[0],
    .baud_rate = 0,
    .max_baud = 4500000UL,
    .irq_NUM = UART0_IRQn,
    .userFunction = usart0_IRQHandler,
};
usart_dev *USART0 = &usart0;



/*
static usart_dev usart1 = {
    .channel     = &USART_BASE[1],
    .baud_rate = 0,
    .max_baud = 4500001UL,
    .irq_NUM = UART1_IRQn,
    .userFunction = usart1_IRQHandler,
};
usart_dev *USART1 = &usart1;




static usart_dev usart2 = {
    .channel     = &USART_BASE[2],
    .baud_rate = 0,
    .max_baud = 2250001UL,
    .irq_NUM = UART2_IRQn,
    .userFunction = usart2_IRQHandler,
};
usart_dev *USART2 = &usart2;




static usart_dev usart3 = {
    .channel     = &USART_BASE[3],
    .baud_rate = 0,
    .max_baud = 2250002UL,
    .irq_NUM = UART3_IRQn,
    .userFunction = usart3_IRQHandler,
};
usart_dev *USART3 = &usart3;

*/


 

//Serial_baud from MBED
void serial_baud(LPC_UART_TypeDef *obj, int baudrate) {

    // set pclk to /1
    if( obj == (LPC_UART_TypeDef *)LPC_UART0_BASE){ LPC_SC->PCLKSEL0 &= ~(0x3 <<  6); LPC_SC->PCLKSEL0 |= (0x1 <<  6);}
    else if( obj == (LPC_UART_TypeDef *)LPC_UART1_BASE){ LPC_SC->PCLKSEL0 &= ~(0x3 <<  8); LPC_SC->PCLKSEL0 |= (0x1 <<  8); }
    else if( obj == (LPC_UART_TypeDef *)LPC_UART2_BASE){ LPC_SC->PCLKSEL1 &= ~(0x3 << 16); LPC_SC->PCLKSEL1 |= (0x1 << 16); }
    else if( obj == (LPC_UART_TypeDef *)LPC_UART3_BASE){ LPC_SC->PCLKSEL1 &= ~(0x3 << 18); LPC_SC->PCLKSEL1 |= (0x1 << 18); }
    else {
        //error("serial_baud");
        return;
        
    }
    
    if(baudrate == 0) return;
    
    
    uint32_t PCLK = SystemCoreClock;
    
    // First we check to see if the basic divide with no DivAddVal/MulVal
    // ratio gives us an integer result. If it does, we set DivAddVal = 0,
    // MulVal = 1. Otherwise, we search the valid ratio value range to find
    // the closest match. This could be more elegant, using search methods
    // and/or lookup tables, but the brute force method is not that much
    // slower, and is more maintainable.
    uint16_t DL = PCLK / (16 * baudrate);
    
    uint8_t DivAddVal = 0;
    uint8_t MulVal = 1;
    int hit = 0;
    uint16_t dlv;
    uint8_t mv, dav;
    if ((PCLK % (16 * baudrate)) != 0) {     // Checking for zero remainder
        int err_best = baudrate, b;
        for (mv = 1; mv < 16 && !hit; mv++)
        {
            for (dav = 0; dav < mv; dav++)
            {
                // baudrate = PCLK / (16 * dlv * (1 + (DivAdd / Mul))
                // solving for dlv, we get dlv = mul * PCLK / (16 * baudrate * (divadd + mul))
                // mul has 4 bits, PCLK has 27 so we have 1 bit headroom which can be used for rounding
                // for many values of mul and PCLK we have 2 or more bits of headroom which can be used to improve precision
                // note: X / 32 doesn't round correctly. Instead, we use ((X / 16) + 1) / 2 for correct rounding
                
                if ((mv * PCLK * 2) & 0x80000000) // 1 bit headroom
                    dlv = ((((2 * mv * PCLK) / (baudrate * (dav + mv))) / 16) + 1) / 2;
                else // 2 bits headroom, use more precision
                    dlv = ((((4 * mv * PCLK) / (baudrate * (dav + mv))) / 32) + 1) / 2;
                
                // datasheet says if DLL==DLM==0, then 1 is used instead since divide by zero is ungood
                if (dlv == 0)
                    dlv = 1;
                
                // datasheet says if dav > 0 then DL must be >= 2
                if ((dav > 0) && (dlv < 2))
                    dlv = 2;
                
                // integer rearrangement of the baudrate equation (with rounding)
                b = ((PCLK * mv / (dlv * (dav + mv) * 8)) + 1) / 2;
                
                // check to see how we went
                b = abs(b - baudrate);
                if (b < err_best)
                {
                    err_best  = b;
                    
                    DL        = dlv;
                    MulVal    = mv;
                    DivAddVal = dav;
                    
                    if (b == baudrate)
                    {
                        hit = 1;
                        break;
                    }
                }
            }
        }
    }
    
    // set LCR[DLAB] to enable writing to divider registers
    obj->LCR |= (1 << 7);
    
    // set divider values
    obj->DLM = (DL >> 8) & 0xFF;
    obj->DLL = (DL >> 0) & 0xFF;
    obj->FDR = (uint32_t) DivAddVal << 0
    | (uint32_t) MulVal    << 4;
    
    // clear LCR[DLAB]
    obj->LCR &= ~(1 << 7);
}

typedef enum {
    ParityNone = 0,
    ParityOdd = 1,
    ParityEven = 2,
    ParityForced1 = 3,
    ParityForced0 = 4
} SerialParity;

void serial_format(LPC_UART_TypeDef *obj, int data_bits, SerialParity parity, int stop_bits) {
    // 5 data bits = 0 ... 8 data bits = 3
    if (data_bits < 5 || data_bits > 8) {
        //error("Invalid number of bits (%d) in serial format, should be 5..8", data_bits);
        return;
    }

    data_bits -= 5;

    int parity_enable, parity_select;
    switch (parity) {
        case ParityNone: parity_enable = 0; parity_select = 0; break;
        case ParityOdd : parity_enable = 1; parity_select = 0; break;
        case ParityEven: parity_enable = 1; parity_select = 1; break;
        case ParityForced1: parity_enable = 1; parity_select = 2; break;
        case ParityForced0: parity_enable = 1; parity_select = 3; break;
        default:
            //error("Invalid serial parity setting");
            return;
    }

    // 1 stop bits = 0, 2 stop bits = 1
    if (stop_bits != 1 && stop_bits != 2) {
        //error("Invalid stop bits specified");
        return;
    }
    stop_bits -= 1;

    int break_transmission   = 0; // 0 = Disable, 1 = Enable
    int divisor_latch_access = 0; // 0 = Disable, 1 = Enable
    obj->LCR = data_bits << 0
    | stop_bits << 2
    | parity_enable << 3
    | parity_select << 4
    | break_transmission << 6
    | divisor_latch_access << 7;
}



void usart_init(usart_dev *dev) {
    uint32_t usartPclk,pclk,RegValue;
    
    
    GPIO_PinFunction(dev->channel->TxPin,dev->channel->PinFunSel);
    GPIO_PinFunction(dev->channel->RxPin,dev->channel->PinFunSel);
    util_BitSet(LPC_SC->PCONP,dev->channel->pconBit);
    
    /* Enable FIFO and reset Rx/Tx FIFO buffers */
    dev->channel->UARTx->FCR = (1<<SBIT_FIFO) | (1<<SBIT_RxFIFO) | (1<<SBIT_TxFIFO);
    

    //SD: updated use mbed function to set baud + format
    serial_baud(dev->channel->UARTx, dev->baud_rate);
    serial_format(dev->channel->UARTx,8, ParityNone, 1);
    
    
    dev->channel->UARTx->IER = 0x01; // Enable Rx  interrupt
    NVIC_EnableIRQ(dev->irq_NUM);
}



void usart_enable(usart_dev *dev) {

}


void usart_disable(usart_dev *dev) {

}



uint32_t usart_tx(usart_dev *dev, const uint8_t *buf, uint32_t len) {
    usart_channel_map *regs = dev->channel;
    uint32_t txed = 0;
    while ((util_IsBitCleared(regs->UARTx->LSR,SBIT_THRE)) && (txed < len)) {
        regs->UARTx->THR = buf[txed++];
    }
    return txed;
}


 
 

/**
 * @return Number of bytes received
 */
 
unsigned int usart_rx_available(usart_dev *dev) 
{
    return ((dev->rx_buf_head -dev->rx_buf_tail) & USART_BUF_SIZE_MASK );
}

char usart_getc(usart_dev *dev)
{
    char ch;

 
   if (dev->rx_buf_head == dev->rx_buf_tail) 
   {
    ch = -1;
    }
    else 
    {
       ch = dev->rx_buf[dev->rx_buf_tail];
       dev->rx_buf_tail = (dev->rx_buf_tail + 1) & USART_BUF_SIZE_MASK;
    }
   
   return ch;
} 

int usart_peek(usart_dev *dev)
{
  if (dev->rx_buf_head == dev->rx_buf_tail) 
  {
    return -1;
  } 
  else 
  {
    return dev->rx_buf[dev->rx_buf_tail];
  }
}

uint32_t usart_rx(usart_dev *dev, uint8_t *buf, uint32_t len) {
    uint32_t rxed = 0;
  

 
  while (rxed < len) {
        *buf++ = usart_getc(dev);
        rxed++;
    }   

    return rxed;
}



void usart_putc(usart_dev *dev, char ch){

      while (util_IsBitCleared(dev->channel->UARTx->LSR,SBIT_THRE));
     dev->channel->UARTx->THR = ch;

} 

void usart_putudec(usart_dev *dev, uint32_t val) {
    char digits[12];
    int i = 0;

    do {
        digits[i++] = val % 10 + '0';
        val /= 10;
    } while (val > 0);

    while (--i >= 0) {
        usart_putc(dev, digits[i]);
    }
}



/***************************************************************************************************
                            USART IRQ call backs
****************************************************************************************************
  Actual UART ISRs are in uart.c file.
                                 
****************************************************************************************************/
#define IIR_RDA		0x04
#define IIR_THRE	0x02
#define LSR_THRE    0x10
#define LSR_RDR     0x01

void usart0_IRQHandler(void)
{
  uint32_t iir_reg,lsr_reg;
  uint8_t uart_data, temp_head;
  

 
  iir_reg = LPC_UART0->IIR & 0x0f;

  lsr_reg = LPC_UART0->LSR; 
  
  if(iir_reg == IIR_RDA)
  {
      if(lsr_reg & LSR_RDR)
      {
      uart_data = LPC_UART0->RBR;
      temp_head = (USART0->rx_buf_head + 1) & USART_BUF_SIZE_MASK;
      
      if(temp_head != USART0->rx_buf_tail )
      {
          USART0->rx_buf[USART0->rx_buf_tail] = uart_data;
          USART0->rx_buf_head = temp_head;
      }
      }     
  }  
}


 
/*
void usart1_IRQHandler(void)
{
   uint32_t iir_reg,lsr_reg;
  uint8_t uart_data, temp_head;
	
 
  iir_reg = LPC_UART1->IIR & 0x0f;

  lsr_reg = LPC_UART1->LSR; 
  
  if(iir_reg == IIR_RDA)
  {
      if(lsr_reg & LSR_RDR)
      {
      uart_data = LPC_UART1->RBR;
      temp_head = (USART1->rx_buf_head + 1) & USART_BUF_SIZE_MASK;
      
      if(temp_head != USART1->rx_buf_tail )
      {
          USART1->rx_buf[USART1->rx_buf_tail] = uart_data;
          USART1->rx_buf_head = temp_head;
      }
      }     
  } 
}


void usart2_IRQHandler(void)
{
   uint32_t iir_reg,lsr_reg;
  uint8_t uart_data, temp_head;
	
 
  iir_reg = LPC_UART2->IIR & 0x0f;

  lsr_reg = LPC_UART2->LSR; 
  
  if(iir_reg == IIR_RDA)
  {
      if(lsr_reg & LSR_RDR)
      {
      uart_data = LPC_UART2->RBR;
      temp_head = (USART2->rx_buf_head + 1) & USART_BUF_SIZE_MASK;
      
      if(temp_head != USART2->rx_buf_tail )
      {
          USART2->rx_buf[USART2->rx_buf_tail] = uart_data;
          USART2->rx_buf_head = temp_head;
      }
      }     
  } 
}


void usart3_IRQHandler(void)
{
  uint32_t iir_reg,lsr_reg;
  uint8_t uart_data, temp_head;
	
 
  iir_reg = LPC_UART3->IIR & 0x0f;

  lsr_reg = LPC_UART3->LSR; 
  
  if(iir_reg == IIR_RDA)
  {
      if(lsr_reg & LSR_RDR)
      {
      uart_data = LPC_UART3->RBR;
      temp_head = (USART3->rx_buf_head + 1) & USART_BUF_SIZE_MASK;
      
      if(temp_head != USART3->rx_buf_tail )
      {
          USART3->rx_buf[USART3->rx_buf_tail] = uart_data;
          USART3->rx_buf_head = temp_head;
      }
      }     
  } 
}
 */
/*************************************************************************************************
                                    END of  ISR's 
*************************************************************************************************/


HardwareSerial Serial0(USART0);
//HardwareSerial Serial1(USART1);
//HardwareSerial Serial2(USART2);
//HardwareSerial Serial3(USART3);
