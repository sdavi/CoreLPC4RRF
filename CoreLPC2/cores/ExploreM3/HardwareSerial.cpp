/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Perry Hung.
 * Copyright (c) 2011, 2012 LeafLabs, LLC.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

/**
 * @file wirish/HardwareSerial.cpp
 * @brief Wirish serial port implementation.
 */

#include "chip.h"
#include "lpc_types.h"
#include "HardwareSerial.h"
#include "usart.h"
#include "interrupt_lpc.h"

HardwareSerial::HardwareSerial(const usart_dev *usart_device, uint8_t *rxBuffer, uint16_t rxRingBufferSize, uint8_t *txBuffer, uint16_t txRingBufferSize)
{
    this->usart_device = usart_device;
    
    //Setup RingBuffers
    RingBuffer_Init(&this->rxRingBuffer, rxBuffer, 1, rxRingBufferSize);
    RingBuffer_Init(&this->txRingBuffer, txBuffer, 1, txRingBufferSize);
    

    
}

void HardwareSerial::begin(uint32_t baud)
{
     
    if (baud > this->usart_device->max_baud)
    {
       return;
    }

//    this->usart_device->baud_rate = baud;
    usart_init(this->usart_device, baud);
	
}

void HardwareSerial::end(void) {
   
}

int HardwareSerial::read(void) {
    //read from the Ring Buffer
    int data;
    if(RingBuffer_Pop(&this->rxRingBuffer, &data))
    {
        return data;
    }
    else
    {
        return -1;
    }
}

int HardwareSerial::peek(void)
{

    int data;
    if(RingBuffer_Peek(&this->rxRingBuffer, &data))
    {
        return -1;
    }
    else
    {
        return data;
    }

}
int HardwareSerial::available(void) {
    irqflags_t flags = cpu_irq_save();
    int avail = RingBuffer_GetCount(&this->rxRingBuffer);
    cpu_irq_restore(flags);
    return avail;
}

int HardwareSerial::availableForWrite(void)
{
///* Currently there isn't an output ring buffer, chars are sent straight to the hardware.
// * so just return 1, meaning that 1 char can be written.
// * This will be slower than a ring buffer implementation.
// */
  //return 1;
    
    //return number for free items in the txRingBuffer
    irqflags_t flags = cpu_irq_save();
    int avail = RingBuffer_GetFree(&this->txRingBuffer);
    cpu_irq_restore(flags);
    return avail;

}
size_t HardwareSerial::canWrite()
{
//    return _tx_buffer->roomLeft();        // we may also be able to write 1 more byte direct to the UART, but this is close enough
//    return 1;
    irqflags_t flags = cpu_irq_save();
    int avail = RingBuffer_GetFree(&this->txRingBuffer);
    cpu_irq_restore(flags);
    return avail;

}

size_t HardwareSerial::write(const uint8_t ch) {
    
    Chip_UART_SendRB(this->usart_device->channel->UARTx, &this->txRingBuffer, &ch, 1);
    //Chip_UART_SendBlocking(this->usart_device->channel->UARTx, &ch, 1);
    

    return 1;
}


size_t HardwareSerial::write(const uint8_t *buffer, size_t size)
{
    size_t ret = size;
    while (size != 0)
    {
        size_t written = Chip_UART_SendRB(this->usart_device->channel->UARTx, &this->txRingBuffer, buffer, size);
        //size_t written = Chip_UART_SendBlocking(this->usart_device->channel->UARTx, buffer, size);
        buffer += written;
        size -= written;
    }
    return ret;

}

void HardwareSerial::flush(void) {
  //As the ring buffer is not used nothing is there to flush, Just included for compilation.
    while(RingBuffer_GetFree(&this->txRingBuffer) > 0); //wait for the ring buffer to empty
    
    //TODO:: should wait for the fifo to empty too
    
}

void HardwareSerial::IRQHandler(){
    //call the LPCOpen Interrupt Handler
    Chip_UART_IRQRBHandler(LPC_UART0, &this->rxRingBuffer, &this->txRingBuffer);

}



//compat with RRF
void HardwareSerial::setInterruptPriority(uint32_t priority)
{
    NVIC_SetPriority(this->usart_device->irq_NUM, priority);
    
}

uint32_t HardwareSerial::getInterruptPriority()
{
    return NVIC_GetPriority(this->usart_device->irq_NUM);
}

