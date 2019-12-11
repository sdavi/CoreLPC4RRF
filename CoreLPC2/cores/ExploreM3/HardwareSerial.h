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
 * @file wirish/include/wirish/HardwareSerial.h
 * @brief Wirish serial port interface.
 */

#ifndef _WIRISH_HARDWARESERIAL_H_
#define _WIRISH_HARDWARESERIAL_H_



#include <inttypes.h>
#include "Print.h"
#include "usart.h"
#include "Stream.h"

class HardwareSerial : public Stream
{

public:
    HardwareSerial(const struct usart_dev *usart_device, uint8_t *rxBuffer, uint16_t rxRingBufferSize, uint8_t *txBuffer, uint16_t txRingBufferSize);


    /* Set up/tear down */
    void begin(uint32_t baud);
    void end();
    int available(void);
    int availableForWrite(void);
    int peek(void);
    int read(void);
    void flush(void);
    size_t write(const uint8_t *buffer, size_t size) override;
    size_t write(uint8_t) override;
    using Print::write;
    size_t canWrite();

	bool IsConnected() { return true; }

    void IRQHandler();
    
    void setInterruptPriority(uint32_t priority);
    uint32_t getInterruptPriority();

private:
    const struct usart_dev *usart_device;
    RINGBUFF_T txRingBuffer;
    RINGBUFF_T rxRingBuffer;

  protected:
};


#define UARTClass HardwareSerial // compatibility with RRF

extern HardwareSerial Serial0;

#if defined(ENABLE_UART1)
    extern HardwareSerial Serial1;
#endif
#if defined(ENABLE_UART2)
    extern HardwareSerial Serial2;
#endif
#if defined(ENABLE_UART3)
    extern HardwareSerial Serial3;
#endif

//
//



#endif
