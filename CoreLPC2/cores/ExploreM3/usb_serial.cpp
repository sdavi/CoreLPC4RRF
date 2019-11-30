/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2015 ExploreEmbedded.com
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



#include "usb_serial.h"
#include <inttypes.h>
#include "type.h"

#include "wirish_time.h"
#include "systick.h"


//SerialUSB::SerialUSB()
SerialUSB::SerialUSB(CircBuffer<uint8_t> *rxBuffer, CircBuffer<uint8_t> *txBuffer)
{
    
    this->usb = new (AHB0) USB();
    usb->init(); //initialise USB
    usb->connect();

    this->usbSerial = new (AHB0) USBSerial(usb, rxBuffer, txBuffer);
    
#if defined(ENABLE_DFU)
    this->dfu = new DFU(this->usb);
#endif

}



void SerialUSB::begin(uint32_t baud)
{
    (void)baud;
}

void SerialUSB::end(void) {
 
}

int SerialUSB::available(void) {
    
    return usbSerial->available();
}


int SerialUSB::peek(void)
{
    return -1;// Not Supported
}


int SerialUSB::read(void) {
    
    return usbSerial->_getc();

}



void SerialUSB::flush(void) {
    usbSerial->flush();
}


size_t SerialUSB::canWrite() const{
    return usbSerial->canWrite();
}

bool SerialUSB::IsConnected() const
{
    return usbSerial->isConnected();
}


size_t SerialUSB::write(uint8_t c)
{
    if (usbSerial->isConnected())
    {
        usbSerial->_putc(c);
    }
    return 1;
}


// Non-blocking write to USB. Returns number of bytes written. If we are not connected, pretend that all bytes hacve been written.
size_t SerialUSB::write(const uint8_t *buffer, size_t size)
{
    return usbSerial->writeBlock(buffer, size);
}



void SerialUSB::setInterruptPriority(uint32_t priority)
{
    //NVIC_SetPriority(_dwIrq, priority & 0x0F);
    NVIC_SetPriority(USB_IRQn, priority);

}

uint32_t SerialUSB::getInterruptPriority()
{
    return NVIC_GetPriority(USB_IRQn);
}
