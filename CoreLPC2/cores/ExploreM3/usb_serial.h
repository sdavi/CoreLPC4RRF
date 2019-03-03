
#ifndef _USB_SERIAL_H_
#define _USB_SERIAL_H_

#include "Stream.h"
#include "USBDevice/USBSerial/USBSerial.h"
#include "USBDevice/USBSerial/CircBuffer.h"

#include "gpio.h"


#if defined(ENABLE_DFU)
# include "DFU/DFU.h"
#endif


//SD: Wrapper around USBSerial to behave like arduino Serial
class SerialUSB : public Stream {

private:
    USBSerial *usbSerial;
    USB *usb;
    bool isConnected;

#if defined(ENABLE_DFU)
    DFU *dfu;
#endif

    
public:
    //SerialUSB();
    SerialUSB(CircBuffer<uint8_t> *rxBuffer, CircBuffer<uint8_t> *txBuffer);

    void begin(uint32_t baud);
    void end();
    
    int available();
    int peek();
    int read();
    
    void flush();
    size_t write(uint8_t);
    size_t write(const uint8_t *buffer, size_t size);
    size_t write(const char *buffer, size_t size) { return write((const uint8_t *)buffer, size); }

    size_t canWrite() const;    

    void setInterruptPriority(uint32_t priority);
    uint32_t getInterruptPriority();

    
    operator bool() const;
    
    
    
};

extern SerialUSB Serial;

#endif

