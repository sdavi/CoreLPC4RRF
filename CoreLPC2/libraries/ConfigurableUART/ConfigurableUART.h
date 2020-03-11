//Author: sdavi

#ifndef _CONFIGURABLEUART_H_
#define _CONFIGURABLEUART_H_


#include "Core.h"
#include "HardwareSerial.h"
#include "Stream.h"
#include "Print.h"




class ConfigurableUART : public Stream
{

public:
    ConfigurableUART();

    bool Configure(Pin rx, Pin tx);
    
    void begin(uint32_t baud);
    void end();

    size_t write(const uint8_t *buffer, size_t size) override;
    size_t write(uint8_t) override;

    int available(void);
    int availableForWrite(void);
    
    int peek(void);
    int read(void);
    void flush(void);
    using Print::write;
    size_t canWrite();

    bool IsConnected();

    int8_t GetUARTPortNumber();

    void setInterruptPriority(uint32_t priority);
    uint32_t getInterruptPriority();

private:
    HardwareSerial *serialPort;
};

#define UARTClass ConfigurableUART // compatibility with RRF

extern ConfigurableUART UART_Slot0;
extern ConfigurableUART UART_Slot1;
extern ConfigurableUART UART_Slot2;


#endif
