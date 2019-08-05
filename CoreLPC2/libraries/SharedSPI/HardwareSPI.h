#ifndef HARDWARESPI_H
#define HARDWARESPI_H

#include "Core.h"
#include "SharedSpi.h"
#include "lpc17xx.h"
#include "variant.h"
#include "SPI.h"

class HardwareSPI: public SPI
{
public:
    HardwareSPI(LPC_SSP_TypeDef* sspDevice);
    spi_status_t sspi_transceive_packet(const uint8_t *tx_data, uint8_t *rx_data, size_t len);
    spi_status_t sspi_transceive_packet_16(const uint8_t *tx_data, uint8_t *rx_data, size_t len);
    void setup_device(const struct sspi_device *device);
    bool waitForTxEmpty();
    
private:
    LPC_SSP_TypeDef* selectedSSPDevice;
    bool needInit;
    
};




#endif
