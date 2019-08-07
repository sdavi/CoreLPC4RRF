#ifndef SPI_H
#define SPI_H

#include "Core.h"


class SPI
{
public:
    virtual void setup_device(const struct sspi_device *device);
    virtual spi_status_t sspi_transceive_packet(const uint8_t *tx_data, uint8_t *rx_data, size_t len);
    virtual spi_status_t sspi_transceive_packet_16(const uint8_t *tx_data, uint8_t *rx_data, size_t len);
    
    virtual bool waitForTxEmpty();
};




#endif
