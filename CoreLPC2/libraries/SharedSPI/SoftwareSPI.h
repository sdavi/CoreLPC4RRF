#ifndef SOFTWARESPI_H
#define SOFTWARESPI_H

#include "Core.h"
#include "SharedSpi.h"
#include "variant.h"
#include "SPI.h"

class SoftwareSPI: public SPI
{
public:
    SoftwareSPI() noexcept;
    void InitPins(Pin sck_pin, Pin miso_pin, Pin mosi_pin) noexcept;
    spi_status_t sspi_transceive_packet(const uint8_t *tx_data, uint8_t *rx_data, size_t len) noexcept;
    spi_status_t sspi_transceive_packet_16(const uint8_t *tx_data, uint8_t *rx_data, size_t len) noexcept;
    void setup_device(const struct sspi_device *device) noexcept;
    bool waitForTxEmpty() noexcept;

private:
    
    uint8_t transfer_byte(uint8_t byte_out) noexcept;

    
    bool needInit;
    Pin sck;
    Pin mosi;
    Pin miso;
};




#endif
