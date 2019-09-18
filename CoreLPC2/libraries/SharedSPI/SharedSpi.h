#ifndef _SHAREDSPI_H_
#define _SHAREDSPI_H_

#include "chip.h"

#define SPI_CPHA  (1 << 0) //Phase Control
#define SPI_CPOL  (1 << 1) //Clock Polarity


//Mode 0 - CPOL=0  - produces a steady state low value on the SCK pin
//       - CPHA=0  - data is captured on the first clock edge transition
#define SPI_MODE_0  0

//Mode 1 - CPOL=0  - produces a steady state low value on the SCK pin
//       - CPHA=1  - data is captured on the second clock edge transition
#define SPI_MODE_1  (SPI_CPHA)

//Mode 2 - CPOL=1  - a steady state high value is placed on the CLK pin when data is not being transferred
//       - CPHA=0  - data is captured on the first clock edge transition
#define SPI_MODE_2  (SPI_CPOL)

//Mode 3 - CPOL=1  - a steady state high value is placed on the CLK pin when data is not being transferred
//       - CPHA=1  - data is captured on the second clock edge transition
#define SPI_MODE_3  (SPI_CPOL | SPI_CPHA)

//! \brief Polled SPI device definition.
struct sspi_device {
	Pin csPin;
    bool csPolarity;
	uint8_t bitsPerTransferControl;
	uint8_t spiMode;
	uint32_t clockFrequency;
    SSPChannel sspChannel;
};

typedef enum
{
    SPI_ERROR = -1,
    SPI_OK = 0,
    SPI_ERROR_TIMEOUT = 1,
    SPI_ERROR_ARGUMENT,
    SPI_ERROR_OVERRUN,
    SPI_ERROR_MODE_FAULT,
    SPI_ERROR_OVERRUN_AND_MODE_FAULT
} spi_status_t;


#ifdef __cplusplus
extern "C" {
#endif

void sspi_master_init(struct sspi_device *device, uint32_t bits);
void sspi_master_setup_device(const struct sspi_device *device);
void sspi_select_device(const struct sspi_device *device);
void sspi_deselect_device(const struct sspi_device *device);
spi_status_t sspi_transceive_packet(const uint8_t *tx_data, uint8_t *rx_data, size_t len);

static inline spi_status_t sspi_read_packet(uint8_t *buf, size_t len)
{
	return sspi_transceive_packet(NULL, buf, len);
}

// Send a packet
static inline spi_status_t sspi_write_packet(const uint8_t *buf, size_t len)
{
	return sspi_transceive_packet(buf, NULL, len);
}
    
    //transceive 1 packet, returns read
uint8_t sspi_transceive_a_packet(uint8_t buf);
spi_status_t sspi_transceive_packet_16(const uint8_t *tx_data, uint8_t *rx_data, size_t len);

    
void sspi_setPinsForChannel(SSPChannel channel, Pin sck, Pin miso, Pin mosi);
    
#ifdef __cplusplus
}
#endif

#endif // _SHAREDSPI_H_
