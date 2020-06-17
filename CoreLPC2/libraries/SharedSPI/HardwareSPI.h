#ifndef HARDWARESPI_H
#define HARDWARESPI_H

#include "Core.h"
#include "SharedSpi.h"
#include "chip.h"
#include "variant.h"
#include "SPI.h"
#include "DMA.h"

#include "FreeRTOS.h"
#include "semphr.h"

typedef struct
{
    LPC_SSP_T* sspHardwareDevice;
    DMA_Channel_t rxDmaChannel;
    DMA_Channel_t txDmaChannel;
    Pin defaultPins[4];
    SemaphoreHandle_t *sspTransferSemaphore;
} SSP_DEVICE_T;



class HardwareSPI: public SPI
{
public:
    HardwareSPI(const SSP_DEVICE_T* sspDevice) noexcept;
    spi_status_t sspi_transceive_packet(const uint8_t *tx_data, uint8_t *rx_data, size_t len) noexcept;
    spi_status_t sspi_transceive_packet_dma(const uint8_t *tx_data, uint8_t *rx_data, size_t len, DMA_TransferWidth_t transferWidth=DMA_WIDTH_BYTE) noexcept;
    void setup_device(const struct sspi_device *device, bool master=true) noexcept;

    void SspDmaRxTransfer(const void *buf, uint32_t transferLength, DMA_TransferWidth_t transferWidth=DMA_WIDTH_BYTE) noexcept;
    void SspDmaTxTransfer(const void *buf, uint32_t transferLength, DMA_TransferWidth_t transferWidth=DMA_WIDTH_BYTE) noexcept;

    void Enable();
    void Disable();
    void FlushAll();
    
private:
    const SSP_DEVICE_T* sspDevice;
    bool needInit;
    
    void SspDmaRxTransferNI(const void *buf, uint32_t transferLength, DMA_TransferWidth_t transferWidth=DMA_WIDTH_BYTE) noexcept;
    void SspDmaTxTransferNI(const void *buf, uint32_t transferLength, DMA_TransferWidth_t transferWidth=DMA_WIDTH_BYTE) noexcept;

    
    void InitialiseHardware(bool master);
    
    
    
};


#endif
