//Author: sdavi

#ifndef DMA_H
#define DMA_H

#include "Core.h"

typedef void (*DMACallbackFunction)(void);

enum DMA_Channel_t : uint8_t
{
    DMA_SSP0_RX=0,
    DMA_SSP0_TX,
    DMA_SSP1_RX,
    DMA_SSP1_TX,
    DMA_TIMER_MAT1_0,
    DMA_TIMER_MAT3_0,
};

enum DMA_TransferWidth_t : uint8_t
{
    DMA_WIDTH_BYTE = GPDMA_WIDTH_BYTE,          //1 byte
    DMA_WIDTH_HALF_WORD = GPDMA_WIDTH_HALFWORD, //2 bytes
    DMA_WIDTH_WORD = GPDMA_WIDTH_WORD           //4 bytes
};

void InitialiseDMA() noexcept;
void AttachDMAChannelInterruptHandler(DMACallbackFunction callback, DMA_Channel_t channel) noexcept;

uint8_t DMAGetChannelNumber(DMA_Channel_t dma_channel) noexcept;

void SspDmaRxTransfer(DMA_Channel_t ssp_dma_channel, const void *buf, uint32_t transferLength, DMA_TransferWidth_t transferWidth=DMA_WIDTH_BYTE) noexcept;
void SspDmaTxTransfer(DMA_Channel_t ssp_dma_channel, const void *buf, uint32_t transferLength, DMA_TransferWidth_t transferWidth=DMA_WIDTH_BYTE) noexcept;

void SspDmaRxTransferNI(DMA_Channel_t ssp_dma_channel, const void *buf, uint32_t transferLength, DMA_TransferWidth_t transferWidth=DMA_WIDTH_BYTE) noexcept;
void SspDmaTxTransferNI(DMA_Channel_t ssp_dma_channel, const void *buf, uint32_t transferLength, DMA_TransferWidth_t transferWidth=DMA_WIDTH_BYTE) noexcept;


#endif // DMA_H
