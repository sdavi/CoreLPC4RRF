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
    DMA_SSP1_TX
};

enum DMA_TransferWidth_t : uint8_t
{
    DMA_WIDTH_BYTE = GPDMA_WIDTH_BYTE,          //1 byte
    DMA_WIDTH_HALF_WORD = GPDMA_WIDTH_HALFWORD, //2 bytes
    DMA_WIDTH_WORD = GPDMA_WIDTH_WORD           //4 bytes
};

void InitialiseDMA(uint32_t irqPriority);
void AttachDMAChannelInterruptHandler(DMACallbackFunction callback, DMA_Channel_t channel);
void SspDmaRxTransfer(DMA_Channel_t ssp_dma_channel, const void *buf, uint32_t transferLength, DMA_TransferWidth_t transferWidth=DMA_WIDTH_BYTE);
void SspDmaTxTransfer(DMA_Channel_t ssp_dma_channel, const void *buf, uint32_t transferLength, DMA_TransferWidth_t transferWidth=DMA_WIDTH_BYTE);

void SspDmaRxTransferNI(DMA_Channel_t ssp_dma_channel, const void *buf, uint32_t transferLength, DMA_TransferWidth_t transferWidth=DMA_WIDTH_BYTE);
void SspDmaTxTransferNI(DMA_Channel_t ssp_dma_channel, const void *buf, uint32_t transferLength, DMA_TransferWidth_t transferWidth=DMA_WIDTH_BYTE);


#endif // DMA_H
