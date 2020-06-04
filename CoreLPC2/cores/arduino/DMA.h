//Author: sdavi

#ifndef DMA_H
#define DMA_H

#include "Core.h"

typedef void (*DMACallbackFunction)(bool error);

//Max of 8 entries
enum DMA_Channel_t : uint8_t
{
    DMA_SSP0_RX=0,
    DMA_SSP0_TX,
    DMA_SSP1_RX,
    DMA_SSP1_TX,
    DMA_TIMER_MAT1_0,
    DMA_TIMER_MAT3_0,
    DMA_UART_TX,
    DMA_UART_RX
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
#endif // DMA_H
