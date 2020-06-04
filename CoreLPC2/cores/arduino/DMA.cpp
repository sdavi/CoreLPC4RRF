//Author: sdavi

#include "DMA.h"

extern "C" void debugPrintf(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));

static constexpr uint8_t NumberDMAChannels = 8;
static DMACallbackFunction dma_callbacks[NumberDMAChannels] = {nullptr};
static uint32_t dma_channels[NumberDMAChannels];

struct DMA_Config_t
{
    DMA_Channel_t chan;
    uint32_t peripheralConnection;
};

//Order defined here determines DMA channel priority
static const DMA_Config_t dma_config[] =
{
    {DMA_TIMER_MAT3_0,  GPDMA_CONN_MAT3_0},
    {DMA_TIMER_MAT1_0,  GPDMA_CONN_MAT1_0},
    {DMA_SSP0_RX,       GPDMA_CONN_SSP0_Rx},
    {DMA_SSP0_TX,       GPDMA_CONN_SSP0_Tx},
    {DMA_SSP1_RX,       GPDMA_CONN_SSP1_Rx},
    {DMA_SSP1_TX,       GPDMA_CONN_SSP1_Tx},
    {DMA_UART_TX,       GPDMA_CONN_UART0_Tx},
    {DMA_UART_RX,       GPDMA_CONN_UART0_Rx}
};

static_assert( (ARRAY_SIZE(dma_config) <= NumberDMAChannels ),  "DMA channel config exceeds max number of dma channels");

void InitialiseDMA() noexcept
{
    static bool gpdmaInit = false;
    if(gpdmaInit == false)
    {
        /* Initialize GPDMA controller */
        Chip_GPDMA_Init(LPC_GPDMA);

        for(uint8_t i=0; i< ARRAY_SIZE(dma_config); i++ )
        {
            dma_channels[dma_config[i].chan] = Chip_GPDMA_GetFreeChannel(LPC_GPDMA, dma_config[i].peripheralConnection);
        }
        
        NVIC_EnableIRQ(DMA_IRQn);
        
        gpdmaInit = true;
    }
}

//Get the DMA channel number that was assigned to DMA channels we configured.
uint8_t DMAGetChannelNumber(DMA_Channel_t dma_channel) noexcept
{
    return dma_channels[dma_channel];
}


//Attach DMA Handler for channel
void AttachDMAChannelInterruptHandler(DMACallbackFunction callback, DMA_Channel_t channel) noexcept
{
    dma_callbacks[channel] = callback;
}


//DMA Interrupt Handler
extern "C"  void DMA_IRQHandler(void) noexcept __attribute__ ((hot));
void DMA_IRQHandler(void) noexcept
{
    uint32_t flags = LPC_GPDMA->INTSTAT & 0xff; //GPDMA interrupt status register

    while(flags != 0)
    {
        unsigned int indx = LowestSetBitNumber(flags);
        flags &= ~(1u << indx);
        
        const bool error = (LPC_GPDMA->INTERRSTAT & (1u << indx))?true:false ; //check if the error interrupt flag is set
        
        //clear interrupts
        LPC_GPDMA->INTTCCLEAR = (1u << indx); //Clear Terminal Count Interrupt
        LPC_GPDMA->INTERRCLR =  (1u << indx); //Clear Error Interrupt
             
        //call the callback handler if it exists
        if(dma_callbacks[dma_config[indx].chan] != nullptr) dma_callbacks[dma_config[indx].chan](error);
    }
}

