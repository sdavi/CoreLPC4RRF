/*
* ADCPreFilter.cpp
*
*  Created on: 15 Jan 2020
*      Author: sdavi
*/
 
#include "ADCPreFilter.h"
#include "Core.h"
#include "Median.h"
#include "DMA.h"

static uint32_t *adcSamplesArray;
static uint8_t numberADCSamples;
static uint16_t *median_buffer;
static uint8_t adcDmaChannel;
static volatile uint8_t currentSamplePosition;
static bool ADCPreFilterInitialised = false;

void ADC_DMA_HANDLER() noexcept;


bool ADCPreFilterInit(const uint8_t numSamples, const uint32_t sampleRateHz) noexcept
{
    numberADCSamples = numSamples;
    currentSamplePosition = 0;
    
    if(numberADCSamples == 0 || sampleRateHz == 0) return false;
    
    //create the array
    //Format will be:
    //       CH0 CH1 CH2 CH3 CH4 CH5 CH6 CH7
    //       CH0 CH1 CH2 CH3 CH4 CH5 CH6 CH7
    //        ...
    
    //DMA will fill in each row of samples (one sample per timer MR0 match, and when reaches last sample, goes back to beginning overwriting the oldest samples

    adcSamplesArray = new uint32_t[NumADCChannels*numberADCSamples];
    if(adcSamplesArray == nullptr) return false; //failed to allocate memory, disable filter
    
    memset(adcSamplesArray, 0, NumADCChannels*numberADCSamples*sizeof(adcSamplesArray[0]));
    median_buffer = new uint16_t[numberADCSamples];
    
    //Init Timer1
    Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_TIMER1);                          //Enable power and clocking
    Chip_TIMER_Reset(LPC_TIMER1);
    Chip_TIMER_Disable(LPC_TIMER1);                                             //Stop the Timer
    Chip_TIMER_PrescaleSet(LPC_TIMER1, (Chip_Clock_GetPeripheralClockRate(SYSCTL_PCLK_TIMER1)/1000000 - 1));   //Set the Prescaler to 1us
    Chip_TIMER_SetMatch(LPC_TIMER1, 0, 1000000/(sampleRateHz*NumADCChannels));  //Set MR0 (will be in microseconds)
    Chip_TIMER_ResetOnMatchEnable(LPC_TIMER1, 0);                               //Reset Timer on MR0 match

    NVIC_DisableIRQ(ADC_IRQn); //ensure ADC interrupts are disabled
    
    //Init GPDMA
    InitialiseDMA(); //Init GPDMA
    AttachDMAChannelInterruptHandler(ADC_DMA_HANDLER, DMA_TIMER_MAT1_0);        //attach to the DMA complete Interrupt handler
    adcDmaChannel = DMAGetChannelNumber(DMA_TIMER_MAT1_0);
    
    ADCPreFilterInitialised = true;
    
    ADC_DMA_HANDLER(); //call the handler to kick off the DMA sampling
    
    return true;
}


//from LPCOpen gpdma_17xx_40xx.c
static inline uint8_t configDMAMux(uint32_t gpdma_peripheral_connection_number) noexcept
{
    if (gpdma_peripheral_connection_number > 15)
    {
        LPC_SYSCTL->DMAREQSEL |= (1 << (gpdma_peripheral_connection_number - 16));
        return gpdma_peripheral_connection_number - 8;
    }
    else
    {
        if (gpdma_peripheral_connection_number > 7)
        {
            LPC_SYSCTL->DMAREQSEL &= ~(1 << (gpdma_peripheral_connection_number - 8));
        }
        return gpdma_peripheral_connection_number;
    }
}

static inline void PrepareADCDMA(uint32_t adcSampleBufferAddress) noexcept
{

    //Setup DMA Channel
    GPDMA_CH_T *pDMAch = (GPDMA_CH_T *) &(LPC_GPDMA->CH[adcDmaChannel]);
    pDMAch->CONFIG = GPDMA_DMACCxConfig_H;                          //halt the DMA channel - Clears DMA FIFO
    
    LPC_GPDMA->INTTCCLEAR = (((1UL << (adcDmaChannel)) & 0xFF));    //clear terminal count interrupt
    LPC_GPDMA->INTERRCLR = (((1UL << (adcDmaChannel)) & 0xFF));     //clear the error interrupt request

    // Enable DMA channels
    LPC_GPDMA->CONFIG = GPDMA_DMACConfig_E;
    while (!(LPC_GPDMA->CONFIG & GPDMA_DMACConfig_E)) {}

    pDMAch->SRCADDR  = (uint32_t)&LPC_ADC->DR[0];                           //Address of the first Channel Data Register
    pDMAch->DESTADDR = adcSampleBufferAddress;

    pDMAch->CONTROL  = GPDMA_DMACCxControl_TransferSize(8)                  //Transfer the 8 channels
                        | GPDMA_DMACCxControl_SBSize(GPDMA_BSIZE_1)         //Source burst size set to 1
                        | GPDMA_DMACCxControl_DBSize(GPDMA_BSIZE_1)         //Destination burst size set to 1
                        | GPDMA_DMACCxControl_SWidth(GPDMA_WIDTH_WORD)      //Source Tranfser width
                        | GPDMA_DMACCxControl_DWidth(GPDMA_WIDTH_WORD)      //Destination Transfer width
                        | GPDMA_DMACCxControl_SI                            //Source increment after each transfer
                        | GPDMA_DMACCxControl_DI                            //Destination increment after each transfer
                        | GPDMA_DMACCxControl_I;


    const uint16_t srcPeripheral = configDMAMux(GPDMA_CONN_MAT1_0);         //select Timer1 Mat0
    LPC_GPDMA->SYNC |= (1 << (srcPeripheral & 0xFFFF));                     //sync for timer mat

    
    pDMAch->CONFIG = GPDMA_DMACCxConfig_E
                        | GPDMA_DMACCxConfig_SrcPeripheral(srcPeripheral)
                        | GPDMA_DMACCxConfig_TransferType(GPDMA_TRANSFERTYPE_P2M_CONTROLLER_DMA)   //Peripheral to Memory
                        | GPDMA_DMACCxConfig_IE                                                    //Interrupt Error Mask
                        | GPDMA_DMACCxConfig_ITC;                                                  //Terminal count interrupt mask
    


}

void ADC_DMA_HANDLER() noexcept
{
    Chip_TIMER_Disable(LPC_TIMER1);

    uint32_t *ptr = adcSamplesArray;
    
    ptr += (currentSamplePosition*NumADCChannels); // Find the position in the array to the next ADC sample line for DMA to fill
    
    PrepareADCDMA((uint32_t )ptr);
    
    currentSamplePosition++;
    if(currentSamplePosition == numberADCSamples)
    {
        currentSamplePosition = 0;
    }
        
    //Start the Timer to kick off the DMA transfer
    LPC_TIMER1->TC = 0x00;          // Restart the Timer Count
    Chip_TIMER_Enable(LPC_TIMER1);
}

uint16_t ADCPreFilterRead(uint8_t channel) noexcept
{
    if(ADCPreFilterInitialised == false)
    {
        return 0;
    }
    
    for(uint8_t i=0; i<numberADCSamples; i++)
    {
        median_buffer[i] = ((adcSamplesArray[i*NumADCChannels + channel] >> 4) & 0xFFF);
    }

    return median_buffer[quick_median(median_buffer, numberADCSamples)];
}


// End
