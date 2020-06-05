//Author: sdavi

//Hardware SPI
#include "HardwareSPI.h"
#include "chip.h"

#include "DMA.h"

#include "FreeRTOS.h"
#include "semphr.h"

#define SPI_TIMEOUT       15000

//#define SSPI_DEBUG
extern "C" void debugPrintf(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));

static SemaphoreHandle_t ssp0TransferSemaphore;
static SemaphoreHandle_t ssp1TransferSemaphore;


// Wait for transmitter ready returning true if timed out
static inline bool waitForTxReady(LPC_SSP_T* sspDevice) noexcept
{
    uint32_t timeout = SPI_TIMEOUT;
    while (sspDevice->SR & SSP_STAT_BSY) //check if SSP BSY flag (1=busy)
    {
        if (--timeout == 0)
        {
            return true;
        }
    }
    return false;
}

//flushTxFifo from GloomyAndy
// Flush the TX fifo. Note this code makes use of "reserved" registers
// these are actually test registers for the ARM PrimeCell Synchronous Serial
// Port (PL022), which is used to provide SSP devices on the LPC1768 device.
// For details see:
// http://infocenter.arm.com/help/topic/com.arm.doc.ddi0194g/DDI0194G_ssp_pl022_r1p3_trm.pdf
#define SSPTCR(SSP) ((__IO uint32_t *)((__IO uint8_t *)(SSP) + 0x80))
#define SSPTDR(SSP) ((__IO uint32_t *)((__IO uint8_t *)(SSP) + 0x8C))

static inline void flushTxFifo(LPC_SSP_T* sspDevice)
{
    if(Chip_SSP_GetStatus(sspDevice, SSP_STAT_TFE)) return;
    // enable test mode access to the TX fifo
    *SSPTCR(sspDevice) |= 0x2;
    //debugPrintf("status reg %x\n", LPC_SSP0->SR);
    int cnt = 8;
    while(!Chip_SSP_GetStatus(sspDevice, SSP_STAT_TFE) && cnt-- > 0)
    {
        (void)(*SSPTDR(sspDevice));
    }
    // back to normal mode
    *SSPTCR(sspDevice) &= ~2;
    //debugPrintf("status reg %x cnt %d\n", LPC_SSP0->SR, cnt);
}



//Make sure all the SSP FIFOs are empty, if they aren't clear them
//returns true if ready, false if timeout
static inline bool clearSSPTimeout(LPC_SSP_T* sspDevice) noexcept
{
#ifdef SSPI_DEBUG
    if( (sspDevice->SR & SSP_STAT_BSY) ) debugPrintf("SPI Busy\n");
    if( !(sspDevice->SR & SSP_STAT_TFE) ) debugPrintf("SPI Tx Not Empty\n");
    if( (sspDevice->SR & SSP_STAT_RNE) ) debugPrintf("SPI Rx Not Empty\n");
#endif

    //wait for SSP to be idle
    if (waitForTxReady(sspDevice)) return true; //timed out

    //clear out the RX FIFO
    while (Chip_SSP_GetStatus(sspDevice, SSP_STAT_RNE))
    {
        Chip_SSP_ReceiveFrame(sspDevice);
    }

    flushTxFifo(sspDevice);
    
    Chip_SSP_ClearIntPending(sspDevice, SSP_INT_CLEAR_BITMASK);
    
    return false; //did not timeout (success)

}

// Wait for receive data available returning true if timed out
static inline bool waitForRxReady(LPC_SSP_T* sspDevice) noexcept
{
    uint32_t timeout = SPI_TIMEOUT;
    while ( !Chip_SSP_GetStatus(sspDevice, SSP_STAT_RNE) ) //wait until receive not empty is 1
    {
        if (--timeout == 0) return true;
    }
    return false;
}

static inline CHIP_SSP_BITS_T getSSPBits(uint8_t bits) noexcept
{
    //we will only support 8 or 16bit
    
    if(bits == 16) return SSP_BITS_16;
    
    return SSP_BITS_8;
}

static inline CHIP_SSP_CLOCK_MODE_T getSSPMode(uint8_t spiMode) noexcept
{
    switch(spiMode)
    {
        case SPI_MODE_0: //CPHA=0, CPOL=0
            return SSP_CLOCK_CPHA0_CPOL0;
        case SPI_MODE_1: //CPHA=1, CPOL=0
            return SSP_CLOCK_CPHA1_CPOL0;
        case SPI_MODE_2: //CPHA=0, CPOL=1
            return SSP_CLOCK_CPHA0_CPOL1;
        case SPI_MODE_3: //CPHA=1, CPOL=1 
            return SSP_CLOCK_CPHA1_CPOL1;
    }
    
    return SSP_CLOCK_CPHA0_CPOL0;
}


void ssp0_dma_interrupt(bool error) noexcept
{
    BaseType_t mustYield=false;
    xSemaphoreGiveFromISR(ssp0TransferSemaphore, &mustYield);
}

void ssp1_dma_interrupt(bool error) noexcept
{
    BaseType_t mustYield=false;
    xSemaphoreGiveFromISR(ssp1TransferSemaphore, &mustYield);
}

HardwareSPI::HardwareSPI(LPC_SSP_T *sspDevice) noexcept
    :selectedSSPDevice(sspDevice), needInit(true)
{
    if(sspDevice == LPC_SSP0)
    {
        ssp0TransferSemaphore = xSemaphoreCreateBinary();
        sspTransferSemaphore = &ssp0TransferSemaphore;
    }
    else
    {
        ssp1TransferSemaphore = xSemaphoreCreateBinary();
        sspTransferSemaphore = &ssp1TransferSemaphore;
    }
}


//setup the master device.
void HardwareSPI::setup_device(const struct sspi_device *device) noexcept
{
    Chip_SSP_Disable(selectedSSPDevice);
    
    if(needInit)
    {
        InitialiseDMA();
        
        if(selectedSSPDevice == LPC_SSP0)
        {
            Chip_Clock_SetPCLKDiv(SYSCTL_PCLK_SSP0, SYSCTL_CLKDIV_1); //set SPP peripheral clock to PCLK/1
            Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SSP0); //enable power and clocking
            
            GPIO_PinFunction(SPI0_SCK, PINSEL_FUNC_2);   /* Configure the Pinfunctions for SPI */
            GPIO_PinFunction(SPI0_MOSI,PINSEL_FUNC_2);
            GPIO_PinFunction(SPI0_MISO,PINSEL_FUNC_2);
            
            GPIO_PinDirection(SPI0_SCK, OUTPUT);        /* Configure SCK,MOSI,SSEl as Output and MISO as Input */
            GPIO_PinDirection(SPI0_MOSI,OUTPUT);
            GPIO_PinDirection(SPI0_MISO,INPUT);

            AttachDMAChannelInterruptHandler(ssp0_dma_interrupt, DMA_SSP0_RX); //attach to the RX complete DMA Intettrupt handler
        }
        else if (selectedSSPDevice == LPC_SSP1)
        {
            Chip_Clock_SetPCLKDiv(SYSCTL_PCLK_SSP1, SYSCTL_CLKDIV_1); //set SPP peripheral clock to PCLK/1
            Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SSP1); //enable power and clocking
            
            GPIO_PinFunction(SPI1_SCK, PINSEL_FUNC_2);   /* Configure the Pinfunctions for SPI */
            GPIO_PinFunction(SPI1_MOSI,PINSEL_FUNC_2);
            GPIO_PinFunction(SPI1_MISO,PINSEL_FUNC_2);
            
            GPIO_PinDirection(SPI1_SCK, OUTPUT);        /* Configure SCK,MOSI,SSEl as Output and MISO as Input */
            GPIO_PinDirection(SPI1_MOSI,OUTPUT);
            GPIO_PinDirection(SPI1_MISO,INPUT);
            
            AttachDMAChannelInterruptHandler(ssp1_dma_interrupt, DMA_SSP1_RX); //attach to the RX complete DMA Intettrupt handler
        }
        
        needInit = false;
    }

    Chip_SSP_Set_Mode(selectedSSPDevice, SSP_MODE_MASTER);
    Chip_SSP_SetFormat(selectedSSPDevice, getSSPBits(device->bitsPerTransferControl), SSP_FRAMEFORMAT_SPI, getSSPMode(device->spiMode));
    Chip_SSP_SetBitRate(selectedSSPDevice, device->clockFrequency);
    Chip_SSP_Enable(selectedSSPDevice);

    clearSSPTimeout(selectedSSPDevice);
}

spi_status_t HardwareSPI::sspi_transceive_packet_dma(const uint8_t *tx_data, uint8_t *rx_data, size_t len, DMA_TransferWidth_t transferWidth) noexcept
{
    if(selectedSSPDevice == nullptr) return SPI_ERROR_TIMEOUT;//todo: just return timeout error if null
    
    uint8_t dontCareRX = 0;
    uint8_t dontCareTX = 0xFF;
    
    DMA_Channel_t chanRX;
    DMA_Channel_t chanTX;
    
    
    selectedSSPDevice->DMACR = SSP_DMA_TX | SSP_DMA_RX; //enable DMA

    if(selectedSSPDevice == LPC_SSP0)
    {
        chanRX = DMA_SSP0_RX;
        chanTX = DMA_SSP0_TX;
    }
    else
    {
        chanRX = DMA_SSP1_RX;
        chanTX = DMA_SSP1_TX;
    }
    
    if(rx_data != nullptr)
    {
        SspDmaRxTransfer(chanRX, rx_data, len);
    }
    else
    {
        SspDmaRxTransferNI(chanRX, &dontCareRX, len); //No Increment mode, dont care about received data, overwrite dontCareRX
    }
        
    if(tx_data != nullptr)
    {
        SspDmaTxTransfer(chanTX, tx_data, len);
    }
    else
    {
        SspDmaTxTransferNI(chanTX, &dontCareTX, len); //No Increment mode, send 0xFF
    }
    
    configASSERT(*sspTransferSemaphore != nullptr);

    spi_status_t ret = SPI_OK;
    const TickType_t xDelay = SPITimeoutMillis / portTICK_PERIOD_MS; //timeout
    if( xSemaphoreTake(*sspTransferSemaphore, xDelay) == pdFALSE) // timed out or failed to take semaphore
    {
        ret = SPI_ERROR_TIMEOUT;
    }
    selectedSSPDevice->DMACR &= ~(SSP_DMA_RX | SSP_DMA_TX); //disable DMA

    
    return ret;
}

spi_status_t HardwareSPI::sspi_transceive_packet(const uint8_t *tx_data, uint8_t *rx_data, size_t len) noexcept
{
    
    if(selectedSSPDevice == nullptr) return SPI_ERROR_TIMEOUT;

    if(len > 32)
    {
        return sspi_transceive_packet_dma(tx_data, rx_data, len, DMA_WIDTH_BYTE);
    }

    uint32_t dOut = 0x000000FF;
    uint32_t dIn = 0x000000FF;

    //LPC has 8 FIFOs, fill them up first

    const size_t preloadFrames = min<size_t>(8, len);
    for (uint8_t i = 0; i < preloadFrames; i++)
    {
        dOut = (tx_data == nullptr) ? 0x000000FF : (uint32_t)*tx_data++;
        Chip_SSP_SendFrame(selectedSSPDevice, dOut);
    }
    len -= preloadFrames;
        
    //send rest of the data
    while (len)
    {
        //Read in a Frame
        while (!Chip_SSP_GetStatus(selectedSSPDevice, SSP_STAT_RNE));
        dIn = Chip_SSP_ReceiveFrame(selectedSSPDevice); //get received data from Data Register
        if(rx_data != nullptr) *rx_data++ = dIn;

        //Send the next Frame
        dOut = (tx_data == nullptr) ? 0x000000FF : (uint32_t)*tx_data++;
        Chip_SSP_SendFrame(selectedSSPDevice, dOut);
        
        len--;
    }
    
    //get the last remaining frames from Receive FIFO
    for (uint8_t i = 0; i < preloadFrames; i++)
    {
        while (!Chip_SSP_GetStatus(selectedSSPDevice, SSP_STAT_RNE));
        dIn = Chip_SSP_ReceiveFrame(selectedSSPDevice);
        if(rx_data != nullptr) *rx_data++ = dIn;
    }
    
    return SPI_OK;
}



/*static*/ void HardwareSPI::SspDmaRxTransfer(DMA_Channel_t ssp_dma_channel, const void *buf, uint32_t transferLength, DMA_TransferWidth_t transferWidth) noexcept
{
    // Setup DMA Receive: SSP --> inBuffer (Peripheral to Memory)
    const uint8_t channelNumber = DMAGetChannelNumber(ssp_dma_channel);
    uint8_t srcPeripheral;
    uint32_t srcAddress;
    
    if(ssp_dma_channel == DMA_SSP0_RX)
    {
        srcPeripheral = GPDMA_CONN_SSP0_Rx;
        srcAddress = (uint32_t)&LPC_SSP0->DR;
    }
    else if(ssp_dma_channel == DMA_SSP1_RX)
    {
        srcPeripheral = GPDMA_CONN_SSP1_Rx;
        srcAddress = (uint32_t)&LPC_SSP1->DR;
    }
    else
    {
        //invalid channel number
        return;
    }
    
    
    
    GPDMA_CH_T *pDMAchRx = (GPDMA_CH_T *) &(LPC_GPDMA->CH[channelNumber]);

    pDMAchRx->CONFIG = GPDMA_DMACCxConfig_H;                        //halt the DMA channel - Clears DMA FIFO
    
    LPC_GPDMA->INTTCCLEAR = (((1UL << (channelNumber)) & 0xFF));    //clear terminal count interrupt requests for Rx and Tx Channels
    LPC_GPDMA->INTERRCLR = (((1UL << (channelNumber)) & 0xFF));     //clear the error interrupt request

    
    // Enable DMA channels
    LPC_GPDMA->CONFIG = GPDMA_DMACConfig_E;
    while (!(LPC_GPDMA->CONFIG & GPDMA_DMACConfig_E)) {}

    pDMAchRx->SRCADDR  = srcAddress;                //Source Address (SSP DataRegister)
    pDMAchRx->DESTADDR = (uint32_t)buf;             //Destination Address (buffer in memory)
    pDMAchRx->LLI      = 0;                         //linked list item register

    pDMAchRx->CONTROL  = (transferLength & 0xFFF)
                            | GPDMA_DMACCxControl_SBSize(GPDMA_BSIZE_1)     //Source burst size set to 1
                            | GPDMA_DMACCxControl_DBSize(GPDMA_BSIZE_1)     //Destination burst size set to 1
                            | GPDMA_DMACCxControl_SWidth(transferWidth)  //Source Tranfser width set to 1 byte
                            | GPDMA_DMACCxControl_DWidth(transferWidth)  //Destination Transfer width set to 1 byte
                            | GPDMA_DMACCxControl_DI                        //Destination increment after each transfer
                            | GPDMA_DMACCxControl_I;                        //Terminal count interrupt enable

    
    pDMAchRx->CONFIG =  GPDMA_DMACCxConfig_E                                                        //Enable
                        | GPDMA_DMACCxConfig_SrcPeripheral(srcPeripheral)                           //Peripheral destination is SSP0 RX
                        | GPDMA_DMACCxConfig_TransferType(GPDMA_TRANSFERTYPE_P2M_CONTROLLER_DMA)    //Peripheral to Memory
                        | GPDMA_DMACCxConfig_IE                                                     //Interrupt Error Mask
                        | GPDMA_DMACCxConfig_ITC;                                                   //Terminal count interrupt mask

    
}

/*static*/ void HardwareSPI::SspDmaTxTransfer(DMA_Channel_t ssp_dma_channel, const void *buf, uint32_t transferLength, DMA_TransferWidth_t transferWidth) noexcept
{
    // Setup DMA transfer: outBuffer --> SSP (Memory to Peripheral Transfer)
    const uint8_t channelNumber = DMAGetChannelNumber(ssp_dma_channel);
    uint8_t dstPeripheral;
    uint32_t dstAddress;
    
    if(ssp_dma_channel == DMA_SSP0_TX)
    {
        dstPeripheral = GPDMA_CONN_SSP0_Tx;
        dstAddress = (uint32_t)&LPC_SSP0->DR;
    }
    else if(ssp_dma_channel == DMA_SSP1_TX)
    {
        dstPeripheral = GPDMA_CONN_SSP1_Tx;
        dstAddress = (uint32_t)&LPC_SSP1->DR;
    }
    else
    {
        //invalid channel number
        return;
    }

    
    
    GPDMA_CH_T *pDMAchTx = (GPDMA_CH_T *) &(LPC_GPDMA->CH[channelNumber]);

    pDMAchTx->CONFIG = GPDMA_DMACCxConfig_H;                        //halt the DMA channel - Also clears the DMA FIFO
    
    LPC_GPDMA->INTTCCLEAR = (((1UL << (channelNumber)) & 0xFF));   //clear terminal count interrupt requests for Rx and Tx Channels
    LPC_GPDMA->INTERRCLR = (((1UL << (channelNumber)) & 0xFF));    //clear the error interrupt request

    
    // Enable DMA channels
    LPC_GPDMA->CONFIG = GPDMA_DMACConfig_E;
    while (!(LPC_GPDMA->CONFIG & GPDMA_DMACConfig_E)) {}
    
    pDMAchTx->SRCADDR  = (uint32_t)buf;             //Source Address (buffer in memory)
    pDMAchTx->DESTADDR = dstAddress;                //Destination Address (SSP0 DataRegister)
    pDMAchTx->LLI      = 0;                         //linked list item register
    pDMAchTx->CONTROL  = (transferLength & 0xFFF)
                            | GPDMA_DMACCxControl_SBSize(GPDMA_BSIZE_1)     //Source burst size set to 1
                            | GPDMA_DMACCxControl_DBSize(GPDMA_BSIZE_1)     //Destination burst size set to 1
                            | GPDMA_DMACCxControl_SWidth(transferWidth)  //Source Tranfser width set to 1 byte
                            | GPDMA_DMACCxControl_DWidth(transferWidth)  //Destination Transfer width set to 1 byte
                            | GPDMA_DMACCxControl_SI                        //Source increment after each transfer
                            | GPDMA_DMACCxControl_I;                        //Terminal count interrupt enable

    //Enable Channel
    pDMAchTx->CONFIG = GPDMA_DMACCxConfig_E                                                             //Enable
                            | GPDMA_DMACCxConfig_DestPeripheral(dstPeripheral)                          //Peripheral destination is SSP0 TX
                            | GPDMA_DMACCxConfig_TransferType(GPDMA_TRANSFERTYPE_M2P_CONTROLLER_DMA)    //Memory to Peripheral
                            | GPDMA_DMACCxConfig_IE                                                     //Interrupt Error Mask
                            | GPDMA_DMACCxConfig_ITC;                                                   //Terminal count interrupt mask

    
}






//No Increment Versions (do not increment the memory buffer after each data transfer)

/*static*/ void HardwareSPI::SspDmaRxTransferNI(DMA_Channel_t ssp_dma_channel, const void *buf, uint32_t transferLength, DMA_TransferWidth_t transferWidth) noexcept
{
    // Setup DMA Receive: SSP --> inBuffer (Peripheral to Memory)
    const uint8_t channelNumber = DMAGetChannelNumber(ssp_dma_channel);
    uint8_t srcPeripheral;
    uint32_t srcAddress;
    
    if(ssp_dma_channel == DMA_SSP0_RX)
    {
        srcPeripheral = GPDMA_CONN_SSP0_Rx;
        srcAddress = (uint32_t)&LPC_SSP0->DR;
    }
    else if(ssp_dma_channel == DMA_SSP1_RX)
    {
        srcPeripheral = GPDMA_CONN_SSP1_Rx;
        srcAddress = (uint32_t)&LPC_SSP1->DR;
    }
    else
    {
        //invalid channel number
        return;
    }
    
    
    
    GPDMA_CH_T *pDMAchRx = (GPDMA_CH_T *) &(LPC_GPDMA->CH[channelNumber]);

    pDMAchRx->CONFIG = GPDMA_DMACCxConfig_H;                        //halt the DMA channel - Clears DMA FIFO
    
    LPC_GPDMA->INTTCCLEAR = (((1UL << (channelNumber)) & 0xFF));    //clear terminal count interrupt requests for Rx and Tx Channels
    LPC_GPDMA->INTERRCLR = (((1UL << (channelNumber)) & 0xFF));     //clear the error interrupt request

    
    // Enable DMA channels
    LPC_GPDMA->CONFIG = GPDMA_DMACConfig_E;
    while (!(LPC_GPDMA->CONFIG & GPDMA_DMACConfig_E)) {}

    pDMAchRx->SRCADDR  = srcAddress;                //Source Address (SSP DataRegister)
    pDMAchRx->DESTADDR = (uint32_t)buf;             //Destination Address (buffer in memory)
    pDMAchRx->LLI      = 0;                         //linked list item register

    pDMAchRx->CONTROL  = (transferLength & 0xFFF)
                            | GPDMA_DMACCxControl_SBSize(GPDMA_BSIZE_1)     //Source burst size set to 1
                            | GPDMA_DMACCxControl_DBSize(GPDMA_BSIZE_1)     //Destination burst size set to 1
                            | GPDMA_DMACCxControl_SWidth(transferWidth)  //Source Tranfser width set to 1 byte
                            | GPDMA_DMACCxControl_DWidth(transferWidth)  //Destination Transfer width set to 1 byte
                            | GPDMA_DMACCxControl_I;                        //Terminal count interrupt enable

    
    pDMAchRx->CONFIG =  GPDMA_DMACCxConfig_E                                                        //Enable
                        | GPDMA_DMACCxConfig_SrcPeripheral(srcPeripheral)                           //Peripheral destination is SSP0 RX
                        | GPDMA_DMACCxConfig_TransferType(GPDMA_TRANSFERTYPE_P2M_CONTROLLER_DMA)    //Peripheral to Memory
                        | GPDMA_DMACCxConfig_IE                                                     //Interrupt Error Mask
                        | GPDMA_DMACCxConfig_ITC;                                                   //Terminal count interrupt mask

    
}

/*static*/ void HardwareSPI::SspDmaTxTransferNI(DMA_Channel_t ssp_dma_channel, const void *buf, uint32_t transferLength, DMA_TransferWidth_t transferWidth) noexcept
{
    // Setup DMA transfer: outBuffer --> SSP (Memory to Peripheral Transfer)
    const uint8_t channelNumber = DMAGetChannelNumber(ssp_dma_channel);
    uint8_t dstPeripheral;
    uint32_t dstAddress;
    
    if(ssp_dma_channel == DMA_SSP0_TX)
    {
        dstPeripheral = GPDMA_CONN_SSP0_Tx;
        dstAddress = (uint32_t)&LPC_SSP0->DR;
    }
    else if(ssp_dma_channel == DMA_SSP1_TX)
    {
        dstPeripheral = GPDMA_CONN_SSP1_Tx;
        dstAddress = (uint32_t)&LPC_SSP1->DR;
    }
    else
    {
        //invalid channel number
        return;
    }

    
    
    GPDMA_CH_T *pDMAchTx = (GPDMA_CH_T *) &(LPC_GPDMA->CH[channelNumber]);

    pDMAchTx->CONFIG = GPDMA_DMACCxConfig_H;                        //halt the DMA channel - Also clears the DMA FIFO
    
    LPC_GPDMA->INTTCCLEAR = (((1UL << (channelNumber)) & 0xFF));   //clear terminal count interrupt requests for Rx and Tx Channels
    LPC_GPDMA->INTERRCLR = (((1UL << (channelNumber)) & 0xFF));    //clear the error interrupt request

    
    // Enable DMA channels
    LPC_GPDMA->CONFIG = GPDMA_DMACConfig_E;
    while (!(LPC_GPDMA->CONFIG & GPDMA_DMACConfig_E)) {}
    
    pDMAchTx->SRCADDR  = (uint32_t)buf;             //Source Address (buffer in memory)
    pDMAchTx->DESTADDR = dstAddress;                //Destination Address (SSP DataRegister)
    pDMAchTx->LLI      = 0;                         //linked list item register
    pDMAchTx->CONTROL  = (transferLength & 0xFFF)
                            | GPDMA_DMACCxControl_SBSize(GPDMA_BSIZE_1)     //Source burst size set to 1
                            | GPDMA_DMACCxControl_DBSize(GPDMA_BSIZE_1)     //Destination burst size set to 1
                            | GPDMA_DMACCxControl_SWidth(transferWidth)  //Source Tranfser width set to 1 byte
                            | GPDMA_DMACCxControl_DWidth(transferWidth)  //Destination Transfer width set to 1 byte
                            | GPDMA_DMACCxControl_I;                        //Terminal count interrupt enable

    //Enable Channel
    pDMAchTx->CONFIG = GPDMA_DMACCxConfig_E                                                             //Enable
                            | GPDMA_DMACCxConfig_DestPeripheral(dstPeripheral)                          //Peripheral destination is SSP TX
                            | GPDMA_DMACCxConfig_TransferType(GPDMA_TRANSFERTYPE_M2P_CONTROLLER_DMA)    //Memory to Peripheral
                            | GPDMA_DMACCxConfig_IE                                                     //Interrupt Error Mask
                            | GPDMA_DMACCxConfig_ITC;                                                   //Terminal count interrupt mask

    
}

