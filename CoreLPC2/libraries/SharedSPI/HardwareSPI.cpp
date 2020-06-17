//Author: sdavi

//Hardware SPI
#include "HardwareSPI.h"
#include "task.h"

//#define SSPI_DEBUG
extern "C" void debugPrintf(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));

static SemaphoreHandle_t ssp0TransferSemaphore;
static SemaphoreHandle_t ssp1TransferSemaphore;


typedef struct
{
    Pin pin;
    uint32_t modefunc;
} SSP_PIN_CONFIG_T;


const SSP_DEVICE_T ssp0_defaults
{
    LPC_SSP0,
    DMA_SSP0_RX,
    DMA_SSP0_TX,
    {SPI0_SCK, SPI0_MISO, SPI0_MOSI, SPI0_SSEL},
    &ssp0TransferSemaphore
};

const SSP_DEVICE_T ssp1_defaults
{
    LPC_SSP1,
    DMA_SSP1_RX,
    DMA_SSP1_TX,
    {SPI1_SCK, SPI1_MISO, SPI1_MOSI, SPI1_SSEL},
    &ssp1TransferSemaphore
};

static const SSP_PIN_CONFIG_T sspPinMuxing[] =
{
    //SSP0 Pins
    {P0_15,   IOCON_MODE_INACT | IOCON_FUNC2},      // SCK0
    {P0_16,   IOCON_MODE_INACT | IOCON_FUNC2},      // SSEL0
    {P0_17,   IOCON_MODE_INACT | IOCON_FUNC2},      // MISO0
    {P0_18,   IOCON_MODE_INACT | IOCON_FUNC2},      // MOSI0
    //Alternates SSP0 Pins
    {P1_20,   IOCON_MODE_INACT | IOCON_FUNC3},      // SCK0
    {P1_21,   IOCON_MODE_INACT | IOCON_FUNC3},      // SSEL0
    {P1_23,   IOCON_MODE_INACT | IOCON_FUNC3},      // MISO0
    {P1_24,   IOCON_MODE_INACT | IOCON_FUNC3},      // MOSI0
    
    //SSP1
    {P0_6,  IOCON_MODE_INACT | IOCON_FUNC2},        // SSEL1
    {P0_7,  IOCON_MODE_INACT | IOCON_FUNC2},        // SCK1
    {P0_8,  IOCON_MODE_INACT | IOCON_FUNC2},        // MISO1
    {P0_9,  IOCON_MODE_INACT | IOCON_FUNC2},        // MOSI1
};


HardwareSPI ssp0(&ssp0_defaults);
HardwareSPI ssp1(&ssp1_defaults);



// Wait for transmitter ready returning true if timed out
static inline bool waitForTxReady(LPC_SSP_T* sspDevice) noexcept
{
    const TickType_t start = xTaskGetTickCount();
    while (sspDevice->SR & SSP_STAT_BSY) //check if SSP BSY flag (1=busy)
    {
        if( (xTaskGetTickCount() - start) >= pdMS_TO_TICKS(SPITimeoutMillis)) return true;
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

static inline void ConfigureSspPin(Pin pin)
{
    for(uint8_t i=0; i<ARRAY_SIZE(sspPinMuxing); i++)
    {
        if(pin == sspPinMuxing[i].pin)
        {
            Chip_IOCON_PinMuxSet(LPC_IOCON, (pin >> 5), (pin & 0x1f), sspPinMuxing[i].modefunc);
            return;
        }
    }
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


HardwareSPI::HardwareSPI(const SSP_DEVICE_T* sspDeviceDefaults) noexcept
    :sspDevice(sspDeviceDefaults), needInit(true)
{
}

void HardwareSPI::InitialiseHardware(bool master)
{
    if(master == true)
    {
        *(sspDevice->sspTransferSemaphore) = xSemaphoreCreateBinary();
    }

    if(sspDevice->sspHardwareDevice == LPC_SSP0)
    {
        Chip_Clock_SetPCLKDiv(SYSCTL_PCLK_SSP0, SYSCTL_CLKDIV_1);                       //set SPP peripheral clock to PCLK/1
        Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SSP0);                                //enable power and clocking
        if(master) AttachDMAChannelInterruptHandler(ssp0_dma_interrupt, DMA_SSP0_RX);   //attach to the RX complete DMA Interrupt handler
    }
    else if (sspDevice->sspHardwareDevice == LPC_SSP1)
    {
        Chip_Clock_SetPCLKDiv(SYSCTL_PCLK_SSP1, SYSCTL_CLKDIV_1);                       //set SPP peripheral clock to PCLK/1
        Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SSP1);                                //enable power and clocking
        if(master) AttachDMAChannelInterruptHandler(ssp1_dma_interrupt, DMA_SSP1_RX);   //attach to the RX complete DMA Interrupt handler
    }
    
    //setup pins. In master mode we don't configure SSEL
    const uint8_t numPins = (master==true)?3:4;
    for(uint8_t i=0; i<numPins; i++)
    {
        const Pin pin = sspDevice->defaultPins[i];
        ConfigureSspPin(pin);
    }
}

void HardwareSPI::FlushAll()
{
    #ifdef SSPI_DEBUG
        if( (sspDevice->sspHardwareDevice->SR & SSP_STAT_BSY) ) debugPrintf("SPI Busy\n");
        if( !(sspDevice->sspHardwareDevice->SR & SSP_STAT_TFE) ) debugPrintf("SPI Tx Not Empty\n");
        if( (sspDevice->sspHardwareDevice->SR & SSP_STAT_RNE) ) debugPrintf("SPI Rx Not Empty\n");
    #endif

    //wait for SSP to be idle
    waitForTxReady(sspDevice->sspHardwareDevice);

    //clear out the RX FIFO
    while (Chip_SSP_GetStatus(sspDevice->sspHardwareDevice, SSP_STAT_RNE))
    {
        Chip_SSP_ReceiveFrame(sspDevice->sspHardwareDevice);
    }

    flushTxFifo(sspDevice->sspHardwareDevice);
    
    Chip_SSP_ClearIntPending(sspDevice->sspHardwareDevice, SSP_INT_CLEAR_BITMASK);

}

void HardwareSPI::Enable()
{
    Chip_SSP_Enable(sspDevice->sspHardwareDevice);
}

void HardwareSPI::Disable()
{
    Chip_SSP_Disable(sspDevice->sspHardwareDevice);
}


//setup the master device.
void HardwareSPI::setup_device(const struct sspi_device *device, bool master) noexcept
{
    if(needInit)
    {
        InitialiseDMA();
        InitialiseHardware(master);
        needInit = false;
    }
    Chip_SSP_Disable(sspDevice->sspHardwareDevice);
    
    Chip_SSP_Set_Mode(sspDevice->sspHardwareDevice, (master==true)?SSP_MODE_MASTER:SSP_MODE_SLAVE);
    Chip_SSP_SetFormat(sspDevice->sspHardwareDevice, getSSPBits(device->bitsPerTransferControl), SSP_FRAMEFORMAT_SPI, getSSPMode(device->spiMode));
    Chip_SSP_SetBitRate(sspDevice->sspHardwareDevice, device->clockFrequency);
    Chip_SSP_Enable(sspDevice->sspHardwareDevice);

    FlushAll();
}

spi_status_t HardwareSPI::sspi_transceive_packet_dma(const uint8_t *tx_data, uint8_t *rx_data, size_t len, DMA_TransferWidth_t transferWidth) noexcept
{
    if(sspDevice->sspHardwareDevice == nullptr) return SPI_ERROR_TIMEOUT;//todo: just return timeout error if null
    
    uint8_t dontCareRX = 0;
    uint8_t dontCareTX = 0xFF;
    
    sspDevice->sspHardwareDevice->DMACR = SSP_DMA_TX | SSP_DMA_RX; //enable DMA

    if(rx_data != nullptr)
    {
        SspDmaRxTransfer(rx_data, len);
    }
    else
    {
        SspDmaRxTransferNI(&dontCareRX, len); //No Increment mode, dont care about received data, overwrite dontCareRX
    }
        
    if(tx_data != nullptr)
    {
        SspDmaTxTransfer(tx_data, len);
    }
    else
    {
        SspDmaTxTransferNI(&dontCareTX, len); //No Increment mode, send 0xFF
    }
    
    configASSERT(*sspDevice->sspTransferSemaphore != nullptr);

    spi_status_t ret = SPI_OK;
    const TickType_t xDelay = pdMS_TO_TICKS(SPITimeoutMillis); //timeout
    if( xSemaphoreTake(*sspDevice->sspTransferSemaphore, xDelay) == pdFALSE) // timed out or failed to take semaphore
    {
        ret = SPI_ERROR_TIMEOUT;
    }
    sspDevice->sspHardwareDevice->DMACR &= ~(SSP_DMA_RX | SSP_DMA_TX); //disable DMA

    
    return ret;
}

spi_status_t HardwareSPI::sspi_transceive_packet(const uint8_t *tx_data, uint8_t *rx_data, size_t len) noexcept
{
    
    if(sspDevice->sspHardwareDevice == nullptr) return SPI_ERROR_TIMEOUT;

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
        Chip_SSP_SendFrame(sspDevice->sspHardwareDevice, dOut);
    }
    len -= preloadFrames;
        
    //send rest of the data
    while (len)
    {
        //Read in a Frame
        while (!Chip_SSP_GetStatus(sspDevice->sspHardwareDevice, SSP_STAT_RNE));
        dIn = Chip_SSP_ReceiveFrame(sspDevice->sspHardwareDevice); //get received data from Data Register
        if(rx_data != nullptr) *rx_data++ = dIn;

        //Send the next Frame
        dOut = (tx_data == nullptr) ? 0x000000FF : (uint32_t)*tx_data++;
        Chip_SSP_SendFrame(sspDevice->sspHardwareDevice, dOut);
        
        len--;
    }
    
    //get the last remaining frames from Receive FIFO
    for (uint8_t i = 0; i < preloadFrames; i++)
    {
        while (!Chip_SSP_GetStatus(sspDevice->sspHardwareDevice, SSP_STAT_RNE));
        dIn = Chip_SSP_ReceiveFrame(sspDevice->sspHardwareDevice);
        if(rx_data != nullptr) *rx_data++ = dIn;
    }
    
    return SPI_OK;
}



void HardwareSPI::SspDmaRxTransfer(const void *buf, uint32_t transferLength, DMA_TransferWidth_t transferWidth) noexcept
{
    // Setup DMA Receive: SSP --> inBuffer (Peripheral to Memory)
    const uint8_t channelNumber = DMAGetChannelNumber(sspDevice->rxDmaChannel);
    uint8_t srcPeripheral;
    uint32_t srcAddress;
    
    if(sspDevice->rxDmaChannel == DMA_SSP0_RX)
    {
        srcPeripheral = GPDMA_CONN_SSP0_Rx;
        srcAddress = (uint32_t)&LPC_SSP0->DR;
    }
    else if(sspDevice->rxDmaChannel == DMA_SSP1_RX)
    {
        srcPeripheral = GPDMA_CONN_SSP1_Rx;
        srcAddress = (uint32_t)&LPC_SSP1->DR;
    }
    else
    {
        //invalid channel number
        return;
    }
    
    sspDevice->sspHardwareDevice->DMACR |= SSP_DMA_RX; //enable DMA

    
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

void HardwareSPI::SspDmaTxTransfer(const void *buf, uint32_t transferLength, DMA_TransferWidth_t transferWidth) noexcept
{
    // Setup DMA transfer: outBuffer --> SSP (Memory to Peripheral Transfer)
    const uint8_t channelNumber = DMAGetChannelNumber(sspDevice->txDmaChannel);
    uint8_t dstPeripheral;
    uint32_t dstAddress;
    
    if(sspDevice->txDmaChannel == DMA_SSP0_TX)
    {
        dstPeripheral = GPDMA_CONN_SSP0_Tx;
        dstAddress = (uint32_t)&LPC_SSP0->DR;
    }
    else if(sspDevice->txDmaChannel == DMA_SSP1_TX)
    {
        dstPeripheral = GPDMA_CONN_SSP1_Tx;
        dstAddress = (uint32_t)&LPC_SSP1->DR;
    }
    else
    {
        //invalid channel number
        return;
    }

    sspDevice->sspHardwareDevice->DMACR |= SSP_DMA_TX; //enable DMA
    
    
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

void HardwareSPI::SspDmaRxTransferNI(const void *buf, uint32_t transferLength, DMA_TransferWidth_t transferWidth) noexcept
{
    // Setup DMA Receive: SSP --> inBuffer (Peripheral to Memory)
    const uint8_t channelNumber = DMAGetChannelNumber(sspDevice->rxDmaChannel);
    uint8_t srcPeripheral;
    uint32_t srcAddress;
    
    if(sspDevice->rxDmaChannel == DMA_SSP0_RX)
    {
        srcPeripheral = GPDMA_CONN_SSP0_Rx;
        srcAddress = (uint32_t)&LPC_SSP0->DR;
    }
    else if(sspDevice->rxDmaChannel == DMA_SSP1_RX)
    {
        srcPeripheral = GPDMA_CONN_SSP1_Rx;
        srcAddress = (uint32_t)&LPC_SSP1->DR;
    }
    else
    {
        //invalid channel number
        return;
    }
    
    sspDevice->sspHardwareDevice->DMACR |= SSP_DMA_RX; //enable DMA
    
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

void HardwareSPI::SspDmaTxTransferNI(const void *buf, uint32_t transferLength, DMA_TransferWidth_t transferWidth) noexcept
{
    // Setup DMA transfer: outBuffer --> SSP (Memory to Peripheral Transfer)
    const uint8_t channelNumber = DMAGetChannelNumber(sspDevice->txDmaChannel);
    uint8_t dstPeripheral;
    uint32_t dstAddress;
    
    if(sspDevice->txDmaChannel == DMA_SSP0_TX)
    {
        dstPeripheral = GPDMA_CONN_SSP0_Tx;
        dstAddress = (uint32_t)&LPC_SSP0->DR;
    }
    else if(sspDevice->txDmaChannel == DMA_SSP1_TX)
    {
        dstPeripheral = GPDMA_CONN_SSP1_Tx;
        dstAddress = (uint32_t)&LPC_SSP1->DR;
    }
    else
    {
        //invalid channel number
        return;
    }

    sspDevice->sspHardwareDevice->DMACR |= SSP_DMA_TX; //enable DMA
    
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

