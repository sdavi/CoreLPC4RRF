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


void ssp0_dma_interrupt() noexcept
{
    ssp0.interrupt();
}

void ssp1_dma_interrupt() noexcept
{
    ssp1.interrupt();
}

inline void HardwareSPI::interrupt() noexcept
{
    dmaTransferComplete = true;

    BaseType_t mustYield=false;
    xSemaphoreGiveFromISR(spiTransferSemaphore, &mustYield);
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


HardwareSPI::HardwareSPI(LPC_SSP_T *sspDevice) noexcept
    :selectedSSPDevice(sspDevice), needInit(true), dmaTransferComplete(false)
{
    spiTransferSemaphore = xSemaphoreCreateBinary();
}

spi_status_t HardwareSPI::sspi_transceive_packet_dma(const uint8_t *tx_data, uint8_t *rx_data, size_t len, DMA_TransferWidth_t transferWidth) noexcept
{
    if(selectedSSPDevice == nullptr) return SPI_ERROR_TIMEOUT;//todo: just return timeout error if null

    dmaTransferComplete = false;
    
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
    
    configASSERT(spiTransferSemaphore != nullptr);
    spi_status_t ret = SPI_OK;
    const TickType_t xDelay = SPITimeoutMillis / portTICK_PERIOD_MS; //timeout
    if( xSemaphoreTake(spiTransferSemaphore, xDelay) == pdFALSE) // timed out or failed to take semaphore
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
