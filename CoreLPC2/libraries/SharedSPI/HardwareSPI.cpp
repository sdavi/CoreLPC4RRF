//Hardware SPI
#include "HardwareSPI.h"
#include "chip.h"

#include "DMA.h"

#include "FreeRTOS.h"
#include "semphr.h"

#define SPI0_FUNCTION  PINSEL_FUNC_2 //SSP
#define SPI1_FUNCTION  PINSEL_FUNC_2

#define SPI_TIMEOUT       15000


//SSP Status Register Bits
constexpr uint8_t SR_TFE = (1<<0); //Transmit FIFO Empty. This bit is 1 is the Transmit FIFO is empty, 0 if not.
constexpr uint8_t SR_TNF = (1<<1); //Transmit FIFO Not Full. This bit is 0 if the Tx FIFO is full, 1 if not.
constexpr uint8_t SR_RNE = (1<<2); //Receive FIFO Not Empty. This bit is 0 if the Receive FIFO is empty, 1 if not
constexpr uint8_t SR_RFF = (1<<3); //Receive FIFO Full. This bit is 1 if the Receive FIFO is full, 0 if not.
constexpr uint8_t SR_BSY = (1<<4); //Busy. This bit is 0 if the SSPn controller is idle, or 1 if it is currently sending/receiving a frame and/or the Tx FIFO is not empty.


//#define SSPI_DEBUG
extern "C" void debugPrintf(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));

// Wait for transmitter ready returning true if timed out
static inline bool waitForTxReady(LPC_SSP_T* sspDevice) noexcept
{
    uint32_t timeout = SPI_TIMEOUT;
    while (sspDevice->SR & SR_BSY) //check if SSP BSY flag (1=busy)
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
    if( (sspDevice->SR & SR_BSY) ) debugPrintf("SPI Busy\n");
    if( !(sspDevice->SR & SR_TFE) ) debugPrintf("SPI Tx Not Empty\n");
    if( (sspDevice->SR & SR_RNE) ) debugPrintf("SPI Rx Not Empty\n");
#endif

    //wait for SSP to be idle
    if (waitForTxReady(sspDevice)) return true; //timed out

    //check the Receive FIFO
    if( (sspDevice->SR & SR_RNE) )
    {
        //clear out the FIFO
        while( (sspDevice->SR & SR_RNE) )
        {
            sspDevice->DR; // read
        }
    }

    Chip_SSP_ClearIntPending(sspDevice, SSP_INT_CLEAR_BITMASK);
    
    return false; //did not timeout (success)

}

//wait for SSP receive FIFO to be not empty
static inline bool waitForReceiveNotEmpty(LPC_SSP_T* sspDevice) noexcept
{
    uint32_t timeout = SPI_TIMEOUT;
    
    //Bit 2 - RNE (Receive Not Empty) - This bit is 0 if the Receive FIFO is empty
    while( !(sspDevice->SR & SR_RNE) )
    {
        if (--timeout == 0)
        {
            return true;
        }
    }
    return false;
}


//SD: added function. check if TX FIFO is not full: return true if timed out
static inline bool waitForTxEmpty_timeout(LPC_SSP_T *ssp, uint32_t timeout)  noexcept
{
    while (!(ssp->SR & (1<<1)) ) // TNF = 0 if full
    {
        if (!timeout--)
        {
            return true;
        }
    }
    return false;
    
}

//SD as ssp_readable but with timeout for sharedSPI: returns true if timed out
static inline bool ssp_readable_timeout(LPC_SSP_T *ssp, uint32_t timeout) noexcept
{
    while ( !(ssp->SR & (1 << 2)) )
    {
        if (--timeout == 0) return true;
    }
    return false;
}


// Wait for transmitter empty returning true if timed out
//static inline bool waitForTxEmpty(LPC_SSP_TypeDef* sspDevice)
bool HardwareSPI::waitForTxEmpty() noexcept
{
    return waitForTxEmpty_timeout(selectedSSPDevice, SPI_TIMEOUT);
}

// Wait for receive data available returning true if timed out
static inline bool waitForRxReady(LPC_SSP_T* sspDevice) noexcept
{
    return ssp_readable_timeout(sspDevice, SPI_TIMEOUT);
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
    HardwareSPI *s = (HardwareSPI *) getSSPDevice(SSP0);
    s->interrupt();
}

void ssp1_dma_interrupt() noexcept
{
    HardwareSPI *s = (HardwareSPI *) getSSPDevice(SSP1);
    s->interrupt();
}

void HardwareSPI::interrupt() noexcept
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
            
            GPIO_PinFunction(SPI0_SCK, SPI0_FUNCTION);   /* Configure the Pinfunctions for SPI */
            GPIO_PinFunction(SPI0_MOSI,SPI0_FUNCTION);
            GPIO_PinFunction(SPI0_MISO,SPI0_FUNCTION);
            
            GPIO_PinDirection(SPI0_SCK, OUTPUT);        /* Configure SCK,MOSI,SSEl as Output and MISO as Input */
            GPIO_PinDirection(SPI0_MOSI,OUTPUT);
            GPIO_PinDirection(SPI0_MISO,INPUT);

            AttachDMAChannelInterruptHandler(ssp0_dma_interrupt, DMA_SSP0_RX); //attach to the RX complete DMA Intettrupt handler
        }
        else if (selectedSSPDevice == LPC_SSP1)
        {
            Chip_Clock_SetPCLKDiv(SYSCTL_PCLK_SSP1, SYSCTL_CLKDIV_1); //set SPP peripheral clock to PCLK/1
            Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SSP1); //enable power and clocking
            
            GPIO_PinFunction(SPI1_SCK, SPI1_FUNCTION);   /* Configure the Pinfunctions for SPI */
            GPIO_PinFunction(SPI1_MOSI,SPI1_FUNCTION);
            GPIO_PinFunction(SPI1_MISO,SPI1_FUNCTION);
            
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

    
    //spi_format(selectedSSPDevice, device->bitsPerTransferControl, device->spiMode, 0); //Set the bits, set Mode 0, and set as Master
    //spi_frequency(selectedSSPDevice, device->clockFrequency);
    
    clearSSPTimeout(selectedSSPDevice);
}


HardwareSPI::HardwareSPI(LPC_SSP_T *sspDevice) noexcept
    :needInit(true)
{
    selectedSSPDevice = sspDevice;
    dmaTransferComplete = false;
    
    spiTransferSemaphore = xSemaphoreCreateBinary();
}

spi_status_t HardwareSPI::sspi_transceive_packet_dma(const uint8_t *tx_data, uint8_t *rx_data, size_t len, DMA_TransferWidth_t transferWidth) noexcept
{
    dmaTransferComplete = false;
    
    uint8_t dontCareRX = 0;
    uint8_t dontCareTX = 0xFF;
    
    DMA_Channel_t chanRX;
    DMA_Channel_t chanTX;
    
    if(selectedSSPDevice == nullptr) return SPI_ERROR_TIMEOUT;//todo: just return timeout error if null

    
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
    
    spi_status_t ret = SPI_OK;
    //while not complete
    //while(dmaTransferComplete == false)
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
    return sspi_transceive_packet_dma(tx_data, rx_data, len, DMA_WIDTH_BYTE);
}
