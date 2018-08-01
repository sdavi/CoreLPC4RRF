//Implement the SharedSpi as in RRF

#include "Core.h"
#include "lpc17xx.h"
#include "SharedSpi.h"
#include "variant.h"




#define SPI0_FUNCTION  PINSEL_FUNC_2 //SSP
#define SPI1_FUNCTION  PINSEL_FUNC_2


// Lock for the SPI subsystem
static bool sspiLocked = false;

// Acquire the SSP bus
// Returns true if successfully acquired

bool sspi_acquire(const sspi_device *device)
{
	irqflags_t flags = cpu_irq_save();
	bool rslt;
	if (sspiLocked)
	{
		rslt = false;
	}
	else
	{
		sspiLocked = true;
		rslt = true;
	}
	cpu_irq_restore(flags);
	return rslt;
}

// Release the SSP bus
void sspi_release(const sspi_device *device)
{
	sspiLocked = false;
}

static inline LPC_SSP_TypeDef *getSSPDevice(SSPChannel channel){
    if(channel == SSP0) return LPC_SSP0;
    else return LPC_SSP1;
}


// Wait for transmitter ready returning true if timed out
static inline bool waitForTxReady(LPC_SSP_TypeDef* sspDevice)
{
	uint32_t timeout = SPI_TIMEOUT;
    while (util_IsBitSet(sspDevice->SR, 4)) //check if SSP BSY flag (1=busy)
	{
		if (--timeout == 0)
		{
			return true;
		}
	}
	return false;
}



// Wait for transmitter empty returning true if timed out
static inline bool waitForTxEmpty(LPC_SSP_TypeDef* sspDevice)
{
    return waitForTxEmpty_timeout(sspDevice, SPI_TIMEOUT);
}

// Wait for receive data available returning true if timed out
static inline bool waitForRxReady(LPC_SSP_TypeDef* sspDevice)
{
    return ssp_readable_timeout(sspDevice, SPI_TIMEOUT);
}


// Set up the Shared SPI subsystem
void sspi_master_init(struct sspi_device *device, uint32_t bits)
{
    GPIO_PinFunction(device->csPin, 0); //set pin to GPIO
    pinMode(device->csPin, (device->csPolarity) ? OUTPUT_LOW : OUTPUT_HIGH);


    //Only support 8 or 16bit
    if(bits == 8 || bits == 16){
        device->bitsPerTransferControl = bits;
    } else {
        device->bitsPerTransferControl = 8; // default
    }
}


//setup the master device.
void sspi_master_setup_device(const struct sspi_device *device)
{
    

    static bool init_ssp0 = true;
    static bool init_ssp1 = true;

    if(init_ssp0 && device->sspChannel == SSP0)
    {
        
        LPC_SC->PCONP |= 1 << 21;//enable Power and clocking
        
        GPIO_PinFunction(SPI0_SCK,  SPI0_FUNCTION);   /* Configure the Pinfunctions for SPI */
        GPIO_PinFunction(SPI0_MOSI, SPI0_FUNCTION);
        GPIO_PinFunction(SPI0_MISO, SPI0_FUNCTION);
        
        GPIO_PinDirection(SPI0_SCK,OUTPUT);        /* Configure SCK,MOSI,SSEl as Output and MISO as Input */
        GPIO_PinDirection(SPI0_MOSI,OUTPUT);
        GPIO_PinDirection(SPI0_MISO,INPUT);
        
        init_ssp0 = false;

    } else if (init_ssp1 && device->sspChannel == SSP1){
        
        LPC_SC->PCONP |= 1 << 10;//enable Power and clocking
        
        GPIO_PinFunction(SPI1_SCK,SPI1_FUNCTION);   /* Configure the Pinfunctions for SPI */
        GPIO_PinFunction(SPI1_MOSI,SPI1_FUNCTION);
        GPIO_PinFunction(SPI1_MISO,SPI1_FUNCTION);
        
        GPIO_PinDirection(SPI1_SCK,OUTPUT);        /* Configure SCK,MOSI,SSEl as Output and MISO as Input */
        GPIO_PinDirection(SPI1_MOSI,OUTPUT);
        GPIO_PinDirection(SPI1_MISO,INPUT);

        init_ssp1 = false;

        }

    spi_format(getSSPDevice(device->sspChannel), device->bitsPerTransferControl, device->spiMode, 0); //Set the bits, set Mode 0, and set as Master
    spi_frequency(getSSPDevice(device->sspChannel), device->clockFrequency);
 
}


LPC_SSP_TypeDef* selectedSSPDevice = nullptr;
// select device to use the SSP bus
void sspi_select_device(const struct sspi_device *device)
{
	// Enable the CS line
    if(device->csPolarity == false){
        digitalWrite(device->csPin, LOW); //active low
    } else {
        digitalWrite(device->csPin, HIGH);// active high
    }
    //since transreceive doesnt pass the device struct, we track it here
    //We are assuming only 1 connection used at a time even though there are 2 busses
    selectedSSPDevice = getSSPDevice(device->sspChannel);
}

// deselect
void sspi_deselect_device(const struct sspi_device *device)
{
	if(selectedSSPDevice != nullptr) waitForTxEmpty(selectedSSPDevice);

	// Disable the CS line
    if(device->csPolarity == false){
        digitalWrite(device->csPin, HIGH);
    } else {
        digitalWrite(device->csPin, LOW);
    }
    selectedSSPDevice = nullptr;

}


//send and receive len bytes from the selected SPI device (use spi_select_device before using).
spi_status_t sspi_transceive_packet(const uint8_t *tx_data, uint8_t *rx_data, size_t len)
{

    if(selectedSSPDevice == nullptr) return SPI_ERROR_TIMEOUT;//todo: just return timeout error if null
    
    for (uint32_t i = 0; i < len; ++i)
	{
        
        
		uint32_t dOut = (tx_data == nullptr) ? 0x000000FF : (uint32_t)*tx_data++;

        //Wait for Transmit to be ready
        if (waitForTxReady(selectedSSPDevice))
		{
			return SPI_ERROR_TIMEOUT;
		}
        
        selectedSSPDevice->DR = dOut; //put data into the Data Registor to transmit
        
        // Some devices are transmit-only e.g. 12864 display, so don't wait for received data if we don't need to
        if (rx_data != nullptr)
        {

            // Wait for receive to be ready
            if (waitForRxReady(selectedSSPDevice))
            {
                return SPI_ERROR_TIMEOUT;
            }
        
        
            uint8_t dIn = selectedSSPDevice->DR; //get received data from Data Registor
            *rx_data++ = dIn;
        }
	}
    
    // If we didn't wait to receive, then we need to wait for transmit to finish and clear the receive buffer and
    if (rx_data == nullptr)
    {
        waitForTxEmpty(selectedSSPDevice);
    }


	return SPI_OK;
}


//TODO:: temp.....for SDCard
int sspi_transceive_a_packet(int buf)
{
    uint8_t tx[1], rx[1];
    tx[0] = (uint8_t)buf;
    sspi_transceive_packet(tx, rx, 1);
    return (int) rx[0];
}

