/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Perry Hung.
 * Copyright (c) 2011, 2012 LeafLabs, LLC.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

/**
 * @file wirish/HardwareSerial.cpp
 * @brief Wirish serial port implementation.
 */

//savi: edited to ringbuffer and added DMA support for UART0

//sdavi: Note that UART0 DMA mode writes TX data in a block using DMA (no ringbuffer)
//     : and RX data is configured to be a ring buffer thats controlled by DMA
//     : the only interrupts from DMA is when the RXBuffer wraps around and when
//     : TX data block has finished sending

#include "chip.h"
#include "HardwareSerial.h"
#include "usart.h"
#include "DMA.h"


HardwareSerial Serial0(USART0);

#if defined(ENABLE_UART1)
    HardwareSerial Serial1(USART1);
#endif

#if defined(ENABLE_UART2)
    HardwareSerial Serial2(USART2);
#endif

#if defined(ENABLE_UART3)
    HardwareSerial Serial3(USART3);
#endif




constexpr uint32_t MaxBaudRate = 460800;

static void Uart0DMATxInterrupt(bool error) noexcept
{
    Serial0.TxDMAInterrupt(error);
}

static void Uart0DMARxInterrupt(bool error) noexcept
{
    Serial0.RxDMAInterrupt(error);
}

static void Uart0DmaTxTransfer(const void *buf, uint32_t transferLength) noexcept
{
    // Setup DMA transfer: outBuffer --> UART0 (Memory to Peripheral Transfer)
    const uint8_t channelNumber = DMAGetChannelNumber(DMA_UART_TX);
    uint8_t dstPeripheral = GPDMA_CONN_UART0_Tx;
    uint32_t dstAddress = (uint32_t)&LPC_UART0->THR;

    GPDMA_CH_T *pDMAchTx = (GPDMA_CH_T *) &(LPC_GPDMA->CH[channelNumber]);

    pDMAchTx->CONFIG = GPDMA_DMACCxConfig_H;                        //halt the DMA channel - Also clears the DMA FIFO
    
    LPC_GPDMA->INTTCCLEAR = (((1UL << (channelNumber)) & 0xFF));    //clear terminal count interrupt requests for Rx and Tx Channels
    LPC_GPDMA->INTERRCLR = (((1UL << (channelNumber)) & 0xFF));     //clear the error interrupt request

    
    // Enable DMA channels
    LPC_GPDMA->CONFIG = GPDMA_DMACConfig_E;
    while (!(LPC_GPDMA->CONFIG & GPDMA_DMACConfig_E)) {}
    
    pDMAchTx->SRCADDR  = (uint32_t)buf;             //Source Address (buffer in memory)
    pDMAchTx->DESTADDR = dstAddress;                //Destination Address (UART DataRegister)
    pDMAchTx->LLI      = 0;                         //linked list item register
    pDMAchTx->CONTROL  = (transferLength & 0xFFF)
                            | GPDMA_DMACCxControl_SBSize(GPDMA_BSIZE_1)     //Source burst size set to 1
                            | GPDMA_DMACCxControl_DBSize(GPDMA_BSIZE_1)     //Destination burst size set to 1
                            | GPDMA_DMACCxControl_SWidth(DMA_WIDTH_BYTE)    //Source Tranfser width set to 1 byte
                            | GPDMA_DMACCxControl_DWidth(DMA_WIDTH_BYTE)    //Destination Transfer width set to 1 byte
                            | GPDMA_DMACCxControl_SI                        //Source increment after each transfer
                            | GPDMA_DMACCxControl_I;                        //Terminal count interrupt enable

    //Enable Channel
    pDMAchTx->CONFIG = GPDMA_DMACCxConfig_E                                                             //Enable
                            | GPDMA_DMACCxConfig_DestPeripheral(dstPeripheral)                          //Peripheral destination is SSP TX
                            | GPDMA_DMACCxConfig_TransferType(GPDMA_TRANSFERTYPE_M2P_CONTROLLER_DMA)    //Memory to Peripheral
                            | GPDMA_DMACCxConfig_IE                                                     //Interrupt Error Mask
                            | GPDMA_DMACCxConfig_ITC;                                                   //Terminal count interrupt mask

    
}

static DMA_TransferDescriptor_t uart0rxRingBufferDescriptor;
static void Uart0DmaRxRingTransfer(const void *buf, uint32_t transferLength) noexcept
{
    // Setup DMA Receive: UART0 --> inBuffer (Peripheral to Memory)
    const uint8_t channelNumber = DMAGetChannelNumber(DMA_UART_RX);
    uint8_t srcPeripheral = GPDMA_CONN_UART0_Rx;
    uint32_t srcAddress = (uint32_t)&LPC_UART0->RBR;

    GPDMA_CH_T *pDMAchRx = (GPDMA_CH_T *) &(LPC_GPDMA->CH[channelNumber]);

    pDMAchRx->CONFIG = GPDMA_DMACCxConfig_H;                                        //halt the DMA channel - Clears DMA FIFO
     
    LPC_GPDMA->INTTCCLEAR = (((1UL << (channelNumber)) & 0xFF));                    //clear terminal count interrupt requests for Rx and Tx Channels
    LPC_GPDMA->INTERRCLR = (((1UL << (channelNumber)) & 0xFF));                     //clear the error interrupt request

    uart0rxRingBufferDescriptor.src = srcAddress;
    uart0rxRingBufferDescriptor.dst = (uint32_t)buf;
    uart0rxRingBufferDescriptor.lli = (uint32_t) &uart0rxRingBufferDescriptor;                  //next in list is this again (circular)
    uart0rxRingBufferDescriptor.ctrl = (transferLength & 0xFFF)
                                            | GPDMA_DMACCxControl_SBSize(GPDMA_BSIZE_1)         //Source burst size set to 1
                                            | GPDMA_DMACCxControl_DBSize(GPDMA_BSIZE_1)         //Destination burst size set to 1
                                            | GPDMA_DMACCxControl_SWidth(DMA_WIDTH_BYTE)        //Source Tranfser width set to 1 byte
                                            | GPDMA_DMACCxControl_DWidth(DMA_WIDTH_BYTE)        //Destination Transfer width set to 1 byte
                                            | GPDMA_DMACCxControl_DI                            //Destination increment after each transfer
                                            | GPDMA_DMACCxControl_I;                            //Terminal count interrupt enable
    
     
    // Enable DMA channels
    LPC_GPDMA->CONFIG = GPDMA_DMACConfig_E;
    while (!(LPC_GPDMA->CONFIG & GPDMA_DMACConfig_E)) {}

    pDMAchRx->SRCADDR  = uart0rxRingBufferDescriptor.src;
    pDMAchRx->DESTADDR = uart0rxRingBufferDescriptor.dst;
    pDMAchRx->CONTROL  = uart0rxRingBufferDescriptor.ctrl;
    pDMAchRx->LLI      = (uint32_t) &uart0rxRingBufferDescriptor;  //linked list item register
    
     
    pDMAchRx->CONFIG =  GPDMA_DMACCxConfig_E                                                        //Enable
                         | GPDMA_DMACCxConfig_SrcPeripheral(srcPeripheral)                          //Peripheral destination is SSP0 RX
                         | GPDMA_DMACCxConfig_TransferType(GPDMA_TRANSFERTYPE_P2M_CONTROLLER_DMA)   //Peripheral to Memory
                         | GPDMA_DMACCxConfig_IE                                                    //Interrupt Error Mask
                         | GPDMA_DMACCxConfig_ITC;                                                  //Terminal count interrupt mask

     
}



HardwareSerial::HardwareSerial(const usart_dev *usart_device, uint16_t rxRingBufferSize, uint16_t txRingBufferSize) noexcept
    :   usartDevice(usart_device),
        rxBufferSize(rxRingBufferSize),
        txBufferSize(txRingBufferSize),
        rxDataBuffer(nullptr),
        txDataBuffer(nullptr),
        usingDMA(false),
        initialised(false),
        tail(0),
        txPending(false)
{
    if(usartDevice == USART0)
    {
        usingDMA = true;
        InitialiseDMA();
        pDMAchRx = (GPDMA_CH_T *) &(LPC_GPDMA->CH[DMAGetChannelNumber(DMA_UART_RX)]); //pointer to the GPDMA channel we are using for RX (needed to read the current head position in the ringbuffer)
        AttachDMAChannelInterruptHandler(Uart0DMATxInterrupt, DMA_UART_TX); //attach to the TX complete DMA Interrupt handler
        AttachDMAChannelInterruptHandler(Uart0DMARxInterrupt, DMA_UART_RX); //attach to the RX DMA Interrupt handler
    }
}

inline void HardwareSerial::TxDMAInterrupt(bool error)
{
    txPending = false;
}

//The RX DMA interrupt is only called when its reached the end of the buffer and
//is wrapping back to the start
inline void HardwareSerial::RxDMAInterrupt(bool error)
{
    //TODO:: we could use the int to track the wrap arounds to detect if overruns have occured
    //    :: currently we assume that RRF reads bytes in before an overrun occurs
}


void HardwareSerial::SetRingBufferSizes(uint16_t rxRingBufferSize, uint16_t txRingBufferSize) noexcept
{
    rxBufferSize = rxRingBufferSize;
    txBufferSize = txRingBufferSize;
}

inline size_t HardwareSerial::GetRingBufferHead()
{
    //DMA we read the position the DMA is currently set to
    return (pDMAchRx->DESTADDR - (uint32_t)rxDataBuffer);
}

void HardwareSerial::begin(uint32_t baud) noexcept
{
    if(initialised == false)
    {
        initialised = true;

        rxDataBuffer = new uint8_t[rxBufferSize];
        txDataBuffer = new uint8_t[txBufferSize];

        if(!usingDMA)
        {
            RingBuffer_Init(&this->rxRingBuffer, rxDataBuffer, 1, rxBufferSize);
            RingBuffer_Init(&this->txRingBuffer, txDataBuffer, 1, txBufferSize);

            //Reset Ringbuffers
            RingBuffer_Flush(&this->rxRingBuffer);
            RingBuffer_Flush(&this->txRingBuffer);
        }
    }
    
    
    if (baud > MaxBaudRate)
    {
       return;
    }

    usart_init(usartDevice, baud, !usingDMA);
    
    if(usingDMA)
    {
        //Enable DMA bit (note: fifos must be enabled to use DMA mode)
        usartDevice->UARTx->FCR |= UART_FCR_DMAMODE_SEL;

        tail = 0;
        //Kick off the RX DMA ring transfers
        Uart0DmaRxRingTransfer(rxDataBuffer, rxBufferSize);
    }
}

void HardwareSerial::end(void) noexcept
{
    if(usingDMA)
    {
        pDMAchRx->CONFIG = GPDMA_DMACCxConfig_H;  //halt the DMA channel - Clears DMA FIFO
    }
    else
    {
        NVIC_DisableIRQ(usartDevice->irq_NUM);
        Chip_UART_IntDisable(usartDevice->UARTx, (UART_IER_RBRINT | UART_IER_RLSINT));
    }
}

int HardwareSerial::read(void) noexcept
{
    if(usingDMA)
    {
        if(GetRingBufferHead() == tail) return -1; //buffer is empty

        const uint8_t data = rxDataBuffer[tail++];
        tail = tail % rxBufferSize; //wrap around if at the end of the buffer
        return data;
    }
    else
    {
        //read from the Ring Buffer
        int data;
        if(RingBuffer_Pop(&this->rxRingBuffer, &data))
        {
            return data;
        }
    }
    
    return -1;
}

int HardwareSerial::peek(void) noexcept
{
    if(usingDMA)
    {
        if(GetRingBufferHead() == tail) return -1; //buffer empty
        return rxDataBuffer[tail];
    }
    else
    {
        int data;
        if(RingBuffer_Peek(&this->rxRingBuffer, &data))
        {
            return data;
        }
    }
    return -1;
}

int HardwareSerial::available(void) noexcept
{
    if(usingDMA)
    {
        const uint16_t count = (rxBufferSize + GetRingBufferHead() - tail) % rxBufferSize;
        return count;
    }
    else
    {
        const int avail = RingBuffer_GetCount(&this->rxRingBuffer);
        return avail;
    }
    return 0;
}

int HardwareSerial::availableForWrite(void) noexcept
{
    return canWrite();
}

size_t HardwareSerial::canWrite() noexcept
{
    if(usingDMA)
    {
        if(txPending == true) return 0;
        //DMA is used which uses its own buffer
        return txBufferSize;
    }
    else
    {
        int avail = RingBuffer_GetFree(&this->txRingBuffer);
        return avail;
    }
}

size_t HardwareSerial::write(const uint8_t ch) noexcept
{
    if(usingDMA)
    {
        const uint8_t *buf = &ch;
        return write(buf, 1);
    }
    else
    {
        Chip_UART_SendRB(usartDevice->UARTx, &this->txRingBuffer, &ch, 1);
        return 1;
    }
}


size_t HardwareSerial::write(const uint8_t *buffer, size_t size) noexcept
{
    if(usingDMA)
    {
        if(txPending == true) return 0;
        
        const size_t bytesToWrite = min<size_t>(txBufferSize, size);
        memcpy(txDataBuffer, buffer, bytesToWrite);
        
        txPending = true;
        
        Uart0DmaTxTransfer(txDataBuffer, bytesToWrite); //kick off the Tx DMA Transfer
        return bytesToWrite;
    }
    else
    {
        size_t ret = size;
        while (size != 0)
        {
            size_t written = Chip_UART_SendRB(usartDevice->UARTx, &this->txRingBuffer, buffer, size);
            buffer += written;
            size -= written;
        }
        return ret;

    }
}

void HardwareSerial::flush(void) noexcept
{
    if(usingDMA)
    {
        while(txPending == true);
    }
    else
    {
        while(RingBuffer_GetFree(&this->txRingBuffer) > 0); //wait for the ring buffer to empty
    }
}

void HardwareSerial::IRQHandler() noexcept
{
    if(!usingDMA)
    {
        //call the LPCOpen Interrupt Handler
        Chip_UART_IRQRBHandler(usartDevice->UARTx, &this->rxRingBuffer, &this->txRingBuffer);
    }
}



//compat with RRF
void HardwareSerial::setInterruptPriority(uint32_t priority) noexcept
{
    NVIC_SetPriority(usartDevice->irq_NUM, priority);
}

uint32_t HardwareSerial::getInterruptPriority() noexcept
{
    return NVIC_GetPriority(usartDevice->irq_NUM);
}
