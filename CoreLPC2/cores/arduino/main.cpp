
#define ARDUINO_MAIN
#include "Core.h"
#include "gpio.h"

#include "lpc17xx.h"
#include "stdutils.h"

#include <FreeRTOS.h>

#include "USBDevice/USB.h"
#include "usb_serial.h"


extern "C" void UrgentInit();
extern "C" void __libc_init_array(void);
extern "C" void AppMain();


//Create the Buffers for USB
const uint16_t rxBufSize = 256;
const uint16_t txBufSize = 128;
//make sure the buffers for USB are in AHB RAM
__attribute__ ((used,section("AHBSRAM0"))) uint8_t circularBufferTxMemory[txBufSize+8] __attribute__ ( ( aligned( 8 ) ) );;
__attribute__ ((used,section("AHBSRAM0"))) uint8_t circularBufferRxMemory[rxBufSize+8] __attribute__ ( ( aligned( 8 ) ) );;

//create the buffers - ok here as we can only have 1 USB object anyway
CircBuffer<uint8_t> rxbuf(rxBufSize, circularBufferRxMemory);
CircBuffer<uint8_t> txbuf(txBufSize, circularBufferTxMemory);
SerialUSB Serial(&rxbuf, &txbuf); // our wrapper object to provide Serial over USB

//Setup the UART, ringbuffer memory in AHB RAM (same sizes as USB buffers)
__attribute__ ((used,section("AHBSRAM0"))) uint8_t uartTxMemory[txBufSize] __attribute__ ( ( aligned( 8 ) ) );;
__attribute__ ((used,section("AHBSRAM0"))) uint8_t uartRxMemory[rxBufSize] __attribute__ ( ( aligned( 8 ) ) );;
HardwareSerial Serial0(USART0, uartRxMemory, rxBufSize, uartTxMemory, txBufSize);



//How much memory to reserve when allocating the heap space.
//Stack size + extra 256 for SoftwareReset data + any other code that uses malloc
const uint32_t reserveMemory = 400+256;


__attribute__ ((used)) uint8_t *ucHeap;


// these are defined in the linker script
extern "C" uint32_t _estack;
extern uint8_t __AHB0_block_start;
extern uint8_t __AHB0_dyn_start;
extern uint8_t __AHB0_end;

extern "C" char *sbrk(int i);


extern "C" void Board_Init(void);
int main( void )
{

    SystemInit();
    
    UrgentInit();

    // zero the data sections in AHB0 SRAM
    memset(&__AHB0_block_start, 0, &__AHB0_dyn_start - &__AHB0_block_start);

    //Determine the size of memory we can allocate in Main RAM
    const char * const ramend = (const char *)&_estack;
    const char * const heapend = sbrk(0);
    const uint32_t freeRam = (ramend - heapend) - reserveMemory;
    ucHeap = (uint8_t *) malloc(freeRam); //allocate the memory so any other code using malloc etc wont corrupt our heapregion
    
    //Determine the size of memory we can use in AHB RAM
    uint32_t ahb0_free = (uint32_t)&__AHB0_end - (uint32_t)&__AHB0_dyn_start ; //Free AHB RAM (we dont need to allocate as there is no dynamic memory being allocated in this location)

    
    //Setup the FreeRTOS Heap5 management
    //Heap5 allows us to span heap memory across non-contigious blocks of memory.
    const HeapRegion_t xHeapRegions[] =
    {
        { ( uint8_t * ) ucHeap, freeRam }, //ucHeap will be placed in main memory
        { ( uint8_t * ) &__AHB0_dyn_start, ahb0_free }, //fill the rest of AHBRAM
        { NULL, 0 } /* Terminates the array. */
    };
    /* Pass the array into vPortDefineHeapRegions(). */
    vPortDefineHeapRegions( xHeapRegions );
    
    //now init all static constructors etc ... 
    __libc_init_array();
 
    
    init(); // Defined in variant.cpp

    SysTick_Init(); 
    
    AppMain();

    return 0;
}
