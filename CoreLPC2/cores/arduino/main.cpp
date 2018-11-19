
#define ARDUINO_MAIN
#include "Core.h"
#include "gpio.h"

#include "lpc17xx.h"
#include "stdutils.h"

#include "USBDevice/USB.h"
#include "USBDevice/USBSerial/USBSerial.h"
#include "usb_serial.h"

#include <FreeRTOS.h>

#include "USBDevice/USBSerial/CircBuffer.h"


extern "C" void UrgentInit();
extern "C" void __libc_init_array(void);
extern "C" void AppMain();


//Create the Buffers for USB
const uint16_t rxBufSize = 256+8;
const uint16_t txBufSize = 128+8;
//make sure the buffers for USB are in AHB RAM
__attribute__ ((used,section("AHBSRAM0"))) uint8_t circularBufferTxMemory[txBufSize];
__attribute__ ((used,section("AHBSRAM0"))) uint8_t circularBufferRxMemory[rxBufSize];

//create the buffers - ok here as we can only have 1 USB object anyway
CircBuffer<uint8_t> rxbuf(rxBufSize, circularBufferRxMemory);
CircBuffer<uint8_t> txbuf(txBufSize, circularBufferTxMemory);

SerialUSB Serial(&rxbuf, &txbuf); // our wrapper object to provide Serial over USB

#include <FreeRTOS.h>
#include "task.h"

#include "board.h"


//RTOS Heap (size set in FreeRTOS config header) when using Heap4
//__attribute__ ((used,section("AHBSRAM0"))) uint8_t ucHeap[configTOTAL_HEAP_SIZE];
uint8_t ucHeap[27584+800]; // leave some extra room in main memory for stack to grow to hold software reset data...

extern "C" void Board_Init(void);
int main( void )
{

    SystemInit();
    
    UrgentInit();

    
    extern uint8_t __AHB0_block_start;
    extern uint8_t __AHB0_dyn_start;
    extern uint8_t __AHB0_end;
    
    // zero the data sections in AHB0 SRAM
    memset(&__AHB0_block_start, 0, &__AHB0_dyn_start - &__AHB0_block_start);

    //FreeRTOS Heap5 management
    uint32_t ahb0_free = (uint32_t)&__AHB0_end - (uint32_t)&__AHB0_dyn_start ;
    const HeapRegion_t xHeapRegions[] =
    {
        { ( uint8_t * ) ucHeap, sizeof(ucHeap) }, //ucHeap will be placed in main memory
        { ( uint8_t * ) &__AHB0_dyn_start, ahb0_free }, //fill the rest of AHBRAM
        { NULL, 0 } /* Terminates the array. */
    };
    /* Pass the array into vPortDefineHeapRegions(). */
    vPortDefineHeapRegions( xHeapRegions );
    
    __libc_init_array();
    
    init(); // Defined in variant.cpp

    SysTick_Init(); // also inits the LEDs for systick/vApplicationTickHook
       
    AppMain();

    return 0;
}
