
#define ARDUINO_MAIN

#include "chip.h"
#include "Core.h"
#include <FreeRTOS.h>
#include "SerialUSB.h"

extern "C" void UrgentInit();
extern "C" void __libc_init_array(void);
extern "C" void AppMain();


SerialUSB Serial;

constexpr uint16_t rxBufSize = 128;
constexpr uint16_t txBufSize = 64;

//Setup the UART, ringbuffer memory in AHB RAM
__attribute__ ((used,section("AHBSRAM0"))) uint8_t uartTxMemory[txBufSize];
__attribute__ ((used,section("AHBSRAM0"))) uint8_t uartRxMemory[rxBufSize];
HardwareSerial Serial0(USART0, uartRxMemory, rxBufSize, uartTxMemory, txBufSize);

#if defined(ENABLE_UART1)
    __attribute__ ((used,section("AHBSRAM0"))) uint8_t uartTxMemory1[txBufSize];
    __attribute__ ((used,section("AHBSRAM0"))) uint8_t uartRxMemory1[rxBufSize];
    HardwareSerial Serial1(USART1, uartRxMemory1, rxBufSize, uartTxMemory1, txBufSize);
#endif

#if defined(ENABLE_UART2)
    __attribute__ ((used,section("AHBSRAM0"))) uint8_t uartTxMemory2[txBufSize];
    __attribute__ ((used,section("AHBSRAM0"))) uint8_t uartRxMemory2[rxBufSize];
    HardwareSerial Serial2(USART2, uartRxMemory2, rxBufSize, uartTxMemory2, txBufSize);
#endif

#if defined(ENABLE_UART3)
    __attribute__ ((used,section("AHBSRAM0"))) uint8_t uartTxMemory3[txBufSize];
    __attribute__ ((used,section("AHBSRAM0"))) uint8_t uartRxMemory3[rxBufSize];
    HardwareSerial Serial3(USART3, uartRxMemory3, rxBufSize, uartTxMemory3, txBufSize);
#endif



//How much memory to reserve when allocating the heap space.
//Stack size + 256 bytes pagesize and <=128 bytes for SoftwareReset data + any other code that uses malloc
constexpr uint32_t stackSize = 300;
constexpr uint32_t reserveMemory = stackSize + (256+128);


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

    //Determine the size of free memory in Main RAM
    const char * const ramend = (const char *)&_estack;
    const char * const heapend = sbrk(0);
    const uint32_t freeRam = (ramend - heapend) - reserveMemory;
    ucHeap = (uint8_t *) sbrk(freeRam); //allocate the memory so any other code using malloc etc wont corrupt our heapregion
    
    //Determine the size of free memory in AHB RAM
    const uint32_t ahb0_free = (uint32_t)&__AHB0_end - (uint32_t)&__AHB0_dyn_start;

    
    //Setup the FreeRTOS Heap5 management to use for Dynamic Memory
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
