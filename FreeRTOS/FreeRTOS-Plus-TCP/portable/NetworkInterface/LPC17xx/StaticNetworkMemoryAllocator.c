//SD: functions to support allocation from static memory rather than dynamically



#include "StaticNetworkMemoryAllocator.h"


/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* FreeRTOS+TCP includes. */
#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"
#include "FreeRTOS_IP_Private.h"
#include "NetworkBufferManagement.h"
#include "NetworkInterface.h"




//This assumes RX and TX Buffers are the same size (1*MSS)
#define NumberSockets 2
#define MemBlocks (ipconfigTCPSOCKET_NUMBER_RX_BUFFERS+ipconfigTCPSOCKET_NUMBER_TX_BUFFERS) * NumberSockets

//SD:: Create a static array of the same data size requested from FreeRTOS_Sockets.c assusing 1*MSS buffer size

//Following is in FreeRTOS_Sockets.c
//StreamBuffer_t *pxBuffer
//uxLength += sizeof( size_t );
//uxLength &= ~( sizeof( size_t ) - 1u );
//uint8_t ucArray[ sizeof( size_t ) ];
//uxSize = sizeof( *pxBuffer ) - sizeof( pxBuffer->ucArray ) + uxLength;

//Create the size to match the above:
#define uxLength ((ipconfigTCP_MSS + sizeof( size_t )) & (~( sizeof( size_t ) - 1u )))
#define TCPDataSizeRequested sizeof( StreamBuffer_t ) - sizeof( 1*(sizeof(size_t) ) ) + uxLength


struct dataBlock_t{
    uint8_t taken;
    uint8_t data[TCPDataSizeRequested];
};

static struct dataBlock_t tcpData[MemBlocks] = {0};




//very simple Memory Allocator, allocates a block
void *staticMallocLarge( size_t xWantedSize )
{
    void * ret = NULL;
    vTaskSuspendAll();
    {
        for(size_t i=0; i<MemBlocks; i++){
            if(tcpData[i].taken == 0){
                tcpData[i].taken = 1;
                ret = tcpData[i].data;
                break;
            }
        }
    }
    ( void ) xTaskResumeAll();
    return ret;
}


void staticFreeLarge( void *blkPtr )
{

    if(blkPtr != NULL)
    {
        vTaskSuspendAll();
        {
            for(size_t i=0; i<MemBlocks; i++){
                if(blkPtr == tcpData[i].data){
                    tcpData[i].taken = 0;
                }
            }
        }
        ( void ) xTaskResumeAll();
    }
}

