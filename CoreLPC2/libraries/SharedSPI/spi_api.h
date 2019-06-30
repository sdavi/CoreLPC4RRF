/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef MBED_SPI_API_H
#define MBED_SPI_API_H

#include "Core.h"

#define SPI_TIMEOUT       15000


#ifdef __cplusplus


extern "C" {
#endif

typedef LPC_SSP_TypeDef spi_t;
    
void spi_init(spi_t *obj, uint8_t SSPChannel);

void spi_free         (spi_t *obj);
void spi_format       (spi_t *obj, int bits, int mode, int slave);
void spi_frequency    (spi_t *obj, int hz);
int  spi_master_write (spi_t *obj, int value);
int  spi_slave_receive(spi_t *obj);
int  spi_slave_read   (spi_t *obj);
void spi_slave_write  (spi_t *obj, int value);
int  spi_busy         (spi_t *obj);

//SD: added function. check if TX FIFO is not full: return true if timed out
static inline bool waitForTxEmpty_timeout(LPC_SSP_TypeDef *ssp, uint32_t timeout) {
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
static inline bool ssp_readable_timeout(LPC_SSP_TypeDef *ssp, uint32_t timeout) {
    while ( !(ssp->SR & (1 << 2)) )
    {
        if (--timeout == 0) return true;
    }
    return false;
}
    

    
#ifdef __cplusplus
}
#endif

#endif
