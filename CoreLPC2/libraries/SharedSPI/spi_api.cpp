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
#include "spi_api.h"

#include <math.h>

#define SPI0_FUNCTION  PINSEL_FUNC_2 //SD updated to use SSP instead of SPI
#define SPI1_FUNCTION  PINSEL_FUNC_2

void spi_free(spi_t *obj) {}

static inline int ssp_disable(spi_t *obj);
static inline int ssp_enable(spi_t *obj);

void spi_format(spi_t *obj, int bits, int mode, int slave) {
    ssp_disable(obj);
    if (!(bits >= 4 && bits <= 16) || !(mode >= 0 && mode <= 3)) {
        //error("SPI format error");
        return;// TODO:
    }

    int polarity = (mode & 0x2) ? 1 : 0;
    int phase = (mode & 0x1) ? 1 : 0;

    // set it up
    int DSS = bits - 1;            // DSS (data select size)
    int FRF = 0;                   // FRF (frame format) = SPI
    int SPO = (polarity) ? 1 : 0;  // SPO - clock out polarity
    int SPH = (phase) ? 1 : 0;     // SPH - clock out phase

    uint32_t tmp = obj->CR0;
    tmp &= ~(0xFFFF);
    tmp |= DSS << 0
        | FRF << 4
        | SPO << 6
        | SPH << 7;
    obj->CR0 = tmp;

    tmp = obj->CR1;
    tmp &= ~(0xD);
    tmp |= 0 << 0                   // LBM - loop back mode - off
        | ((slave) ? 1 : 0) << 2   // MS - master slave mode, 1 = slave
        | 0 << 3;                  // SOD - slave output disable - na
    obj->CR1 = tmp;

    ssp_enable(obj);
}

void spi_frequency(spi_t *obj, int hz) {
    ssp_disable(obj);

    // setup the spi clock diveder to /1
    if( obj == LPC_SSP0){
        LPC_SC->PCLKSEL1 &= ~(3 << 10);
        LPC_SC->PCLKSEL1 |=  (1 << 10);
    }else if( obj == LPC_SSP0){
        LPC_SC->PCLKSEL0 &= ~(3 << 20);
        LPC_SC->PCLKSEL0 |=  (1 << 20);
    }

    uint32_t PCLK = SystemCoreClock;
    int prescaler;

    for (prescaler = 2; prescaler <= 254; prescaler += 2) {
        int prescale_hz = PCLK / prescaler;

        // calculate the divider
        int divider = floor(((float)prescale_hz / (float)hz) + 0.5);

        // check we can support the divider
        if (divider < 256) {
            // prescaler
            obj->CPSR = prescaler;

            // divider
            obj->CR0 &= ~(0xFFFF << 8);
            obj->CR0 |= (divider - 1) << 8;
            ssp_enable(obj);
            return;
        }
    }
    //error("Couldn't setup requested SPI frequency");
}

static inline int ssp_disable(spi_t *obj) {
    return obj->CR1 &= ~(1 << 1);
}

static inline int ssp_enable(spi_t *obj) {
    return obj->CR1 |= (1 << 1);
}

static inline int ssp_readable(spi_t *obj) {
    return obj->SR & (1 << 2);
}


static inline int ssp_writeable(spi_t *obj) {
    return obj->SR & (1 << 1);
}

static inline void ssp_write(spi_t *obj, int value) {
    while (!ssp_writeable(obj));
    obj->DR = value;
}

static inline int ssp_read(spi_t *obj) {
    while (!ssp_readable(obj));
    return obj->DR;
}


static inline int ssp_busy(spi_t *obj) {
    return (obj->SR & (1 << 4)) ? (1) : (0);
}

int spi_master_write(spi_t *obj, int value) {
    ssp_write(obj, value);
    return ssp_read(obj);
}

int spi_slave_receive(spi_t *obj) {
    return (ssp_readable(obj) && !ssp_busy(obj)) ? (1) : (0);
};

int spi_slave_read(spi_t *obj) {
    return obj->DR;
}

void spi_slave_write(spi_t *obj, int value) {
    while (ssp_writeable(obj) == 0) ;
    obj->DR = value;
}

int spi_busy(spi_t *obj) {
    return ssp_busy(obj);
}


