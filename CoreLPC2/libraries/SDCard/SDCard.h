/* mbed SDFileSystem Library, for providing file access to SD cards
 * Copyright (c) 2008-2010, sford
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 *
 * This version significantly altered by Michael Moon and is (c) 2012
 * SD: Modified to work with RRF
 
 */

#ifndef SDCARD_H
#define SDCARD_H

#include "gpio.h"
#include "Core.h"
#include "SharedSpi.h"

class SDCard {
public:

    /** Create the File System for accessing an SD Card using SPI
     *
     * @param SSPSlot SSP slot to use
     * @param cs   DigitalOut pin used as SD Card chip select
     * @param name The name used to access the virtual filesystem
     */
    SDCard(uint8_t SSPSlot, Pin cs);
    ~SDCard() {};

    void ReInit(Pin cs, uint32_t frequency);

    
    
    typedef enum {
        SDCARD_FAIL,
        SDCARD_V1,
        SDCARD_V2,
        SDCARD_V2HC
    } CARD_TYPE;

    int disk_initialize();
    int disk_write(const uint8_t *buffer, uint32_t block_number);
    int disk_read(uint8_t *buffer, uint32_t block_number);
    int disk_status();
    int disk_sync();
    uint32_t disk_sectors();
    uint64_t disk_size();
    uint32_t disk_blocksize();
    bool disk_canDMA(void);

    CARD_TYPE card_type(void);

    bool busy();
    
    uint32_t interface_speed(){return frequency;};

protected:

    int _cmd(int cmd, uint32_t arg);
    int _cmdx(int cmd, uint32_t arg);
    int _cmd8();
    int _cmd58(uint32_t*);
    CARD_TYPE initialise_card();
    CARD_TYPE initialise_card_v1();
    CARD_TYPE initialise_card_v2();

    int _read(uint8_t *buffer, int length);
    int _write(const uint8_t *buffer, int length);

    uint32_t _sd_sectors();
    uint32_t _sectors;

    sspi_device _sspi_device;
    volatile bool busyflag;

    CARD_TYPE cardtype;
    
    uint32_t frequency;
};

#endif
