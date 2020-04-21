/*
 SDClass for use by RepRapFirmware
 
 Author: sdavi
 
*/
#ifndef SDCARD_H
#define SDCARD_H


#include "gpio.h"
#include "Core.h"
#include "SharedSpi.h"

#include "diskio.h" //fatfs




/* MMC card type flags (MMC_GET_TYPE) */
#define CT_MMC        0x01        /* MMC ver 3 */
#define CT_SD1        0x02        /* SD ver 1 */
#define CT_SD2        0x04        /* SD ver 2 */
#define CT_SDC        (CT_SD1|CT_SD2)    /* SD */
#define CT_BLOCK      0x08        /* Block addressing */



typedef uint8_t CARD_TYPE;

class SDCard
{
public:


    SDCard(uint8_t SSPSlot, Pin cs) noexcept;
    ~SDCard() noexcept {};
    void ReInit(Pin cs, uint32_t frequency) noexcept;
    void SetSSPChannel(SSPChannel channel) noexcept;

    CARD_TYPE card_type(void) noexcept;
    
    void unmount() noexcept;
    uint32_t interface_speed() noexcept { return frequency; };
    uint32_t disk_sectors() noexcept { return sdcardSectors; };
    uint32_t disk_blocksize() noexcept { return sdcardBlockSize; };
    uint32_t disk_highSpeedMode() noexcept {return isHighSpeed; };
    
    
    
    //DiskIO
    uint8_t disk_initialize() noexcept;
    uint8_t disk_status() noexcept;
    DRESULT disk_read (uint8_t *buff, uint32_t sector, uint32_t count) noexcept;
    DRESULT disk_write (const uint8_t *buff, uint32_t sector, uint32_t count) noexcept;
    DRESULT disk_ioctl (uint8_t cmd, void *buff) noexcept;

protected:

    int wait_ready (uint32_t wt) noexcept;
    void deselect (void) noexcept;
    int select (void) noexcept;
    int rcvr_datablock (uint8_t *buff, uint32_t btr) noexcept;
    int xmit_datablock (const uint8_t *buff, uint8_t token) noexcept;
    uint8_t send_cmd (uint8_t cmd, uint32_t arg) noexcept;

    bool enableHighSpeedMode() noexcept;
    
    //variables
    sspi_device _sspi_device;
    CARD_TYPE cardtype;
    uint32_t maxFrequency; //max frequency the user has specified
    uint32_t frequency;
    uint8_t status;
    
    
    uint32_t sdcardSectors;
    uint32_t sdcardBlockSize;
    
    bool isHighSpeed;
};

#endif
