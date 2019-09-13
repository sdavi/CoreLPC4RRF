/*
 * @brief LPC175x/6x basic chip inclusion file
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2014
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#ifndef __EXTRAS_H_
#define __EXTRAS_H_


//SD:: some of the LPCOpen defs conflict with existing code, so cherrypick the ones from LPCOpen we need that arent aready defined.
#include "lpc17xx.h"
typedef void (*p_msDelay_func_t)(uint32_t);
    /* AHB peripheral */
#define LPC_ENET_BASE             0x50000000
#define LPC_SYSCTL_BASE           0x400FC000
    /* Assign LPC_* names to structures mapped to addresses */
#define LPC_ETHERNET              ((LPC_ENET_T             *) LPC_ENET_BASE)
#define LPC_SYSCTL                ((LPC_SYSCTL_T           *) LPC_SYSCTL_BASE)

    /**
     * Power control for peripherals
     */
    typedef enum CHIP_SYSCTL_CLOCK {
#if defined(CHIP_LPC175X_6X)
        SYSCTL_CLOCK_RSVD0,
#else
        SYSCTL_CLOCK_LCD,                    /*!< LCD clock */
#endif
        SYSCTL_CLOCK_TIMER0,            /*!< Timer 0 clock */
        SYSCTL_CLOCK_TIMER1,            /*!< Timer 1 clock */
        SYSCTL_CLOCK_UART0,                /*!< UART 0 clock */
        SYSCTL_CLOCK_UART1,                /*!< UART 1 clock */
#if defined(CHIP_LPC175X_6X)
        SYSCTL_CLOCK_RSVD5,
#else
        SYSCTL_CLOCK_PWM0,                /*!< PWM0 clock */
#endif
        SYSCTL_CLOCK_PWM1,                /*!< PWM1 clock */
        SYSCTL_CLOCK_I2C0,                /*!< I2C0 clock */
#if defined(CHIP_LPC175X_6X)
        SYSCTL_CLOCK_SPI,                    /*!< SPI clock */
#else
        SYSCTL_CLOCK_UART4,                /*!< UART 4 clock */
#endif
        SYSCTL_CLOCK_RTC,                    /*!< RTC clock */
        SYSCTL_CLOCK_SSP1,                /*!< SSP1 clock */
#if defined(CHIP_LPC175X_6X)
        SYSCTL_CLOCK_RSVD11,
#else
        SYSCTL_CLOCK_EMC,                    /*!< EMC clock */
#endif
        SYSCTL_CLOCK_ADC,                    /*!< ADC clock */
        SYSCTL_CLOCK_CAN1,                /*!< CAN1 clock */
        SYSCTL_CLOCK_CAN2,                /*!< CAN2 clock */
        SYSCTL_CLOCK_GPIO,                /*!< GPIO clock */
#if defined(CHIP_LPC175X_6X)
        SYSCTL_CLOCK_RIT,                /*!< RIT clock */
#else
        SYSCTL_CLOCK_SPIFI,                /*!< SPIFI clock */
#endif
        SYSCTL_CLOCK_MCPWM,                /*!< MCPWM clock */
        SYSCTL_CLOCK_QEI,                    /*!< QEI clock */
        SYSCTL_CLOCK_I2C1,                /*!< I2C1 clock */
#if defined(CHIP_LPC175X_6X)
        SYSCTL_CLOCK_RSVD20,
#else
        SYSCTL_CLOCK_SSP2,                /*!< SSP2 clock */
#endif
        SYSCTL_CLOCK_SSP0,                /*!< SSP0 clock */
        SYSCTL_CLOCK_TIMER2,            /*!< Timer 2 clock */
        SYSCTL_CLOCK_TIMER3,            /*!< Timer 3 clock */
        SYSCTL_CLOCK_UART2,                /*!< UART 2 clock */
        SYSCTL_CLOCK_UART3,                /*!< UART 3 clock */
        SYSCTL_CLOCK_I2C2,                /*!< I2C2 clock */
        SYSCTL_CLOCK_I2S,                    /*!< I2S clock */
#if defined(CHIP_LPC175X_6X)
        SYSCTL_CLOCK_RSVD28,
#else
        SYSCTL_CLOCK_SDC,                /*!< SD Card interface clock */
#endif
        SYSCTL_CLOCK_GPDMA,                /*!< GP DMA clock */
        SYSCTL_CLOCK_ENET,                /*!< EMAC/Ethernet clock */
        SYSCTL_CLOCK_USB,                    /*!< USB clock */
        SYSCTL_CLOCK_RSVD32,
        SYSCTL_CLOCK_RSVD33,
        SYSCTL_CLOCK_RSVD34,
#if defined(CHIP_LPC40XX)
        SYSCTL_CLOCK_CMP,                /*!< Comparator clock (PCONP1) */
#else
        SYSCTL_CLOCK_RSVD35,
#endif
    } CHIP_SYSCTL_CLOCK_T;
    


#include "sysctl_17xx_40xx.h"
//#include "clock_17xx_40xx.h"
#include "enet_17xx_40xx.h"
#include "gpdma_17xx_40xx.h"
#include "ssp_17xx_40xx.h"
    /* Enables power and clocking for a peripheral */
    static inline void Chip_Clock_EnablePeriphClock(CHIP_SYSCTL_CLOCK_T clk) {
        uint32_t bs = (uint32_t) clk;
        LPC_SYSCTL->PCONP |= (1 << bs);
    }

    
    /* Disables power and clocking for a peripheral */
    static inline void Chip_Clock_DisablePeriphClock(CHIP_SYSCTL_CLOCK_T clk) {
        uint32_t bs = (uint32_t) clk;
        LPC_SYSCTL->PCONP |= ~(1 << bs);
    }

#endif /* __EXTRAS_H_ */
