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

#ifndef __CHIP_LPC175X_6X_H_
#define __CHIP_LPC175X_6X_H_

#include "lpc_types.h"
//#include "sys_config.h"
//#include "cmsis.h"



#ifdef __cplusplus
extern "C" {
#endif

#if !defined(CORE_M3)
#error CORE_M3 is not defined for the LPC175x/6x architecture
#error CORE_M3 should be defined as part of your compiler define list
#endif

#ifndef CHIP_LPC175X_6X
#error CHIP_LPC175X_6X is not defined!
#endif


    typedef void (*p_msDelay_func_t)(uint32_t);

    
    /* AHB peripheral */
#define LPC_ENET_BASE             0x50000000

/* Assign LPC_* names to structures mapped to addresses */
#define LPC_ETHERNET              ((LPC_ENET_T             *) LPC_ENET_BASE)

#include "enet_17xx_40xx.h"


/**
 * @}
 */
/*
#include "sysctl_17xx_40xx.h"
#include "clock_17xx_40xx.h"
#include "iocon_17xx_40xx.h"
#include "adc_17xx_40xx.h"
#include "can_17xx_40xx.h"
#include "dac_17xx_40xx.h"
#include "enet_17xx_40xx.h"
#include "gpdma_17xx_40xx.h"
#include "gpio_17xx_40xx.h"
#include "gpioint_17xx_40xx.h"
#include "i2c_17xx_40xx.h"
#include "i2s_17xx_40xx.h"
#include "mcpwm_17xx_40xx.h"
#include "pmu_17xx_40xx.h"
#include "qei_17xx_40xx.h"
#include "ritimer_17xx_40xx.h"
#include "rtc_17xx_40xx.h"
#include "spi_17xx_40xx.h"
#include "ssp_17xx_40xx.h"
#include "timer_17xx_40xx.h"
#include "uart_17xx_40xx.h"
#include "usb_17xx_40xx.h"
#include "wwdt_17xx_40xx.h"
#include "fmc_17xx_40xx.h"
#include "romapi_17xx_40xx.h"
*/
 /* FIXME : PWM drivers */

#ifdef __cplusplus
}
#endif

#endif /* __CHIP_LPC175X_6X_H_ */
