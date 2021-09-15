/* ColdFire C Header File
 * Copyright Freescale Semiconductor Inc
 * All rights reserved.
 *
 * 2007/03/19 Revision: 0.91
 */

#ifndef __MCF52235_H__
#define __MCF52235_H__

#ifdef __cplusplus
extern "C" {
#endif

#pragma define_section system ".system" far_absolute RW

/***
 * MCF52235 Derivative Memory map definitions from linker command files:
 * __IPSBAR, __RAMBAR, __RAMBAR_SIZE, __FLASHBAR, __FLASHBAR_SIZE linker
 * symbols must be defined in the linker command file.
 */

extern __declspec(system)  uint8 __IPSBAR[];
extern __declspec(system)  uint8 __RAMBAR[];
extern __declspec(system)  uint8 __RAMBAR_SIZE[];
extern __declspec(system)  uint8 __FLASHBAR[];
extern __declspec(system)  uint8 __FLASHBAR_SIZE[];

#define IPSBAR_ADDRESS   (uint32)__IPSBAR
#define RAMBAR_ADDRESS   (uint32)__RAMBAR
#define RAMBAR_SIZE      (uint32)__RAMBAR_SIZE
#define FLASHBAR_ADDRESS (uint32)__FLASHBAR
#define FLASHBAR_SIZE    (uint32)__FLASHBAR_SIZE

#include "MCF52235_SCM.h"
#include "MCF52235_DMA.h"
#include "MCF52235_UART.h"
#include "MCF52235_I2C.h"
#include "MCF52235_QSPI.h"
#include "MCF52235_RTC.h"
#include "MCF52235_DTIM.h"
#include "MCF52235_INTC.h"
#include "MCF52235_GIACR.h"
#include "MCF52235_FEC.h"
#include "MCF52235_GPIO.h"
#include "MCF52235_PAD.h"
#include "MCF52235_RCM.h"
#include "MCF52235_CCM.h"
#include "MCF52235_CIM.h"
#include "MCF52235_PMM.h"
#include "MCF52235_CLOCK.h"
#include "MCF52235_EPORT.h"
#include "MCF52235_PIT.h"
#include "MCF52235_ADC.h"
#include "MCF52235_GPTA.h"
#include "MCF52235_PWM.h"
#include "MCF52235_FlexCAN.h"
#include "MCF52235_CFM.h"
#include "MCF52235_EPHY.h"
#include "MCF52235_RNGA.h"
#include "mii.h"

#ifdef __cplusplus
}
#endif

/********************************************************************/
/*
 * function prototypes
 */

void 
PIT_Timer_Init(uint8 PCSR, uint16 PMR);

void 
cpu_pause(int usecs);

#endif /* __MCF52235_H__ */
