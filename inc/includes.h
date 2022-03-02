#ifndef _INCLUDES_H
#define _INCLUDES_H

#include "config.h"
#include "string.h"
#ifdef UNIT_TEST_EN
void UnitTestProcess(void);
#endif

#if(MCU_LIB_SELECT ==1)

#elif(MCU_LIB_SELECT ==2)

#define USE_STDPERIPH_DRIVER

#include "n32l40x.h"
#include "haltimer.h"
#include "common.h"
#include "mcuEEPROM.h"
#include "iap.h"
#include "haluart0.h"
#include "mcuAdapter.h"
#include "halN32Rtc.h"
#include "halN32Can.h"
#include "SEGGER_RTT.h"
#include "createtask.h"
//os
#include "cmsis_os.h"





#endif

#endif