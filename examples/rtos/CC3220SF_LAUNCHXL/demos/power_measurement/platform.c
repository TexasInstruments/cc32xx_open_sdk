/*
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//*****************************************************************************
// Includes
//*****************************************************************************
#ifdef __MSP432P401R__
#include <unistd.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerMSP432.h>
#elif CC32XX
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC32XX.h>
#include <ti/devices/cc32xx/driverlib/prcm.h>
#endif

#include "platform.h"

//*****************************************************************************
// Defines
//*****************************************************************************
#ifdef __MSP432P401R__
#define HIB_RETENTION_REGISTER          (0)
#elif CC32XX
#define HIB_RETENTION_REGISTER          (0x4402FC20)
#endif

//*****************************************************************************
// Local Functions
//*****************************************************************************

//*****************************************************************************
//
//! \brief Pre hibernate callback
//!
//! \param[in]  eventType - Powerxxxx_ENTERING_SHUTDOWN
//!             (second parameter in Power_registerNotify)
//!             eventArg - Shutdown State (ignored in CC32xx)
//!             clientArg - additional parameter
//!             (fourth parameter in Power_registerNotify)
//!
//! \return
//!
//*****************************************************************************
int preHibConfig(unsigned int eventType,
                 uintptr_t eventArg,
                 uintptr_t clientArg)
{
    return(Power_SOK);
}

//*****************************************************************************
//
//! \brief power Shutdown
//!
//! \param[in]
//!
//! \return
//!
//*****************************************************************************
void powerShutdown(uint32_t shutdownTime)
{
#ifdef CC32XX
    Power_NotifyObj hibSignal;
#endif

#ifdef __MSP432P401R__
#if 0
    Power_registerNotify(&hibSignal, PowerMSP432_ENTERING_SHUTDOWN,
                         preHibConfig,
                         (uintptr_t)NULL);
    if(shutdownTime != MAX_INT)
    {
        /* config gpio to wakeup from hib */
    }
    /*
     *    PowerMSP432_SHUTDOWN_0 => PCM_LPM35_VCORE0
     *    PowerMSP432_SHUTDOWN_1 => PCM_LPM45
     */
    Power_shutdown(PowerMSP432_SHUTDOWN_0,0);
#else
    if(shutdownTime != MAX_INT)
    {
        if(shutdownTime >= 1000)
        {
            sleep(shutdownTime / 1000);
        }
        usleep((shutdownTime % 1000) * 1000);
    }
#endif
#elif CC32XX
    Power_registerNotify(&hibSignal, PowerCC32XX_ENTERING_SHUTDOWN,
                         preHibConfig,
                         (uintptr_t)NULL);
    Power_shutdown(0,shutdownTime);
#endif
}

//*****************************************************************************
//
//! \brief check if we woken up from Hib or not
//!
//! \param[in]
//!
//! \return true if we woken up from hibernate else false
//!
//*****************************************************************************
bool isWokenFromHib(void)
{
#ifdef __MSP432P401R__
    /* read from retention the cause for wakeup */
    return(false);
#elif CC32XX
    if(MAP_PRCMSysResetCauseGet() == PRCM_HIB_EXIT)
    {
        return(true);
    }
    else
    {
        return(false);
    }
#endif
}

//*****************************************************************************
//
//! \brief read from retention register
//!
//! \param[in]
//!
//! \return value of retention register
//!
//*****************************************************************************
uint32_t getHibRetentionReg(void)
{
#ifdef __MSP432P401R__
/* read from retention register */
    return(0);
#elif CC32XX
    return (HWREGB(HIB_RETENTION_REGISTER));
#endif
}

//*****************************************************************************
//
//! \brief write to retention register
//!
//! \param[in]
//!
//! \return
//!
//*****************************************************************************
void setHibRetentionReg(uint32_t val)
{
#ifdef __MSP432P401R__
/* write to retention register */
#elif CC32XX
    HWREGB(HIB_RETENTION_REGISTER) = val;
#endif
}
