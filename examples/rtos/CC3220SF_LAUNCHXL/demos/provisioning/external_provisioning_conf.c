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

/*****************************************************************************

 -------------  External Provisioning Configuration ---------------

In case of using external provisioning configuration, implement the functions bellow:

*****************************************************************************/
//****************************************************************************
//
//! \addtogroup
//! @{
//
//****************************************************************************
#include <stdint.h>
#include <ti/drivers/net/wifi/simplelink.h>

/* Implement a function which starts the external provisioning process
 * for example, open a server socket, wait for connection and receive the profile credentials.
 * Please note! once the external provisioning starts (i.e client connected successfully) internal
 * provisioning should be stopped */
void StartExternalProvisioning()
{
  /* External provisioning start implementation */

  /* Stop internal provisioning  (just in case it is running) and stay in current role after stop */
    sl_WlanProvisioning(SL_WLAN_PROVISIONING_CMD_STOP, 0xFF, 0, NULL,
                        (uint32_t)NULL);
}

/* Implement a function which stops the external provisioning process
 * for example, in case of AP provisioning or AP SC success, external provisioning is no longer needed */
void StopExternalProvisioning()
{
  /* External provisioning stop implementation */
}

/* Implement a function which check the current status of the external provisioning run */
uint8_t IsActiveExternalConfiguration()
{
    /* return TRUE in case external configuration is currently running, FALSE otherwise */
    return(FALSE);
}


