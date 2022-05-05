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
// includes
//*****************************************************************************
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>

/* TI-DRIVERS Header files */
#include <ti/drivers/net/wifi/simplelink.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/Power.h>
#include <ti/net/atcmd/atcmd.h>
#include "uart_term.h"
#include "pthread.h"
#include <ti/net/atcmd/atcmd_defs.h>
/* SlNetSock includes */
#include <ti/drivers/net/wifi/slnetifwifi.h>

//*****************************************************************************
// defines
//*****************************************************************************
#define APPLICATION_NAME        "AT Commands"
#define APPLICATION_VERSION     "1.1.2"

#define ATCOMMANDS_TASK_STACK_SIZE   (4096)
#define ATCOMMANDS_STOP_TIMEOUT      (200)
#define ATCOMMANDS_CMD_BUFFER_SIZE   (1024)
#define ATCOMMANDS_EVENT_BUFFER_SIZE (1024)
#define SLNET_IF_WIFI_PRIO           (5)

//****************************************************************************
// globals
//****************************************************************************
pthread_t ATCommands_eventThread = (pthread_t)NULL;
char ATCommands_cmdBuffer[ATCOMMANDS_CMD_BUFFER_SIZE];
char ATCommands_eventBuffer[ATCOMMANDS_EVENT_BUFFER_SIZE];



//****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//****************************************************************************

//****************************************************************************
//                         EXTERNAL FUNTIONS
//****************************************************************************
extern int32_t ti_net_SlNet_initConfig();

//*****************************************************************************
//
//! \brief Display Application Banner
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
int32_t ATCommands_displayBanner(void)
{
    int32_t status = -1;
    uint8_t macAddress[SL_MAC_ADDR_LEN];
    uint16_t macAddressLen = SL_MAC_ADDR_LEN;
    uint16_t configSize = 0;
    uint8_t configOpt = SL_DEVICE_GENERAL_VERSION;
    SlDeviceVersion_t ver = {0};

    configSize = sizeof(SlDeviceVersion_t);

    /* Print device version info. */
    status =
        sl_DeviceGet (  SL_DEVICE_GENERAL,
                         &configOpt,
                         &configSize,
                         (uint8_t*)(&ver));
    if(status < 0)
    {
        return(-1);
    }

    /* Print device Mac address */
    status = sl_NetCfgGet(   SL_NETCFG_MAC_ADDRESS_GET,
                             0,
                             &macAddressLen,
                             &macAddress[0]);
    if(status < 0)
    {
        return(-1);
    }

    UART_PRINT("\n\n\n\r");
    UART_PRINT("\t ==============================================\n\r");
    UART_PRINT("\t    %s Example Ver: %s\n\r",
               APPLICATION_NAME,
               APPLICATION_VERSION           );
    UART_PRINT("\t ==============================================\n\r");
    UART_PRINT("\n\r");
    UART_PRINT("\t CHIP: 0x%x",ver.ChipId);
    UART_PRINT("\n\r");
    UART_PRINT("\t MAC:  %d.%d.%d.%d",
                ver.FwVersion[0],
                ver.FwVersion[1],
                ver.FwVersion[2],
                ver.FwVersion[3]            );
    UART_PRINT("\n\r");
    UART_PRINT("\t PHY:  %d.%d.%d.%d",
               ver.PhyVersion[0],
               ver.PhyVersion[1],
               ver.PhyVersion[2],
               ver.PhyVersion[3]            );
    UART_PRINT("\n\r");
    UART_PRINT("\t NWP:  %d.%d.%d.%d",
               ver.NwpVersion[0],
               ver.NwpVersion[1],
               ver.NwpVersion[2],
               ver.NwpVersion[3]            );
    UART_PRINT("\n\r");
    UART_PRINT("\t ROM:  %d",ver.RomVersion);
    UART_PRINT("\n\r");
    UART_PRINT("\t HOST: %s", SL_DRIVER_VERSION);
    UART_PRINT("\n\r");
    UART_PRINT("\t MAC address: %02x:%02x:%02x:%02x:%02x:%02x"
             , macAddress[0],
               macAddress[1],
               macAddress[2],
               macAddress[3],
               macAddress[4],
               macAddress[5]                );
    UART_PRINT("\n\r");
    UART_PRINT("\n\r");
    UART_PRINT("\t ==============================================\n\r");
    UART_PRINT("\n\r");
    UART_PRINT("\n\r");

    return(status);
}

//*****************************************************************************
//
//! ATCommands_readCmd
//!
//!  \return none
//!
//!  \brief Handler function to handle the AT commands input
//
//*****************************************************************************
int32_t ATCommands_readCmd(void)
{
    int32_t lRetVal;
    uint32_t i = 1;
    char *pbuf;
    uint16_t length, readLen, offset;
    int16_t residue;
    uint8_t format = ATCMD_DATA_FORMAT_BASE64;
    char tmpBuf[16], *pTmpBuf;

    UART_PRINT("Enter AT Command:\n\r");
    while(i)
    {
        pbuf = NULL;
        usleep(1000);
        /* Poll UART terminal to receive user command terminated by '/r' */
        lRetVal = GetRawCmd((char *)ATCommands_cmdBuffer, 
                                            ATCOMMANDS_CMD_BUFFER_SIZE);
        if (lRetVal <= 1)
        {
            UART_PRINT("\n\r");
            continue;
        }
        offset = lRetVal;

        /* send */
        /* sendto */
        if ((strstr(ATCommands_cmdBuffer, ATCmd_sockSendStr)) ||
            (strstr(ATCommands_cmdBuffer, ATCmd_sockSendToStr)))
        {
            pbuf = strchr(ATCommands_cmdBuffer,ATCMD_DELIM_ARG) + 1;
        }

        if (pbuf != NULL)
        {
            pTmpBuf = tmpBuf;
            strncpy(pTmpBuf,pbuf,sizeof(tmpBuf));
            /* format */
            StrMpl_getVal(&pTmpBuf, &format , ATCMD_DELIM_ARG,
                                    STRMPL_FLAG_PARAM_SIZE_8);
            if (format == ATCMD_DATA_FORMAT_BINARY)
            {
                /* data length */
                StrMpl_getVal(&pTmpBuf, &length, ATCMD_DELIM_ARG,
                                       STRMPL_FLAG_PARAM_SIZE_16);
                /* pTmpBuf now points to the beginning of the data portion */
                // the extra 1 is to account for the NULL character at the end
                readLen = lRetVal - 
                (pbuf - ATCommands_cmdBuffer) - (pTmpBuf - tmpBuf) - 1;
                residue = length - readLen;
                offset = lRetVal;
                // means that the requested length is smaller than the actual payload
                if (residue < 0)   
                {
                    UART_PRINT("\n\rERROR: length smaller than payload\n\r");
                    continue;
                }

                while (residue > 0)
                {
                    /* read the residue */
                    lRetVal = GetRawCmd((char *)&ATCommands_cmdBuffer[offset],
                                                       residue);
                    if (lRetVal < 1)
                    {
                        UART_PRINT("\n\r");
                        break;
                    }

                    residue -= lRetVal;
                    offset += lRetVal;
                    // means that the requested length is smaller than the actual payload
                    if (residue < 0)    
                    {
                        UART_PRINT("ERROR: length smaller than payload\n\r");
                        continue;
                    }
                }
                if (lRetVal < 1)   // meaning a non valid break from the while loop
                {
                    continue;
                }
            }
        }

        UART_PRINT("\n\r");

        /* remove last CR or LF character */
        ATCommands_cmdBuffer[offset - 1] = '\0';

        ATCmd_send(ATCommands_cmdBuffer);
    }
    return(0);
}

//*****************************************************************************
//
//! \brief determine event type and send it to the UART
//!
//! \param  event buffer
//!
//! \return
//!
//*****************************************************************************
int32_t ATCommands_sendDataToUart(char *buffer)
{
    uint32_t i;
    char *pbuf = NULL;
    uint8_t format = ATCMD_DATA_FORMAT_BASE64;
    uint16_t length;
    char tmpBuf[8], *pTmpBuf;

    /* recv */
    /* recv from */
    if((strstr(buffer, ATCmd_sockRecvStr)) ||
       (strstr(buffer, ATCmd_sockRecvFromStr)) ||
       (strstr(buffer, ATCmd_httpGetHeaderStr)))
    {
        pbuf = strchr(buffer,ATCMD_DELIM_ARG) + 1;
    }

    /* file read */
    else if(strstr(buffer, ATCmd_fileReadStr))
    {
        pbuf = strchr(buffer,ATCMD_DELIM_EVENT) + 1;
    }

    /* netapp recv */
    else if((strstr(buffer, ATCmd_netappRecvStr)) ||
       (strstr(buffer, ATCmd_httpReadResBodyStr)))
    {
        pbuf = buffer;
        for(i = 0; i < 2; i++)
        {
            pbuf = strchr(pbuf,ATCMD_DELIM_ARG) + 1;
            if(pbuf == NULL)
            {
                break;
            }
        }
    }

    /* netutil cmd */
    else if((strstr(buffer, ATCmd_netUtilCmdStr)) ||
       (strstr(buffer, ATCmd_netUtilGetStr)))
    {
        /* only netutil cmd sign_msg and netutil
        get public key return more then 1 parameter */
        /* therefore look for argument delimiter */
        pbuf = strchr(buffer,ATCMD_DELIM_ARG);
        if(pbuf != NULL)
        {
            pbuf = strchr(buffer,ATCMD_DELIM_EVENT) + 1;
        }
    }

    /* mqtt recv event */
    else if((strstr(buffer, ATCmd_eventMqttStr)) &&
       (strstr(buffer, ATCmd_mqttEventId[1].str)))
    {
        pbuf = buffer;
        for(i = 0; i < 5; i++)
        {
            pbuf = strchr(pbuf,ATCMD_DELIM_ARG) + 1;
            if(pbuf == NULL)
            {
                break;
            }
        }
    }

    if(pbuf != NULL)
    {
        pTmpBuf = tmpBuf;
        strncpy(pTmpBuf,pbuf,sizeof(tmpBuf));
        /* format */
        StrMpl_getVal(   &pTmpBuf,
                         &format, ATCMD_DELIM_ARG,
                         STRMPL_FLAG_PARAM_SIZE_8    );
        if(format == ATCMD_DATA_FORMAT_BINARY)
        {
            /* data length */
            StrMpl_getVal(  &pTmpBuf, &length,
                          ATCMD_DELIM_ARG,
                          STRMPL_FLAG_PARAM_SIZE_16);
            length += ((pbuf - buffer) + (pTmpBuf - tmpBuf));

            /* send binary data to the uart */
            for(i = 0; i < length; i++)
            {
                putch(buffer[i]);
            }
            UART_PRINT("\n\r");
            return(0);
        }
    }

    UART_PRINT("%s\n\r",buffer);
    return(0);
}

//*****************************************************************************
//
//! ATCommands_eventTask
//!
//!  \param  pvParameters
//!
//!  \return none
//!
//!  \brief   AT event Task handler function to receive inputs
//
//*****************************************************************************

void * ATCommands_eventTask(void *pvParameters)
{
    int status;

    while(1)
    {
        status = ATCmd_recv(ATCommands_eventBuffer,0);
        if(status >= 0)
        {
            ATCommands_sendDataToUart(ATCommands_eventBuffer);
        }
    }
}

//*****************************************************************************
//
//! mainThread
//!
//!  \param  pvParameters
//!
//!  \return none
//!
//!  \brief Task handler
//
//*****************************************************************************

void * mainThread(void *pvParameters)
{
    int32_t status = 0;
    pthread_attr_t pAttrs;
    struct sched_param priParam;

    /* Initialize SlNetSock layer with CC31xx/CC32xx interface */
    status = ti_net_SlNet_initConfig();
    if(0 != status)
    {
        UART_PRINT("Failed to initialize SlNetSock\n\r");
    }

    GPIO_init();
    SPI_init();

    /* Configure the UART */
    InitTerm();

    /* Create AT Command module */
    ATCmd_create();

    sl_Start(0, 0, 0);
    ATCommands_displayBanner();

    pthread_attr_init(&pAttrs);
    priParam.sched_priority = 5;
    status = pthread_attr_setschedparam(&pAttrs, &priParam);
    status |= pthread_attr_setstacksize(&pAttrs, ATCOMMANDS_TASK_STACK_SIZE);

    status =
        pthread_create( &ATCommands_eventThread,
                        &pAttrs,
                        ATCommands_eventTask,
                        NULL                        );
    if(status != 0)
    {
        UART_PRINT("could not create task\n\r");
        /* error handling */
        while(1)
        {
            ;
        }
    }
    ATCommands_readCmd();
    
    return(0);
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
