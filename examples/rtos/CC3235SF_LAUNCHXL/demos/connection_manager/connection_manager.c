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

   Application Name     - Connection manager application
   Application Overview - This application demonstrates how to use 
                          the provisioning method
                        in order to establish connection to the AP.
                        The application
                        connects to an AP and ping's the gateway to
                        verify the connection.

   Application Details  - Refer to 'Connection manager' README.html

 *****************************************************************************/
//****************************************************************************
//
//! \addtogroup
//! @{
//
//****************************************************************************

/* Standard Include */
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include "mqueue.h"
#include <pthread.h>
#include <time.h>
#include <assert.h>

/* TI-DRIVERS Header files */
#include <ti/drivers/net/wifi/simplelink.h>
#include <ti/drivers/net/wifi/slwificonn.h>

#include <ti/net/slnet.h>
#include <ti/net/slnetif.h>
#include <ti/net/slnetconn.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/GPIO.h>
#include <uart_term.h>
#include "wifi_if.h"

/* Application Version and Naming*/
#define APPLICATION_NAME                "CONNECTION MANAGER"
#define APPLICATION_VERSION             "02.00.00.00"


#define CONNECTION_SERVICE_LEVEL        SLNETCONN_SERVICE_LVL_INTERNET
#define INTERNET_SERVER_ADDRESS         0x08080808 // Google DNS Server
#define NUM_OF_TEST_ITERATIONS          5
#define NUM_OF_PING_ATTEMPTS            10
#define SLEEP_INTERVAL                  2
#define SLNETCONN_TIMEOUT               0xffff // "infinite" Timeout
/* USER's defines */
#define SLNETCONN_TASK_STACK_SIZE       (2048)
#define DISPLAY_TASK_STACK_SIZE         (512)
#define PING_TIMEOUT_SEC                (1)


/* Enable UART Log */
#define LOG_MESSAGE_ENABLE
#define LOG_MESSAGE UART_PRINT


/* LED State */
typedef enum
{
    LedState_CONNECTION,
    LedState_ERROR
}LedState;


/****************************************************************************
              LOCAL FUNCTION PROTOTYPES
 ****************************************************************************/

/****************************************************************************
              GLOBAL VARIABLES
 ****************************************************************************/
pthread_t gSlNetConnThread = (pthread_t)NULL;
pthread_t gDisplayThread = (pthread_t)NULL;

static bool gIsConnected = 0;

uint16_t gLedCount = 0;
uint8_t  gLedState = 0;
uint32_t gErrledCount = 0;
uint8_t  gErrLedState = 0;
LedState gLedDisplayState = LedState_CONNECTION;

extern pthread_t TaskCreate(int prio, size_t stacksize, void* (*fTask)(void*), void *arg);

//*****************************************************************************
//                 Local Functions
//*****************************************************************************
//*****************************************************************************
//
//! \brief The Function Handles the Fatal errors
//!
//! \param[in]  slFatalErrorEvent - Pointer to Fatal Error Event info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkFatalErrorEventHandler(SlDeviceFatal_t *slFatalErrorEvent)
{

    switch (slFatalErrorEvent->Id)
    {
    case SL_DEVICE_EVENT_FATAL_DEVICE_ABORT:
    {
        UART_PRINT("FATAL ERROR: Abort NWP event detected: "
                "AbortType=%d, AbortData=0x%x\n\r",
                slFatalErrorEvent->Data.DeviceAssert.Code,
                slFatalErrorEvent->Data.DeviceAssert.Value);
    }
    break;

    case SL_DEVICE_EVENT_FATAL_DRIVER_ABORT:
    {
        UART_PRINT("FATAL ERROR: Driver Abort detected\n\r");
    }
    break;

    case SL_DEVICE_EVENT_FATAL_NO_CMD_ACK:
    {
        UART_PRINT("FATAL ERROR: No Cmd Ack detected "
                "[cmd opcode = 0x%x]\n\r",
                slFatalErrorEvent->Data.NoCmdAck.Code);
    }
    break;

    case SL_DEVICE_EVENT_FATAL_SYNC_LOSS:
    {
        UART_PRINT("FATAL ERROR: Sync loss detected\n\r");
    }
    break;

    case SL_DEVICE_EVENT_FATAL_CMD_TIMEOUT:
    {
        UART_PRINT("FATAL ERROR: Async event timeout detected "
                "[event opcode = 0x%x]\n\r",
                slFatalErrorEvent->Data.CmdTimeout.Code);
    }
    break;

    default:
        UART_PRINT("FATAL ERROR: Unspecified error detected\n\r");
        break;
    }
}

void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
    /* Unused in this application */
}

void SimpleLinkHttpServerEventHandler(
        SlNetAppHttpServerEvent_t *pHttpEvent,
        SlNetAppHttpServerResponse_t *
        pHttpResponse)
{
    /* Unused in this application */
}

void SimpleLinkNetAppRequestEventHandler(SlNetAppRequest_t *pNetAppRequest,
                                         SlNetAppResponse_t *pNetAppResponse)
{
    /* Unused in this application */
}

void SimpleLinkNetAppRequestMemFreeEventHandler(uint8_t *buffer)
{
    /* Unused in this application */
}


//*****************************************************************************
//
//! \brief Error indication Led
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
void ErrorLedDisplay(void)
{
    GPIO_write(CONFIG_GPIO_LED_2, CONFIG_GPIO_LED_OFF);
    GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
    gErrLedState = 0;

    while(1)
    {
        gErrledCount++;
        gErrledCount &=  0xFFFFF;

        if (0 == gErrledCount)
        {
            if(gErrLedState)
            {
                GPIO_write(CONFIG_GPIO_LED_2, CONFIG_GPIO_LED_OFF);
                gErrLedState = 0;
            }
            else
            {
                GPIO_write(CONFIG_GPIO_LED_2, CONFIG_GPIO_LED_ON);
                gErrLedState = 1;
            }
        }
    }
}

//*****************************************************************************
//
//! \brief Update status for indication Led
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
void * UpdateLedDisplay(void *arg)
{
    while(1)
    {
        int freq = 500000; /* 500 msec */
        if(LedState_ERROR == gLedDisplayState)
        {
            ErrorLedDisplay();
        }
        else
        {
            if(gIsConnected) /* connected (led ON) */
            {
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
            }
            else /* disconnected (blink led) */
            {
                GPIO_toggle(CONFIG_GPIO_LED_0);
                if(gIsProvsioning)
                {
                    freq = 50000; /* 50 msec */
                }
            }
        }
        usleep(freq);
    }
}



//*****************************************************************************
//
//! \brief  Application side function
//!         Ping to Gateway and open timer for next ping
//!
//! \param  None
//!
//! \return None
//!
//*****************************************************************************
int32_t Ping(int numOfAttempts)
{
    int i;
    uint32_t gPingSent = 0;
    uint32_t gPingSuccess = 0;
    SlNetAppPingReport_t report;
    SlNetAppPingCommand_t pingCommand;

    /* Get IP and Gateway information */
    uint16_t len = sizeof(SlNetCfgIpV4Args_t);
    uint16_t ConfigOpt = 0;   /* return value could be one of the following: 
                         SL_NETCFG_ADDR_DHCP / SL_NETCFG_ADDR_DHCP_LLA
                                            / SL_NETCFG_ADDR_STATIC  */
    SlNetCfgIpV4Args_t ipV4 = {0};

    sl_NetCfgGet(SL_NETCFG_IPV4_STA_ADDR_MODE,&ConfigOpt,&len,(uint8_t *)&ipV4);

    LOG_MESSAGE(
            "\tDHCP is %s \r\n\tIP \t%d.%d.%d.%d \r\n\tMASK \t%d.%d.%d.%d \r\n\tGW"
            " \t%d.%d.%d.%d \r\n\tDNS \t%d.%d.%d.%d\n\r",
            (ConfigOpt == SL_NETCFG_ADDR_DHCP) ? "ON" : "OFF",
                    SL_IPV4_BYTE(ipV4.Ip, 3), SL_IPV4_BYTE(ipV4.Ip, 2),
                    SL_IPV4_BYTE(ipV4.Ip, 1), SL_IPV4_BYTE(ipV4.Ip, 0),
                    SL_IPV4_BYTE(ipV4.IpMask, 3), SL_IPV4_BYTE(ipV4.IpMask, 2),
                    SL_IPV4_BYTE(ipV4.IpMask, 1), SL_IPV4_BYTE(ipV4.IpMask, 0),
                    SL_IPV4_BYTE(ipV4.IpGateway, 3),SL_IPV4_BYTE(ipV4.IpGateway, 2),
                    SL_IPV4_BYTE(ipV4.IpGateway, 1),SL_IPV4_BYTE(ipV4.IpGateway, 0),
                    SL_IPV4_BYTE(ipV4.IpDnsServer, 3), SL_IPV4_BYTE(ipV4.IpDnsServer, 2),
                    SL_IPV4_BYTE(ipV4.IpDnsServer, 1), SL_IPV4_BYTE(ipV4.IpDnsServer, 0));

     /* destination IP of gateway                */
    if(CONNECTION_SERVICE_LEVEL == SLNETCONN_SERVICE_LVL_INTERNET)
    {
        /* Ineternet Level connection: Ping the Google DNS server*/
        pingCommand.Ip = INTERNET_SERVER_ADDRESS;
    }
    else
    {
        /* IP Level connection: Ping the Local Gateway */
         pingCommand.Ip = ipV4.IpGateway;
    }
    /* size of ping, in bytes                   */
    pingCommand.PingSize = 150;           

    /* delay between pings, in milliseconds     */
    pingCommand.PingIntervalTime = 100;    

    /* timeout for every ping in milliseconds   */
    pingCommand.PingRequestTimeout = 1000; 

    /* max number of ping requests. 0 - forever */
    pingCommand.TotalNumberOfAttempts = 1; 

    /* report only when finished                */
    pingCommand.Flags = 0;                          
    for (i=0; i< numOfAttempts; i++)
    {
        /* Ping Gateway */
        sl_NetAppPing( &pingCommand, SL_AF_INET, &report, NULL );

        /* Set Over all ping Statistics */
        gPingSent++;
        if (report.PacketsSent == report.PacketsReceived)
        {
            gPingSuccess++;
        }
        LOG_MESSAGE(
                "[%02d] Reply from %d.%d.%d.%d: %s, "
                "Time=%dms, \tOverall Stat Success (%d/%d)\r\n",
                i,
                SL_IPV4_BYTE(pingCommand.Ip,3),SL_IPV4_BYTE(pingCommand.Ip, 2),
                SL_IPV4_BYTE(pingCommand.Ip,1),SL_IPV4_BYTE(pingCommand.Ip, 0),
                (report.PacketsSent == report.PacketsReceived) ? "SUCCESS" : "FAIL",
                (report.PacketsSent == report.PacketsReceived) ? report.MinRoundTime : 0,
                gPingSuccess, gPingSent);

        sleep(PING_TIMEOUT_SEC);
    }
    return(0);
}

//*****************************************************************************
//
//! \brief  Application startup display on UART
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
static void DisplayBanner(char * AppName, char *AppVersion)
{

    LOG_MESSAGE("\n\n\n\r");
    LOG_MESSAGE("\t\t *************************************************\n\r");
    LOG_MESSAGE("\t\t            %s Application       \n\r", AppName);
    LOG_MESSAGE("\t\t            %s            \n\r", AppVersion);
    LOG_MESSAGE("\t\t *************************************************\n\r");
    LOG_MESSAGE("\n\n\n\r");
}



//*****************************************************************************
//
//! \brief  SlWifiConn Event Handler
//!
//*****************************************************************************
static void SlNetConnEventHandler(uint32_t ifID, SlNetConnStatus_e netStatus, void* data)
{
    switch(netStatus)
    {
    case SLNETCONN_STATUS_CONNECTED_MAC:
        gIsConnected = 1;
        LOG_MESSAGE("[SlNetConnEventHandler] I/F %d - CONNECTED (MAC LEVEL)!\n\r", ifID);
    break;
    case SLNETCONN_STATUS_CONNECTED_IP:
        gIsConnected = 1;
        LOG_MESSAGE("[SlNetConnEventHandler] I/F %d - CONNECTED (IP LEVEL)!\n\r", ifID);
    break;
    case SLNETCONN_STATUS_CONNECTED_INTERNET:
        gIsConnected = 1;
        LOG_MESSAGE("[SlNetConnEventHandler] I/F %d - CONNECTED (INTERNET LEVEL)!\n\r", ifID);
    break;
    case SLNETCONN_STATUS_WAITING_FOR_CONNECTION:
    case SLNETCONN_STATUS_DISCONNECTED:
        gIsConnected = 0;
        LOG_MESSAGE("[SlNetConnEventHandler] I/F %d - DISCONNECTED!\n\r", ifID);
    break;
    default:
        LOG_MESSAGE("[SlNetConnEventHandler] I/F %d - UNKNOWN STATUS\n\r", ifID);
    break;
    }
}


//*****************************************************************************
//
//! \brief Launchpad switch used to enable one shot provisioning
//!
//*****************************************************************************
void pushButtonEnableProvisioning(uint_least8_t index)
{
    int retVal = SlWifiConn_enableProvisioning(WifiProvMode_ONE_SHOT, SL_WLAN_PROVISIONING_CMD_START_MODE_APSC, 0);
    assert(retVal == 0);
}


//*****************************************************************************
//
//! \brief  Main application thread
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
void * mainThread( void *arg )
{
    int retVal;
    int i = 0;
    GPIO_init();
    SPI_init();

    /* Initial Terminal, and print Application name */
    InitTerm();
    DisplayBanner(APPLICATION_NAME, APPLICATION_VERSION);

    /* Switch off all LEDs on boards */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
    GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
    GPIO_write(CONFIG_GPIO_LED_2, CONFIG_GPIO_LED_OFF);

    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, pushButtonEnableProvisioning);
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    /* Enable SlWifiConn */
    retVal = WIFI_IF_init();
    assert (retVal == 0);

    /* Enable SlNet framework */
    retVal = ti_net_SlNet_initConfig();
    assert (retVal == 0);

    /* Enable SlNetConn */
    retVal = SlNetConn_init(0);
    assert (retVal == 0);
    gSlNetConnThread = TaskCreate(1, SLNETCONN_TASK_STACK_SIZE, SlNetConn_process, NULL);
    assert(gSlNetConnThread);

    /* create led display thread */
    gDisplayThread = TaskCreate(5, DISPLAY_TASK_STACK_SIZE, UpdateLedDisplay, NULL);

    LOG_MESSAGE("[APP] Init (%d) \r\n", retVal);

    for(i = 0; (i < NUM_OF_TEST_ITERATIONS) && (retVal == 0); i++)
    {
        retVal = SlNetConn_start(CONNECTION_SERVICE_LEVEL, SlNetConnEventHandler, SLNETCONN_TIMEOUT, 0);
        if(retVal == 0)
        {
            LOG_MESSAGE("[APP] Networking App Starts (%d) \r\n ", retVal);

            Ping(NUM_OF_PING_ATTEMPTS);

            LOG_MESSAGE("[APP] Networking App Completed (entering low power mode)\r\n ");
            retVal = SlNetConn_stop(SlNetConnEventHandler);
        }
        else
        {
            LOG_MESSAGE("[APP] Error! SlNetConn_start (%d)\n\r", retVal);
        }
        sleep(SLEEP_INTERVAL);
     }
    retVal = WIFI_IF_deinit();
    LOG_MESSAGE("[APP] Exit (%d) \r\n", retVal);
    assert (retVal == 0);

    return(0);
}
