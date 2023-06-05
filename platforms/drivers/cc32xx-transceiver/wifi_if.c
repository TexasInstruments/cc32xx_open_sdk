/*
 * Copyright (C) 2016-2021, Texas Instruments Incorporated
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

   Application Name     - Provisioning application
   Application Overview - This application demonstrates how to use 
                          the provisioning method
                        in order to establish connection to the AP.
                        The application
                        connects to an AP and ping's the gateway to
                        verify the connection.

   Application Details  - Refer to 'Provisioning' README.html

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
#include <ti/drivers/net/wifi/slnetifwifi.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/apps/LED.h>
#include "utils_if.h"
#include "debug_if.h"
#include "wifi_if.h"
#include "tcpip_if.h"

#include <ti/net/slnet.h>
#include <ti/net/slnetconn.h>
#include <ti/net/slnetif.h>

#undef DEBUG_IF_NAME
#undef DEBUG_IF_SEVERITY
#define DEBUG_IF_NAME       "WIFI"
#define DEBUG_IF_SEVERITY   WIFI_IF_DEBUG_LEVEL

#define WIFI_BUFF_SIZE 1544

#define SLNETCONN_TASK_STACK_SIZE 	(2048)
#define SLNETCONN_TIMEOUT              9 // 10 Second Timeout

#if (DEBUG_IF_SEVERITY == E_TRACE)
typedef enum { LOG_RX, LOG_TX } logType_e;
static void __LogFrame(logType_e type, char *pIfName, uint32_t nBytes, uint8_t * p);
#else
#define __LogFrame(p)
#endif


typedef struct _connRequest
{
    WifiEventHandler_f    handler;
    WifiConnStatus_e      reqStatus;
    struct _connRequest  *pNext;
} connRequest_t;

typedef struct 
{
    pthread_t            spawnThread;
    pthread_t            slWifiConnThread;
    pthread_t            slExtProvThread;
    pthread_t            slNetConnThread;

    bool                 isInitiated; 
    bool                 isProvsioning; 
    bool                 isConnected; 
    bool                 isExtProvsioning;

    /* Simplelink Raw Ethernet Socket */
    _i16 	          slRawSocket;

    void                 *hStaNetif;
    /* This bit-wise status variable shows the state of the NWP */
    uint32_t             Status;                    

    /* This field keeps the device's role (STA, P2P or AP) */
    uint32_t             Role;                     

    connRequest_t       *pConnRequests;

    uint32_t             ConnectedStations;
    WifiServiceLevel_e   maxReqConnLevel;
    WifiConnStatus_e     currConnStatus;
    /* STA/AP mode CB */
} WifiCtx_t;

/****************************************************************************
              GLOBAL VARIABLES
 ****************************************************************************/
static WifiCtx_t m_ctx = {0};
static ExtProv_start_f gfExtProvStart = NULL;
static ExtProv_stop_f  gfExtProvStop = NULL;
static void *gpExtProvHandle = NULL;
#ifdef WIFI_LED_HANDLE
extern LED_Handle WIFI_LED_HANDLE;
#endif

/****************************************************************************
              STATIC FUNCTIONS
 ****************************************************************************/
static void *ReceivePacket_Task(void * hParam);
static int NetworkBypass(bool bEnable);
static void Notify(WifiConnStatus_e wifiConnStatus);

/*****************************************************************************
          SimpleLink Callback Functions :WLAN, NETAPP and GENERAL EVENTS
          are served here. They are used internally by SlWifiConn (as lib
          registration) and are available here only for debug prints.
 *****************************************************************************/

//*****************************************************************************
//
//! \brief The Function Handles WLAN Events (used only for log messages)
//!
//! \param[in]  pWlanEvent - Pointer to WLAN Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent)
{
    static const char *Roles[] = {"STA","STA","AP","P2P"};
    static const char *WlanStatus[] = {"DISCONNECTED","SCANING","CONNECTING","CONNECTED"};

    switch(pWlanEvent->Id)
    {
    case SL_WLAN_EVENT_CONNECT:
        /* set the string terminate */
        pWlanEvent->Data.Connect.SsidName[pWlanEvent->Data.Connect.SsidLen] =
                '\0';
        LOG_INFO(" [Event] STA connected to AP "
                "- BSSID:%.2x:%.2x:%.2x:%.2x:%.2x:%.2x, SSID:%s",
                pWlanEvent->Data.Connect.Bssid[0],
                pWlanEvent->Data.Connect.Bssid[1],
                pWlanEvent->Data.Connect.Bssid[2],
                pWlanEvent->Data.Connect.Bssid[3],
                pWlanEvent->Data.Connect.Bssid[4],
                pWlanEvent->Data.Connect.Bssid[5],
                pWlanEvent->Data.Connect.SsidName);

        break;

    case SL_WLAN_EVENT_DISCONNECT:
        LOG_INFO(" [Event] STA disconnected from AP (Reason Code = %d)",
                 pWlanEvent->Data.Disconnect.ReasonCode);
        break;

    case SL_WLAN_EVENT_STA_ADDED:
        LOG_DEBUG(" [Event] New STA Addeed (MAC Address:  %.2x:%.2x:%.2x:%.2x:%.2x)",
                  pWlanEvent->Data.STAAdded.Mac[0],
                  pWlanEvent->Data.STAAdded.Mac[1],
                  pWlanEvent->Data.STAAdded.Mac[2],
                  pWlanEvent->Data.STAAdded.Mac[3],
                  pWlanEvent->Data.STAAdded.Mac[4],
                  pWlanEvent->Data.STAAdded.Mac[5]);
        break;

    case SL_WLAN_EVENT_STA_REMOVED:
        LOG_DEBUG(" [Event] STA Removed (MAC Address: %.2x:%.2x:%.2x:%.2x:%.2x)",
                  pWlanEvent->Data.STAAdded.Mac[0],
                  pWlanEvent->Data.STAAdded.Mac[1],
                  pWlanEvent->Data.STAAdded.Mac[2],
                  pWlanEvent->Data.STAAdded.Mac[3],
                  pWlanEvent->Data.STAAdded.Mac[4],
                  pWlanEvent->Data.STAAdded.Mac[5]);
        break;

    case SL_WLAN_EVENT_PROVISIONING_PROFILE_ADDED:
        LOG_DEBUG(" [Provisioning] Profile Added: SSID: %s",
                  pWlanEvent->Data.ProvisioningProfileAdded.Ssid);
        if(pWlanEvent->Data.ProvisioningProfileAdded.ReservedLen > 0)
        {
            LOG_DEBUG(" [Provisioning] Profile Added: PrivateToken:%s",
                      pWlanEvent->Data.ProvisioningProfileAdded.Reserved);
        }
        break;

    case SL_WLAN_EVENT_PROVISIONING_STATUS:
    {
        switch(pWlanEvent->Data.ProvisioningStatus.ProvisioningStatus)
        {
        case SL_WLAN_PROVISIONING_GENERAL_ERROR:
        case SL_WLAN_PROVISIONING_ERROR_ABORT:
        case SL_WLAN_PROVISIONING_ERROR_ABORT_INVALID_PARAM:
        case SL_WLAN_PROVISIONING_ERROR_ABORT_HTTP_SERVER_DISABLED:
        case SL_WLAN_PROVISIONING_ERROR_ABORT_PROFILE_LIST_FULL:
        case SL_WLAN_PROVISIONING_ERROR_ABORT_PROVISIONING_ALREADY_STARTED:
            LOG_DEBUG(" [Provisioning] Provisioning Error status=%d",
                      pWlanEvent->Data.ProvisioningStatus.ProvisioningStatus);
            break;

        case SL_WLAN_PROVISIONING_CONFIRMATION_STATUS_FAIL_NETWORK_NOT_FOUND:
            LOG_DEBUG(" [Provisioning] Profile confirmation failed: "
                    "network not found");
            break;

        case SL_WLAN_PROVISIONING_CONFIRMATION_STATUS_FAIL_CONNECTION_FAILED:
            LOG_DEBUG(" [Provisioning] Profile confirmation failed:"
                    " Connection failed");
            break;

        case
        SL_WLAN_PROVISIONING_CONFIRMATION_STATUS_CONNECTION_SUCCESS_IP_NOT_ACQUIRED:
            LOG_DEBUG(" [Provisioning] Profile confirmation failed:"
                    " IP address not acquired");
            break;

        case SL_WLAN_PROVISIONING_CONFIRMATION_STATUS_SUCCESS_FEEDBACK_FAILED:
            LOG_DEBUG(" [Provisioning] Profile Confirmation failed "
                    " (Connection Success, feedback to Smartphone app failed)");
            break;

        case SL_WLAN_PROVISIONING_CONFIRMATION_STATUS_SUCCESS:
            LOG_DEBUG(" [Provisioning] Profile Confirmation Success!");
            break;

        case SL_WLAN_PROVISIONING_AUTO_STARTED:
            LOG_DEBUG(" [Provisioning] Auto-Provisioning Started");
            break;

        case SL_WLAN_PROVISIONING_STOPPED:
            LOG_DEBUG(" Provisioning stopped: Current Role: %s",
                      Roles[pWlanEvent->Data.ProvisioningStatus.Role]);
            if(ROLE_STA == pWlanEvent->Data.ProvisioningStatus.Role)
            {
                LOG_DEBUG("WLAN Status: %s",
                          WlanStatus[pWlanEvent->Data.ProvisioningStatus.
                                     WlanStatus]);

                if(SL_WLAN_STATUS_CONNECTED ==
                        pWlanEvent->Data.ProvisioningStatus.WlanStatus)
                {
                    LOG_DEBUG("Connected to SSID: %s",
                              pWlanEvent->Data.ProvisioningStatus.Ssid);
                }
            }
            break;

        case SL_WLAN_PROVISIONING_SMART_CONFIG_SYNCED:
            LOG_DEBUG(" [Provisioning] Smart Config Synced!");
            break;

        case SL_WLAN_PROVISIONING_SMART_CONFIG_SYNC_TIMEOUT:
            LOG_DEBUG(" [Provisioning] Smart Config Sync Timeout!");
            break;

        case SL_WLAN_PROVISIONING_CONFIRMATION_WLAN_CONNECT:
            LOG_DEBUG(
                    " [Provisioning] Profile confirmation: WLAN Connected!");
            break;

        case SL_WLAN_PROVISIONING_CONFIRMATION_IP_ACQUIRED:
            LOG_DEBUG(
                    " [Provisioning] Profile confirmation: IP Acquired!");
            break;

        case SL_WLAN_PROVISIONING_EXTERNAL_CONFIGURATION_READY:
            LOG_DEBUG(" [Provisioning] External configuration is ready! ");
            /* [External configuration]: External configuration is ready,
start the external configuration process.
        In case of using the external provisioning
enable the function below which will trigger StartExternalProvisioning() */
            break;

        default:
            LOG_ERROR(" [Provisioning] Unknown Provisioning Status: %d",
                      pWlanEvent->Data.ProvisioningStatus.ProvisioningStatus);
            break;
        }
    }
    break;

    default:
        LOG_ERROR(" [Event] - WlanEventHandler has received %d !!!!",
                  pWlanEvent->Id);
        break;
    }
}

//*****************************************************************************
//
//! \brief This function handles network events such as IP acquisition, IP
//!           leased, IP released etc (used only for log messages)
//!
//! \param[in]  pNetAppEvent - Pointer to NetApp Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    LOG_ERROR("[NETAPP EVENT] Unhandled event [0x%x] ", pNetAppEvent->Id);
}

//*****************************************************************************
//
//! \brief This function handles General Events
//!
//! \param[in]  pDevEvent - Pointer to General Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
    /* Most of the general errors are not FATAL are are to be handled
appropriately by the application */
    LOG_DEBUG("[GENERAL EVENT] - ID=[%d] Sender=[%d]\n\n",
              pDevEvent->Data.Error.Code,
              pDevEvent->Data.Error.Source);
}

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
        LOG_ERROR("FATAL ERROR: Abort NWP event detected: "
                "AbortType=%d, AbortData=0x%x\n\r",
                slFatalErrorEvent->Data.DeviceAssert.Code,
                slFatalErrorEvent->Data.DeviceAssert.Value);
    }
    break;

    case SL_DEVICE_EVENT_FATAL_DRIVER_ABORT:
    {
        LOG_ERROR("FATAL ERROR: Driver Abort detected\n\r");
    }
    break;

    case SL_DEVICE_EVENT_FATAL_NO_CMD_ACK:
    {
        LOG_ERROR("FATAL ERROR: No Cmd Ack detected "
                "[cmd opcode = 0x%x]\n\r",
                slFatalErrorEvent->Data.NoCmdAck.Code);
    }
    break;

    case SL_DEVICE_EVENT_FATAL_SYNC_LOSS:
    {
        LOG_ERROR("FATAL ERROR: Sync loss detected\n\r");
    }
    break;

    case SL_DEVICE_EVENT_FATAL_CMD_TIMEOUT:
    {
        LOG_ERROR("FATAL ERROR: Async event timeout detected "
                "[event opcode = 0x%x]\n\r",
                slFatalErrorEvent->Data.CmdTimeout.Code);
    }
    break;

    default:
        LOG_ERROR("FATAL ERROR: Unspecified error detected\n\r");
        break;
    }
}


__attribute__((weak)) void SimpleLinkSockEventHandler(SlSockEvent_t * pSock)
{
    /* Unused in this application */
}

__attribute__((weak)) void SimpleLinkHttpServerEventHandler(SlNetAppHttpServerEvent_t * pHttpEvent, SlNetAppHttpServerResponse_t * pHttpResponse)
{
    /* Unused in this application */
}

__attribute__((weak)) void SimpleLinkNetAppRequestEventHandler(SlNetAppRequest_t * pNetAppRequest, SlNetAppResponse_t * pNetAppResponse)
{
    /* Unused in this application */
}

__attribute__((weak)) void SimpleLinkNetAppRequestMemFreeEventHandler(uint8_t * buffer)
{
    /* Unused in this application */
}

//*****************************************************************************
//
//! \brief  SlWifiConn Event Handler
//!
//*****************************************************************************
void SlNetConnEventHandler(uint32_t ifID, SlNetConnStatus_e netStatus, void * data)
{
    switch (netStatus)
    {
    case SLNETCONN_STATUS_CONNECTED_MAC:
        m_ctx.isConnected = 1;
        NetworkBypass(true);
        TCPIP_IF_notifyLinkChange(m_ctx.hStaNetif, E_TCPIP_LINK_UP);
        Notify(WIFI_STATUS_CONNECTED);
        LED_setOn(gLedBlueHandle, LED_BRIGHTNESS_MAX);
        cc32xxLog("[SlNetConnEventHandler] I/F %d - CONNECTED (MAC LEVEL)!\n\r", ifID);
        break;
    case SLNETCONN_STATUS_CONNECTED_IP:
        m_ctx.isConnected = 1;
        cc32xxLog("[SlNetConnEventHandler] I/F %d - CONNECTED (IP LEVEL)!\n\r", ifID);
        NetworkBypass(false);
        TCPIP_IF_notifyLinkChange(m_ctx.hStaNetif, E_TCPIP_LINK_DOWN);
        Notify(WIFI_STATUS_DISCONNECTED);
        break;
    case SLNETCONN_STATUS_CONNECTED_INTERNET:
        m_ctx.isConnected = 1;
        cc32xxLog("[SlNetConnEventHandler] I/F %d - CONNECTED (INTERNET LEVEL)!\n\r", ifID);
        break;
    case SLNETCONN_STATUS_WAITING_FOR_CONNECTION:
    case SLNETCONN_STATUS_DISCONNECTED:
        m_ctx.isConnected = 0;
        LED_setOff(gLedBlueHandle);
        cc32xxLog("[SlNetConnEventHandler] I/F %d - DISCONNECTED!\n\r", ifID);
        break;
    default:
        cc32xxLog("[SlNetConnEventHandler] I/F %d - UNKNOWN STATUS\n\r", ifID);
        break;
    }
}


static unsigned char rxbuff[WIFI_BUFF_SIZE];
static void *ReceivePacket_Task(void * hNetIf)
{
    int rc = 0;

    /* Start listening */
    LOG_INFO("%s:: starting Rx Task",  __func__);


    while (1) 
    {
        while (m_ctx.isConnected) 
        {
#ifdef TCPIP_IF_ZERO_COPY
            void *pbuf = NULL;
            // Allocate an LwIP pbuf to hold the inbound packet.
            pbuf = TCPIP_IF_pktAlloc(WIFI_BUFF_SIZE);
            if (pbuf)
            {
                void *pPayload = TCPIP_IF_pktPayload(pbuf); 
                rc = sl_Recv(m_ctx.slRawSocket, pPayload, WIFI_BUFF_SIZE, 0);
                if (0 >= rc)
                {
                    LOG_ERROR("SL ReceiveLogFrame Error (%d)", rc);
                }
                else
                {
                    __LogFrame(LOG_RX, "WIFI", rc, pPayload);
                    rc = TCPIP_IF_receive(pbuf, rc, hNetIf);
                    if (rc != 0)
                    {
                        // If an error occurred, make sure the pbuf gets freed.
                        LOG_ERROR("LWIP Receive Error (%d)", rc);
                    }
                }
                //TCPIP_IF_pktFree(pbuf);
            }
#else

            rc = sl_Recv(m_ctx.slRawSocket, rxbuff, WIFI_BUFF_SIZE, 0);
            if (0 >= rc)
            {
                LOG_ERROR("SL ReceiveLogFrame Error (%d)", rc);
            }
            else
            {
                __LogFrame(LOG_RX, "WIFI", rc, rxbuff);
                rc = TCPIP_IF_receive(rxbuff, rc, hNetIf);
                if (rc != 0)
                {
                    // If an error occurred, make sure the pbuf gets freed.
                    LOG_ERROR("LWIP Receive Error (%d)", rc);
                }
            }
#endif //  TCPIP_IF_ZERO_COPY
       }
        usleep(10);
    }
    pthread_exit(NULL);
    return NULL;
}

static void Notify(WifiConnStatus_e wifiConnStatus)
{
    connRequest_t *pConnReq = m_ctx.pConnRequests;
    while(pConnReq)
    {
    	if(pConnReq->handler)
    	   pConnReq->handler(wifiConnStatus);
    	pConnReq = pConnReq->pNext;
    }

} 

static void TcpipEventHandler(void *pNetif, TcpipStatue_e status, void * pParams)
{
    LOG_INFO("TcpipEventHandler(%d)\n\r", status);
    switch (status)
    {
    case E_TCPIP_STATUS_IP_ACQUIRED:
        Notify(WIFI_STATUS_CONNECTED_IP);
        break;
    case E_TCPIP_STATUS_IP_LOST:
        Notify(WIFI_STATUS_CONNECTED);
        break;
    default:
        break;
    }
}

//*****************************************************************************
//                 Local Functions
//*****************************************************************************


//*****************************************************************************
//
//! \brief  Configure static profile based on hard coded setting or
//!         a configuration file
//!
//*****************************************************************************
int ConfigureLocalNetwork()
{
    int retVal = 0;
    uint8_t *pBuf = NULL;
    char *pSSID = NULL;
    SlWlanSecParams_t  secParams = {SL_WLAN_SEC_TYPE_OPEN, NULL, 0};
    if (AP_SSID != NULL)
    {
        /* AP_SSID is not NULL - it will be set as a static profile
         * This should be used in development
         */
        pSSID = AP_SSID;
        if(AP_PASSWORD != NULL)
        {
            secParams.Type = SL_WLAN_SEC_TYPE_WPA_WPA2;
            secParams.Key = (int8_t*)AP_PASSWORD;
            secParams.KeyLen = strlen((char*)secParams.Key);
        }
    }
    else if(AP_CFG_FILENAME != NULL)
    {
        int len;
        /* Read Network credential from a file (if exists) */
        pBuf  = (uint8_t *)malloc(AP_CFG_MAX_SIZE);
        if(pBuf)
        {
            len = FILE_read((int8_t*)AP_CFG_FILENAME, AP_CFG_MAX_SIZE, pBuf, AP_CFG_TOKEN);
            if(len > 0)
            {
                int i;
                pSSID = (char*)pBuf;
                for(i=0; i<len; i++)
                {
                    if(pBuf[i] == ' ')
                    {
                        pBuf[i] = 0;
                        if(i < len-1)
                        {
                            secParams.Type = SL_WLAN_SEC_TYPE_WPA_WPA2;
                            secParams.Key = (int8_t*)&pBuf[i+1];
                            secParams.KeyLen = len-(i+1);
                            break;
                        }
                    }
                }
                pBuf[len] = 0;
            }
        }
    }
    if(pSSID)
    {
        uint16_t ssidLen = strlen(pSSID);
        retVal = SlWifiConn_addProfile(pSSID, ssidLen, NULL, &secParams, NULL, 15, SLWIFICONN_PROFILE_FLAG_NON_PERSISTENT);
    }
    if(pBuf)
        free(pBuf);

    return retVal;
}

//*****************************************************************************
//
//! \brief  Thread context for the SlWifiConn
//!
//! \note   The SlWifiConn_pocess only returns when the module is destoryed
//!         (see WIFI_IF_deinit)
//!
//*****************************************************************************
static void *SlWifiConnTask(void *pvParameters)
{
    void* retVal = SlWifiConn_process(pvParameters);
    pthread_exit(NULL);
    return retVal;
}


//*****************************************************************************
//
//! \brief  Thread context for the external provisioning method (e.g. WAC)
//!
//! \note   The start_f should return upon
//!
//*****************************************************************************
static void *WifiExtProvisioingTask(void* pvParameters)
{
    if (gfExtProvStart)
    {
        gfExtProvStart(gpExtProvHandle);
    }
    pthread_detach(pthread_self());
    pthread_exit(NULL);
    return NULL;
}


//*****************************************************************************
//
//! \brief  SlWifiConn Event Handler
//!
//*****************************************************************************
static void SlWifiConnEventHandler(WifiConnEventId_e eventId , WifiConnEventData_u *pData)
{
    uint8_t  simpleLinkMac[SL_MAC_ADDR_LEN];
    uint16_t macAddressLen;

    switch(eventId)
    {
    case WifiConnEvent_POWERED_UP:
    {
        LOG_INFO("[SlWifiConnEventHandler] POWERED_UP ");
        macAddressLen = sizeof(simpleLinkMac);
        sl_NetCfgGet(SL_NETCFG_MAC_ADDRESS_GET,NULL,&macAddressLen,
                     (unsigned char *)simpleLinkMac);
        LOG_INFO("  MAC address: %x:%x:%x:%x:%x:%x",
                 simpleLinkMac[0],
                 simpleLinkMac[1],
                 simpleLinkMac[2],
                 simpleLinkMac[3],
                 simpleLinkMac[4],
                 simpleLinkMac[5]                            );
    }
    break;
    case WifiConnEvent_POWERED_DOWN:
        LOG_INFO("[SlWifiConnEventHandler] POWERED DOWN "_CLR_RESET_);
        break;
    case WifiConnEvent_PROVISIONING_STARTED:
        m_ctx.isProvsioning = 1;
        LOG_INFO("[SlWifiConnEventHandler] PROVISIONING STARTED !\n\r"
                "      mode=%d (0-AP, 1-SC, 2-AP+SC, 3-AP+SC+EXT)" _CLR_RESET_, pData->provisioningCmd);
#ifdef WIFI_LED_HANDLE
        LED_startBlinking(WIFI_LED_HANDLE, 500, LED_BLINK_FOREVER);
#endif
        break;
    case WifiConnEvent_PROVISIONING_STOPPED:
        m_ctx.isProvsioning = 0;
        LOG_INFO("[SlWifiConnEventHandler] PROVISIONING_STOPPED !\n\r"
                "      status = %d (0-SUCCESS, 1-FAILED, 2-STOPPED)"_CLR_RESET_, pData->status);
#ifdef WIFI_LED_HANDLE
        LED_stopBlinking(WIFI_LED_HANDLE);
#endif
        break;
    case WifiConnEvent_EXTERNAL_PROVISIONING_START_REQ:
        m_ctx.isExtProvsioning = 1;
        if(gfExtProvStart)
        {
            m_ctx.slExtProvThread =  OS_createTask(EXT_PROV_TASK_PRIORITY, EXT_PROV_STACK_SIZE, WifiExtProvisioingTask, NULL, OS_TASK_FLAG_DETACHED);
            assert(m_ctx.slExtProvThread);
        }
        LOG_INFO("[SlWifiConnEventHandler] START EXT PROVISIONING !"_CLR_RESET_);
        break;
    case WifiConnEvent_EXTERNAL_PROVISIONING_STOP_REQ:
        m_ctx.isExtProvsioning = 0;
        if(gfExtProvStop)
        {
            gfExtProvStop(gpExtProvHandle);
        }
        LOG_INFO("[SlWifiConnEventHandler] STOP EXT PROVISIONING !"_CLR_RESET_);
        break;
    default:
        LOG_INFO("[SlWifiConnEventHandler] UNKNOWN EVENT "_CLR_RESET_);
    }
}

static int TcpipCB_send(void *pPayload, int16_t payloadLen, uint32_t flags)
{
    int rc = sl_Send(m_ctx.slRawSocket, pPayload, payloadLen, (int16_t)flags);
    if (rc > 0)
    {
        __LogFrame(LOG_TX, "WIFI", payloadLen, pPayload);
    }
    else
    {
        LOG_ERROR("sl_Send (len = %d) :: Error = %d", payloadLen, rc);
    }
    return 0;
} 

static int NetworkBypass(bool bEnableCmd)
{
    int rc;
    static bool bEnabled = 0;
    
    if(bEnableCmd && !bEnabled)
    {
	    /* The following will set IP Address to 0.0.0.0 - for bypass mode */
	    SlNetCfgIpV4Args_t ipAddr = { 0 };
	    memset(&ipAddr, 0, sizeof(SlNetCfgIpV4Args_t));
	    rc = sl_NetCfgSet(SL_NETCFG_IPV4_STA_ADDR_MODE, SL_NETCFG_ADDR_STATIC, sizeof(SlNetCfgIpV4Args_t), (_u8 *) &ipAddr);
	    if(rc == 0)
	    {
	        SlNetCfgIpV6Args_t ipAddr6 = { 0 };
	        memset(&ipAddr6, 0, sizeof(SlNetCfgIpV6Args_t));
  	        //rc = sl_NetCfgSet(SL_NETCFG_IPV6_ADDR_LOCAL, SL_NETCFG_ADDR_STATIC, sizeof(SlNetCfgIpV6Args_t), (_u8 *) &ipAddr6);
                if(rc == 0)
		{
    	             //rc = sl_NetCfgSet(SL_NETCFG_IPV6_ADDR_GLOBAL, SL_NETCFG_ADDR_STATIC, sizeof(SlNetCfgIpV6Args_t), (_u8 *) &ipAddr6);
	  	}
  	    }

        #if 1
        if (rc >= 0)
        {
              SlWlanRxFilterSysFiltersMask_t  FilterSysIdMask;
              _u16 len = sizeof(SlWlanRxFilterSysFiltersMask_t);
              _u16  config_opt = SL_WLAN_RX_FILTER_SYS_STATE;

              memset(FilterSysIdMask, 0, sizeof(FilterSysIdMask));
              rc = sl_WlanGet(SL_WLAN_RX_FILTERS_ID, &config_opt, &len, (_u8*)FilterSysIdMask); // FilterSysIdMask[0] should be 0xF8
              assert(rc == 0); 
              LOG_INFO("FilterSysIdMask(before): %d\n", *(unsigned long*)FilterSysIdMask);
              
    
              FilterSysIdMask[0] &= ~(1 << (7 - SL_WLAN_RX_FILTER_MULTICASTSIPV6_SYS_FILTERS));
              FilterSysIdMask[0] &= ~(1 << (7 - SL_WLAN_RX_FILTER_MULTICASTSIPV4_SYS_FILTERS));
              rc = sl_WlanSet(SL_WLAN_RX_FILTERS_ID, config_opt, len, (_u8*)FilterSysIdMask);
              assert(rc == 0);        
                  
              memset(FilterSysIdMask, 0, sizeof(FilterSysIdMask));
              rc = sl_WlanGet(SL_WLAN_RX_FILTERS_ID, &config_opt, &len, (_u8*)FilterSysIdMask);
              assert(rc == 0); 
              LOG_INFO("FilterSysIdMask(after): %d\n", *(unsigned long*)FilterSysIdMask);
              
       }
       #endif

    	#if 1
	    if (rc >= 0)
	    {
		// disable network applications
		rc = sl_NetAppStop(SL_NETAPP_HTTP_SERVER_ID | SL_NETAPP_DHCP_SERVER_ID | SL_NETAPP_MDNS_ID);
		rc = sl_NetAppStart(SL_NETAPP_MDNS_ID);
	    }
       #if 0	    
	    if (rc >= 0)
	    {
		// restart NWP by calling stop then start to init with static IP 0.0.0.0
		rc = WIFI_IF_reset(1000);
	    }
	#endif
	#endif
	    if(rc == 0)
	    {
		SlWlanRxFilterIdMask_t FilterIdMask;
		_u16 len = sizeof(SlWlanRxFilterIdMask_t);
		memset(FilterIdMask, 0, sizeof(FilterIdMask));
		rc = sl_WlanSet(SL_WLAN_RX_FILTERS_ID, SL_WLAN_RX_FILTER_STATE, len, (_u8 *) FilterIdMask);
		if(rc == 0)
		{
		    if(m_ctx.slRawSocket == -1)
		    {
		        m_ctx.slRawSocket = sl_Socket(SL_AF_PACKET, SL_SOCK_RAW, 0);
		        if(m_ctx.slRawSocket >= 0)
		        {
		            OS_createTask(7, 0x400, ReceivePacket_Task, m_ctx.hStaNetif, PTHREAD_CREATE_DETACHED); // thread is terminated upon exit
		            bEnabled = true;
                       }
#if 0
                       if(rc == 0)
  	               {
  	                    SlSockIpV6Mreq_t mreq = {0};
  	                    mreq.ipv6mr_multiaddr._S6_un._S6_u32[0] = 0xff020000;
  	                    mreq.ipv6mr_multiaddr._S6_un._S6_u32[1] = 0;
  	                    mreq.ipv6mr_multiaddr._S6_un._S6_u32[2] = 0;
  	                    mreq.ipv6mr_multiaddr._S6_un._S6_u32[3] = 0xfb;
                            rc = sl_SetSockOpt(m_ctx.slRawSocket, SL_IPPROTO_IP, SL_IPV6_ADD_MEMBERSHIP, &mreq, sizeof(mreq));
                            assert(rc == 0);
                       }
#endif
		    }
		}

	    }
    }
    else if(!bEnableCmd && bEnabled)
    {
       rc = sl_NetCfgSet(SL_NETCFG_IPV4_STA_ADDR_MODE,SL_NETCFG_ADDR_DHCP_LLA,0,0);
       bEnabled = false;
    }
    LOG_INFO("Configure N/W Bypass mode (%d)", rc);
    return rc;
}



//*****************************************************************************
//
//! \brief  WiFi (SimpleLink driver and SlWifiConn) initialization
//!
//*****************************************************************************
int WIFI_IF_init(unsigned char bNetworkStackInit)
{
    int rc;

    assert(m_ctx.isInitiated == false);

    /*** CC32XX SW Initialization ***/
    /* Enable SlNet framework (NWP is enabled (sl_Start) but RF is off)*/
    rc = ti_net_SlNet_initConfig();
    assert(rc == 0);        /* Simplelink Spawn Thread init (should be done before SlWifiConn i initiated */

    /* SL Driver init  */
    m_ctx.spawnThread =  OS_createTask(SL_SPAWN_TASK_PRIORITY, SL_SPAWN_STACK_SIZE, sl_Task, NULL, OS_TASK_FLAG_DETACHED);
    assert(m_ctx.spawnThread);

    /* SlWifiConn init + Thread creation  */
    rc = SlWifiConn_init(SlWifiConnEventHandler);
    assert(rc == 0);
    m_ctx.slWifiConnThread =  OS_createTask(WIFI_CONN_TASK_PRIORITY, WIFI_CONN_STACK_SIZE, SlWifiConnTask, NULL, OS_TASK_FLAG_DETACHED);
    assert(m_ctx.slWifiConnThread);

    /* SlNetConn init + Thread creation  */
    rc = SlNetConn_init(0);
    assert(rc == 0);
    m_ctx.slNetConnThread = OS_createTask(1, SLNETCONN_TASK_STACK_SIZE, SlNetConn_process, NULL, OS_TASK_FLAG_DETACHED);
    assert(m_ctx.slNetConnThread);

    /* Set Provisioning parameters (based on wifi_settings) */
    if (PROVISIONING_MODE != WifiProvMode_OFF)
    {
        uint32_t flags = 0;
#if FORCE_PROVISIONING
        flags = SLWIFICONN_PROV_FLAG_FORCE_PROVISIONING;
#endif
        rc = SlWifiConn_enableProvisioning(PROVISIONING_MODE, PROVISIONING_CMD, flags);
    }

    /* Adding logging capabilities to the SlWifiConn */
#if (SLWIFICONN_DEBUG_ENABLE)
    if(DEBUG_IF_SEVERITY <= E_DEBUG)
    {
        SlWifiConn_registerDebugCallback((SlWifiConn_Debug_f)cc32xxLog);
        assert(rc == 0);
    }
#endif

    /* Static Network Profile Settings */
    if(!FORCE_PROVISIONING)
    {
        ConfigureLocalNetwork();
    }

    rc = sl_NetAppStart(SL_NETAPP_MDNS_ID);
    assert(rc == 0 || rc == SL_ERROR_NET_APP_MDNS_ALREADY_STARTED);
    rc = 0;

    /* Update SlWifiConn Settings */
    WifiConnSettings_t wifiConnSettings;
    SlWifiConn_getConfiguration(&wifiConnSettings);
    if(PROVISIONING_TIMEOUT) 
    {
        wifiConnSettings.provisioningTimeout = PROVISIONING_TIMEOUT;
    }
    if(PROVISIONING_AP_PASSWORD != NULL)
    {
        wifiConnSettings.provisioningAP_secType = SL_WLAN_SEC_TYPE_WPA_WPA2;
        wifiConnSettings.provisioningAP_password = PROVISIONING_AP_PASSWORD;
    }
    if(PROVISIONING_SC_KEY != NULL)
    {
        wifiConnSettings.provisioningSC_key = PROVISIONING_SC_KEY;
    }
    wifiConnSettings.connectTimeout = 10; 
    wifiConnSettings.ipTimeout = 10;
    
    SlWifiConn_setConfiguration(&wifiConnSettings);

    assert(rc == 0);

    /*** Prepare TCPIP stack ***/
    rc = TCPIP_IF_init(TcpipEventHandler, bNetworkStackInit);
    if (rc == 0)
    {
        uint16_t macAddressLen = 6;
        uint16_t ConfigOpt = 0;
        uint8_t macaddr[6];
        sl_NetCfgGet(SL_NETCFG_MAC_ADDRESS_GET,&ConfigOpt, &macAddressLen,(_u8 *)macaddr);

        m_ctx.hStaNetif = TCPIP_IF_addInterface("st", macaddr, TcpipCB_send, TCPIP_IF_FLAGS_DHCPC);
        assert(m_ctx.hStaNetif);
        m_ctx.slRawSocket = -1;
        m_ctx.isInitiated = true;
    }
    return rc;
}




//*****************************************************************************
//
//! \brief  try to connect to an AP (based on provisioning, stored profiles
//!           or hard-coded setting). If timeout is provided, the function will
//!          block until connection is established or timeout occurs.         
//!
//*****************************************************************************
int WIFI_IF_start(WifiEventHandler_f handler, WifiServiceLevel_e level, unsigned long timeout_ms, void **hConnReq)
{
    int rc;
    connRequest_t *pConnReq;
    
    LOG_INFO("Start Wi-Fi");
    /* Try to connect to AP and go through provisioning (if needed) */
    TCPIP_IF_setInterfaceState(m_ctx.hStaNetif, E_TCPIP_IF_UP);
    rc = SlNetConn_start(SLNETCONN_SERVICE_LVL_MAC, SlNetConnEventHandler, SLNETCONN_TIMEOUT, 0); 
    if(rc >= 0)
    {
        pConnReq = (connRequest_t*)malloc(sizeof(connRequest_t));
        assert(pConnReq);
    	pConnReq->handler = handler;
    	pConnReq->reqStatus = (WifiConnStatus_e)(WIFI_STATUS_CONNECTED + level);
        pConnReq->pNext = m_ctx.pConnRequests;
        m_ctx.pConnRequests = pConnReq;
        
        if(hConnReq)
        {
            *hConnReq = (void*)pConnReq;
        }
    }
    return rc;
}

//*****************************************************************************
//
//! \brief  try to connect to an AP (based on provisioning, stored profiles
//!           or hard-coded setting). If timeout is provided, the function will
//!          block until connection is established or timeout occurs.         
//!
//*****************************************************************************
int WIFI_IF_stop(void *hConnReq)
{
    int rc = 0;

    LOG_INFO("Start Wi-Fi");
    TCPIP_IF_setInterfaceState(m_ctx.hStaNetif, E_TCPIP_IF_DOWN);
    rc = SlNetConn_stop((SlNetConn_AppEvent_f)(hConnReq));

    return rc;
}

//*****************************************************************************
//
//! \brief  Registration of External Provsioning Callbacks
//!
//*****************************************************************************
int WIFI_IF_registerExtProvCallbacks(ExtProv_start_f fStart, ExtProv_stop_f fStop, void *pHandle)
{
    gfExtProvStart = fStart;
    gfExtProvStop = fStop;
    gpExtProvHandle = pHandle;
    return 0;
}

//*****************************************************************************
//
//! \brief  A request for NWP reset
//!
//*****************************************************************************742
int WIFI_IF_reset()
{
    return SlWifiConn_reset();
}

//*****************************************************************************
//
//! \brief  Free SlWifiConn resources (including the module's thread)
//!
//*****************************************************************************
int WIFI_IF_deinit()
{
    int retVal = SlWifiConn_deinit();
    if(retVal == 0)
    {
        void *ret;
        retVal = pthread_join(m_ctx.slWifiConnThread, &ret);
    }
    return retVal;
}

#if (DEBUG_IF_SEVERITY == E_TRACE)
#define NTOS(ptr) (((uint16_t)(ptr)[0])*256+(ptr)[1])

/* frequency (# packet received/sent) of heap log, 0 to disable logs */
#define ARP 0x806
#define IP 0x800
#define IP6 0x86DD
#define ICMP    1
#define IGMP    2
#define TCP     6
#define UDP     17
#define ICMP6   58
void breakpoint()
{
    //__asm__("BKPT");
}

#define ETH_LOG    (1)
#define IP4_LOG    (1)
#define IP6_LOG    (1)
#define TCP_LOG    (1)
#define UDP_LOG    (1)
#define ARP_LOG    (1)
#define ICMP4_LOG  (1)
#define ICMP6_LOG  (1)
#define IGMP_LOG   (1)

static uint32_t IPv6_LOG(char *buff, uint32_t offset, uint8_t *iphdr, int ipOffset)
{
    int i;
    bool bFoundZero = false;
    for(i=0; i<8; i++)
    {    
        uint16_t value = iphdr[ipOffset+i*2]*256+iphdr[ipOffset+1+i*2];
        if(value)
        {
            offset += sprintf(&buff[offset], "%04x:", value); 
            bFoundZero = false;
        }
        else
        {
            if(!bFoundZero)
            {
                offset += sprintf(&buff[offset], ":");
            } 
            bFoundZero = true;
        }
    }
    return offset-1;
}
      
static void __LogFrame(logType_e type, char *pIfName, uint32_t nBytes, uint8_t * p)
{
    uint16_t prot;
    uint8_t *ethhdr = p;
    char *pTypeStr = (type==LOG_RX)?"RCVD":"SENT";
    bool bIsIP6 = false;
    uint32_t offset = 0;
    char buff[256];

//    offset = sprintf(&buff[0], "%s (%s) :: %lu bytes, ", pTypeStr, pIfName, nBytes);
    offset = sprintf(&buff[0], "%s (%4lu) | ", pTypeStr, nBytes);

#if ETH_LOG
    offset += sprintf(&buff[offset], "ETH: %02x:%02x:%02x:%02x:%02x:%02x -> %02x:%02x:%02x:%02x:%02x:%02x", 
              ethhdr[6], ethhdr[7], ethhdr[8], ethhdr[9], ethhdr[10], ethhdr[11],
              ethhdr[0], ethhdr[1], ethhdr[2], ethhdr[3], ethhdr[4], ethhdr[5]);
#endif

    prot = NTOS(&ethhdr[12]);
    switch (prot)
    {
    case ARP: {
        uint8_t *arphdr = &ethhdr[14];
        uint16_t op = NTOS(&arphdr[6]);
#if ARP_LOG
        offset += sprintf(&buff[offset], " | ARP: OP=%d sender=%d.%d.%d.%d target=%d.%d.%d.%d", op, 
                              arphdr[14], arphdr[15], arphdr[16], arphdr[17], 
                              arphdr[24], arphdr[25], arphdr[26], arphdr[27]);
#else 
        return;
#endif
        break;
    }
    case IP:
    case IP6: {
    	uint8_t *iphdr = &ethhdr[14];
    	uint8_t ip_prot = 0;
    	uint8_t ip_prot_offset = 0;
    	char strIP[72];
    	uint32_t strIPOffset = 0;
        // 14 bytes from ethernet header + 9 bytes from IP header
        if((prot==IP) && ((iphdr[0]&0xf0) == 0x40))
        {
#if IP4_LOG
             sprintf(strIP, " | IP4: %d.%d.%d.%d -> %d.%d.%d.%d", iphdr[12], iphdr[13], iphdr[14], iphdr[15], iphdr[16], iphdr[17], iphdr[18], iphdr[19]);
             ip_prot = iphdr[9];
             ip_prot_offset = (iphdr[0]&0xf)*4;
#else
             return;
#endif
        }
        else if((prot==IP6) && ((iphdr[0]&0xf0) == 0x60))
        {
#if IP6_LOG
             bIsIP6 = true;

             //offset += sprintf(&buff[offset], " IP6: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x -> %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x",
             strIPOffset += sprintf(&strIP[strIPOffset], " | IP6: "); 
             strIPOffset = IPv6_LOG(strIP, strIPOffset, iphdr, 8);
             strIPOffset += sprintf(&strIP[strIPOffset], " -> "); 
             strIPOffset = IPv6_LOG(strIP, strIPOffset, iphdr, 24);
             ip_prot = iphdr[6];
             ip_prot_offset = 40;
             
             if(ip_prot == 0)
             {
                 //strIPOffset += sprintf(&strIP[strIPOffset], "HOP-BY-HOP6 | ");
                 ip_prot = iphdr[40];
                 ip_prot_offset = 48+iphdr[41];
             }
            if(type == LOG_RX)
            {
                 breakpoint();
            }        
#else
             return;
#endif
        }
        offset += sprintf(&buff[offset], "%-72s", strIP);
        if (ip_prot == UDP)
        {
#if UDP_LOG
            uint8_t *udphdr = &iphdr[ip_prot_offset];  
	    uint16_t src_port, dst_port;         	
            src_port = NTOS(&udphdr[0]);
            dst_port = NTOS(&udphdr[2]);
            offset += sprintf(&buff[offset], " | UDP: %d -> %d ", src_port, dst_port);
            if(type == LOG_RX && bIsIP6 && dst_port == 5353)
            {
                 breakpoint();
            }
#else
             return;
#endif
            break;
        }
        else if (ip_prot == ICMP)
        {
#if ICMP4_LOG 
           LOG_TRACE(" ICMP:");
#else
             return;
#endif
        }
        else if (ip_prot == ICMP6)
        {
#if ICMP4_LOG 
            char strType[4] = {0};
            char *icmp6Type = strType;
            uint8_t *icmp6hdr = &iphdr[ip_prot_offset];
            sprintf(strType, "%d", icmp6hdr[0]);
            switch(icmp6hdr[0]) {
            case 128:  icmp6Type="Echo Request"; break;
            case 129:  icmp6Type="Echo Reply"; break;
            case 130:  icmp6Type="Multicast Listener Query"; break;
            case 131:  icmp6Type="Multicast Listener Report"; break;    
            case 133:  icmp6Type="Router Solicitation"; break;    
            case 134:  icmp6Type="Router Advertisement "; break;    
            case 135:  icmp6Type="Neighbor Solicitation"; break;    
            case 136:  icmp6Type="Neighbor Advertisement "; break;    
            case 137:  icmp6Type="Redirect"; break; 
            }   
            offset += sprintf(&buff[offset], " | ICMP6: %s", icmp6Type);
#else
             return;
#endif
        }
        else if (ip_prot == IGMP)
        {
#if IGMP4_LOG
             offset += sprintf(&buff[offset], " | IGMP:");
#else
             return;
#endif
        }
        else if (ip_prot == TCP)
        {
#if TCP_LOG
            offset += sprintf(&buff[offset], " | TCP:");
        }
        else 
        {
            offset += sprintf(&buff[offset], " | IP-PROT (%d):", ip_prot);
#else
            return;
#endif
        }
        break;
    }
    default: {
        offset += sprintf(&buff[offset], " | prot: %d", prot);
    }
    }
    LOG_TRACE("%s", buff);
}

#endif


