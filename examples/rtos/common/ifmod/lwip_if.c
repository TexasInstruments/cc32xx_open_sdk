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
#include <assert.h>
#include <pthread.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

/* TI-DRIVERS Header files */
#include <ti/drivers/net/wifi/simplelink.h>
#include <ti/drivers/net/wifi/slwificonn.h>

#include "wifi_if.h"
#include <ti/net/slnet.h>
#include <ti/net/slnetconn.h>
#include <ti/net/slnetif.h>

#include "lwip/autoip.h"
#include "lwip/dhcp.h"
#include "lwip/err.h"
#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "lwip/tcpip.h"

#include "AppConfig.h"

#include "debug_if.h"
#undef DEBUG_IF_NAME
#undef DEBUG_IF_SEVERITY
#define DEBUG_IF_NAME "LWIP_IF"
#define DEBUG_IF_SEVERITY E_INFO

#include "utils_if.h"

#define ETH_MAX_PAYLOAD 1514
#define VLAN_TAG_SIZE (4U)
#define ETHHDR_SIZE 14
#define ETH_FRAME_SIZE (ETH_MAX_PAYLOAD + VLAN_TAG_SIZE)

#define ARP 0x806
#define IP 0x800
#define ICMP 1
#define IGMP 2
#define TCP 16
#define UDP 17

/****************************************************************************
              LOCAL FUNCTION PROTOTYPES
 ****************************************************************************/
static void * RecievePacketTask(void * hParam);
/****************************************************************************
              GLOBAL VARIABLES
 ****************************************************************************/
/* dhcp struct for the ethernet netif */
static struct dhcp netif_dhcp;
/* autoip struct for the ethernet netif */
static struct autoip netif_autoip;
//*****************************************************************************
//                 Local Functions
//*****************************************************************************

#define WIFI_BUFF_SIZE 1544
static _u8 txBuff[WIFI_BUFF_SIZE];
static _i16 RawEthernetSocket;

#define CHIP_DEVICE_CONFIG_SL_WIFI_MTU 1280
static struct netif mNetIf;

#if (DEBUG_IF_SEVERITY == E_TRACE)
#define NTOS(ptr) (((uint16_t)(ptr)[0])*256+(ptr)[1])

/* frequency (# packet received/sent) of heap log, 0 to disable logs */
#define HEAP_LOG_THRESHOLD 0  

void log_frame(char *str, uint32_t nBytes, uint8_t * p)
{
    uint16_t prot;
    uint8_t *ethhdr = p;

    LOG_TRACE("%s (%d)\t\t", str, nBytes);

    LOG_TRACE(" ETH: %02x:%02x:%02x:%02x:%02x:%02x -> %02x:%02x:%02x:%02x:%02x:%02x", 
              ethhdr[6], ethhdr[7], ethhdr[8], ethhdr[9], ethhdr[10], ethhdr[11],
              ethhdr[0], ethhdr[1], ethhdr[2], ethhdr[3], ethhdr[4], ethhdr[5]);

    prot = NTOS(&ethhdr[12]);
    switch (prot)
    {
    case ARP: {
        uint8_t *arphdr = &ethhdr[14];
        uint16_t op = NTOS(&arphdr[6]);
        LOG_TRACE(" ARP: OP=%d sender=%d.%d.%d.%d target=%d.%d.%d.%d", op, 
                  arphdr[14], arphdr[15], arphdr[16], arphdr[17], 
                  arphdr[24], arphdr[25], arphdr[26], arphdr[27]);
        break;
    }
    case IP: {
    	uint8_t *iphdr = &ethhdr[14];
        // 14 bytes from ethernet header + 9 bytes from IP header
        uint8_t ip_prot = iphdr[9];
        if (ip_prot == UDP)
        {
            uint8_t *udphdr = &iphdr[(iphdr[0]&0xf)*4];  
	    uint16_t src_port, dst_port;         	
            src_port = NTOS(&udphdr[0]);
            dst_port = NTOS(&udphdr[2]);
            LOG_TRACE(" IP: UDP: %d -> %d ", src_port, dst_port);
            break;
        }
        else if (ip_prot == ICMP)
        {
            LOG_TRACE(" IP: ICMP");
        }
        else if (ip_prot == IGMP)
        {
            LOG_TRACE(" IP: ICMP");
        }
        else if (ip_prot == TCP)
        {
            LOG_TRACE(" IP: TCP");
        }
        else if (ip_prot == TCP)
        {
            LOG_TRACE(" IP: %d", ip_prot);
        }
        break;
    }
    default: {
        LOG_TRACE(" prot: %d", prot);
    }
    }

#if HEAP_LOG_THRESHOLD 
    {   
        HeapStats_t heapStats;
        static size_t heapMin = 0xfffffff,  heapCntr = 0;
        vPortGetHeapStats( &heapStats );
    
        if(heapStats.xMinimumEverFreeBytesRemaining < heapMin)
        {
    	    heapCntr = HEAP_LOG_THRESHOLD; /* Force log */
    	    heapMin = heapStats.xMinimumEverFreeBytesRemaining;
        }
        if(heapCntr++ >= HEAP_LOG_THRESHOLD)		
        {
    	    LOG_TRACE("*** HEAP: avail=%d, min=%d ***", heapStats.xAvailableHeapSpaceInBytes, heapMin);
    	    heapCntr = 0;
        }
    }
#endif
}
#else
#define log_frame(p)
#endif

void status_callback(struct netif *pNetif)
{
  if (netif_is_up(pNetif))
  {
    const ip4_addr_t *ipv4;
    uint16_t macAddressLen = 6;
    uint16_t ConfigOpt = 0;
    sl_NetCfgGet(SL_NETCFG_MAC_ADDRESS_GET,&ConfigOpt,&macAddressLen,(_u8 *)pNetif->hwaddr);

    pNetif->hwaddr_len = 6;
    ipv4 = netif_ip4_addr(pNetif);
    LOG_INFO("STATUS:: I/F UP, IP: %s", ip4addr_ntoa(ipv4));
  }
  else
  {
    LOG_INFO("STATUS:: I/F DOWN\r\n");
  }
}


err_t ReceivePacket()
{
    err_t lwipErr      = ERR_OK;
    struct pbuf * pbuf = NULL;
    // Allocate an LwIP pbuf to hold the inbound packet.
    pbuf = pbuf_alloc(PBUF_LINK, WIFI_BUFF_SIZE, PBUF_POOL);
    if (pbuf)
    {
        int rc = sl_Recv(RawEthernetSocket, pbuf->payload, WIFI_BUFF_SIZE, 0);
        if (0 > rc)
        {
            LOG_ERROR("receive error (%d) \r\n", rc);
        }
        else
        {
            log_frame("RCVD", rc, pbuf->payload);
            lwipErr = mNetIf.input(pbuf, &mNetIf);
        }
        if (lwipErr != ERR_OK)
        {
            // If an error occurred, make sure the pbuf gets freed.
            if (pbuf != NULL)
            {
                pbuf_free(pbuf);
            }
            LOG_ERROR("mNetIf.input = %d", lwipErr);    
        }
    }
    return lwipErr;
}

err_t SendPacket(struct netif * netif, struct pbuf * pktPBuf)
{
    int rc = 0;
    struct pbuf * partialPkt;
    int offset = 0;

    for (partialPkt = pktPBuf; partialPkt != NULL; partialPkt = partialPkt->next)
    {
        if (offset + partialPkt->len > WIFI_BUFF_SIZE)
        {
            while (1)
                ;
        }
        memcpy(txBuff + offset, partialPkt->payload, partialPkt->len);
        offset += partialPkt->len;
    }

    rc = sl_Send(RawEthernetSocket, txBuff, offset, 0);
    if (0 > rc)
    {
        LOG_ERROR("Sl_Send = %d", rc);
        return ERR_BUF;
    }
    else
    {       
        log_frame("SENT", offset, txBuff);
        if(rc < offset)
        {
            LOG_WARNING("Sl_Send = %d", rc);
        }
    }
    return (err_t) ERR_OK;
}

void link_callback(struct netif * state_netif)
{
    err_t err;
    if (netif_is_link_up(state_netif))
    {
        err = dhcp_start(state_netif);
        LOG_INFO("link_callback==UP starting DHCP (%d)", err);
    }
    else
    {
        dhcp_stop(state_netif);
        LOG_INFO("LINK DOWN:: DHCP stopped");
    }
}

err_t lwipNetIfInit(struct netif * netif)
{
    netif_set_status_callback(netif, status_callback);
    netif_set_link_callback(netif, link_callback);
    autoip_set_struct(netif, &netif_autoip);
    dhcp_set_struct(netif, &netif_dhcp);

    netif->output     = etharp_output;
    netif->linkoutput = SendPacket;
    netif->flags |= (NETIF_FLAG_ETHARP | NETIF_FLAG_IGMP);

    netif->mtu = ETH_FRAME_SIZE - ETHHDR_SIZE - VLAN_TAG_SIZE;
    return ERR_OK;
}

int enable_netif(void)
{
    uint16_t hwaddrlen;

    ip4_addr_t ipaddr  = { 0 };
    ip4_addr_t netmask = { 0 };
    ip4_addr_t gw      = { 0 };

    mNetIf.name[0] = 's';
    mNetIf.name[1] = 'l';

    netif_add(&mNetIf, &ipaddr, &netmask, &gw, NULL, lwipNetIfInit, tcpip_input);

    _u16 ConfigOpt = 0;
    sl_NetCfgGet(SL_NETCFG_MAC_ADDRESS_GET, &ConfigOpt, &hwaddrlen, (_u8 *) mNetIf.hwaddr);

    cc32xxLog("%2x:%2x:%2x:%2x:%2x:%2x", mNetIf.hwaddr[0], mNetIf.hwaddr[1], mNetIf.hwaddr[2], mNetIf.hwaddr[3], mNetIf.hwaddr[4],
              mNetIf.hwaddr[5]);

    mNetIf.hwaddr_len = (uint8_t) hwaddrlen;
    return 0;
}

void LWIP_IF_enableInterface()
{
    /* set IpV6 Address(es) */
    int rc;

    rc = enable_netif();
    while (rc != 0)
        ;
}

void tcpinternal_network_set_up(void * newif)
{
    struct netif * nif = newif;
    netif_set_up(nif);
    nif->mtu = ETH_FRAME_SIZE - ETHHDR_SIZE - VLAN_TAG_SIZE;
    netif_set_link_up(nif);
}

void network_set_up(void * newif)
{
    tcpip_callback(tcpinternal_network_set_up, newif);
}

int NetworkBypassInit()
{
    int rc;

    SlNetCfgIpV4Args_t ipAddr = { 0 };
    memset(&ipAddr, 0, sizeof(SlNetCfgIpV4Args_t));
    rc = sl_NetCfgSet(SL_NETCFG_IPV4_STA_ADDR_MODE, SL_NETCFG_ADDR_STATIC, sizeof(SlNetCfgIpV4Args_t), (_u8 *) &ipAddr);

    if (rc >= 0)
    {
        // disable network applications
        rc = sl_NetAppStop(SL_NETAPP_HTTP_SERVER_ID | SL_NETAPP_DHCP_SERVER_ID | SL_NETAPP_MDNS_ID);
        rc = sl_NetAppStart(SL_NETAPP_MDNS_ID);
    }
    if (rc >= 0)
    {
        // restart NWP by calling stop then start to init with static IP 0.0.0.0
        rc = WIFI_IF_reset(1000);
    }
    LOG_INFO("Configure N/W Bypass mode (%d)", rc);
    return rc;
}



int LWIP_IF_start()
{
    /* set IpV6 Address(es) */
    int rc;
    
    rc = NetworkBypassInit();
    rc = enable_netif();
    network_set_up(&mNetIf);
    if (rc < 0)
    {
        return rc;
    }

    {
        SlWlanRxFilterIdMask_t FilterIdMask;
        _u16 len = sizeof(SlWlanRxFilterIdMask_t);
        memset(FilterIdMask, 0, sizeof(FilterIdMask));
        rc = sl_WlanSet(SL_WLAN_RX_FILTERS_ID, SL_WLAN_RX_FILTER_STATE, len, (_u8 *) FilterIdMask);
        if (rc < 0)
        {
            return rc;
        }
    }

    /* Create raw socket */
    RawEthernetSocket = sl_Socket(SL_AF_PACKET, SL_SOCK_RAW, 0);
    OS_createTask(1, 0x400, RecievePacketTask, NULL, PTHREAD_CREATE_DETACHED); // thread is terminated upon exit
    return rc;
}

void * RecievePacketTask(void * hParam)
{
    /* Start listening */
    while (1)
    {
        ReceivePacket();
    }
    return NULL;
}
