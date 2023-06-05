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

//*****************************************************************************
// Includes
//*****************************************************************************
// Standard includes

#ifndef TCPIP_IF_H
#define TCPIP_IF_H

/****************************************************************************
              TYPE DEFINITIONS
 ****************************************************************************/
 //#define TCPIP_IF_ZERO_COPY 
 
/* interface states */
typedef enum 
{
    E_TCPIP_IF_DOWN,
    E_TCPIP_IF_UP,
} TcpipInterfaceState_e;

/* link states */
typedef enum 
{
    E_TCPIP_LINK_DOWN,
    E_TCPIP_LINK_UP,
} TcpipLinkState_e;

/* tcpip_status enumeration */
typedef enum 
{
    E_TCPIP_STATUS_IP_ACQUIRED,
    E_TCPIP_STATUS_IP_LOST,
    E_TCPIP_STATUS_PROV_STARTED,
    E_TCPIP_STATUS_PROV_STOPPED,
    E_TCPIP_STATUS_MAX
} TcpipStatue_e;

 
typedef void (*EvtCallback_f)(void *hNetif, TcpipStatue_e tcpip_status, void *pParams);
typedef int (*SendCallback_f)(void *hNetif, uint8_t *pPkt, int16_t payloadLen, uint32_t flags);

/****************************************************************************
             PUBLIC API
 ****************************************************************************/
typedef uint32_t ip4addr_t;


/*!

    \brief     Network-Stack init (should be called once before any other API)
    \param[in] fUserCallback - user callback for state change inidcation
    \param[in] bStackInit - flag indicating whether to initialize LWIP stack 
    \return    0 upon success or negative error code 
*/
int TCPIP_IF_init(EvtCallback_f fEvtCallback, unsigned char bStackInit);

/*!

    \brief     User request to register The WIFI Interface 
    		at the network stack 
    \return    pointer (handle) to the network interface or NULL in case of failure 
*/

#define TCPIP_IF_FLAGS_DHCPC 	0x00000001 /* DHCP CLIENT */
void * TCPIP_IF_addInterface(char *pName, uint8_t *pMacAddr, SendCallback_f fSendCallback, unsigned long flags);

void TCPIP_IF_deleteInterface(void *hNetIf);

/*!

    \brief     Enable/Disable the interface
    \param[in] hNetIf - network interface handle
    \param[in] state - up or down  

    \return    0 upon success or negative error code 
*/
int TCPIP_IF_setInterfaceState(void *hNetIf, TcpipInterfaceState_e state);

/*!

    \brief     Wi-Fi notification on change of Libk state
    \param[in] hNetIf - network interface handle
    \param[in] state - up or down

    \return    0 upon success or negative error code 
*/
int TCPIP_IF_notifyLinkChange(void *hNetIf, TcpipLinkState_e state);

int TCPIP_IF_setIp4Addr(void *hNetIf, ip4addr_t ip, ip4addr_t mask, ip4addr_t gw);

int   TCPIP_IF_receive(void *hPkt, int pktLen, void *hNetIf);

#ifdef TCPIP_IF_ZERO_COPY
void *TCPIP_IF_pktAlloc(uint16_t pktSize);
void  TCPIP_IF_pktFree(void *hPkt);
void *TCPIP_IF_pktPayload(void *hPkt);
int   TCPIP_IF_pktLength(void *hPkt);
#endif

#endif // TCPIP_IF_H
