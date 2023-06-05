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
//
//! \addtogroup file_operations
//! @{
//
//*****************************************************************************

// Standard includes



#include <stdint.h>
#include <stddef.h>
#include <unistd.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <mqueue.h>

#include "net_log.h"
#include "debug_if.h"

#undef DEBUG_IF_NAME
#define DEBUG_IF_NAME       "NET"
#undef DEBUG_IF_SEVERITY
#define DEBUG_IF_SEVERITY   E_INFO

#ifndef bool
typedef enum { false, true } bool;
#endif

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
      
/*!

    \brief     Ethernet Packet Trace (Log)

    \param[in] type - LOG_RX or LOG_TX 
    \param[in] pIfName - string name of the interface
    \param[in] nBytes - length of packet (headers + payload)
    \param[in] p - pointer to start of packet (ethernet header)

    \return    module id (>=0) upon success, or negative error code

*/
void ETHERNET_logFrame(logType_e type, char *pIfName, uint32_t nBytes, uint8_t * p)
{
    uint16_t prot;
    uint8_t *ethhdr = p;
    char *pTypeStr = (type==LOG_RX)?"RCVD":"SENT";
    uint32_t offset = 0;
    char buff[256];

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
             strIPOffset += sprintf(&strIP[strIPOffset], " | IP6: "); 
             strIPOffset = IPv6_LOG(strIP, strIPOffset, iphdr, 8);
             strIPOffset += sprintf(&strIP[strIPOffset], " -> "); 
             strIPOffset = IPv6_LOG(strIP, strIPOffset, iphdr, 24);
             ip_prot = iphdr[6];
             ip_prot_offset = 40;
             
             if(ip_prot == 0)
             {
                 ip_prot = iphdr[40];
                 ip_prot_offset = 48+iphdr[41];
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


