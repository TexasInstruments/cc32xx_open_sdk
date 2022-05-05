/*
 * Copyright (c) 2017-2018, Texas Instruments Incorporated
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

/*
 *    ======== tcpEchoTLS.c ========
 */

#include <string.h>
#include <stdint.h>

#include <time.h>
#include <pthread.h>

/* BSD support */
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#include <ti/net/slnetsock.h>
#include <ti/net/slnetif.h>

#include <ti/display/Display.h>

/* TI-DRIVERS Header files */
#include "ti_drivers_config.h"

/* Example/Board Header file */
#include "certs/certificate.h"

/* Secure object names */
#define ROOT_CA_CERT_FILE     "DummyCA"
#define PRIVATE_KEY_FILE      "DummyKey"
#define TRUSTED_CERT_FILE     "DummyTrustedCert"

#define TCPPACKETSIZE 256
#define NUMTCPWORKERS 2

extern Display_Handle display;

extern void startSNTP(void);
extern void *TaskCreate(void (*pFun)(), char *Name, int Priority,
        uint32_t StackSize, uintptr_t Arg1, uintptr_t Arg2, uintptr_t Arg3);

/*
 *  ======== tcpWorker ========
 *  Task to handle TCP connection. Can be multiple Tasks running
 *  this function.
 */
void tcpWorker(uint32_t arg0, uint32_t arg1)
{
    int  clientFd = (int)arg0;
    int  bytesRcvd;
    int  bytesSent;
    char buffer[TCPPACKETSIZE];

    Display_printf(display, 0, 0, "tcpWorker: start clientFd = 0x%x\n",
            clientFd);

    while ((bytesRcvd = recv(clientFd, buffer, TCPPACKETSIZE, 0)) > 0) {
        bytesSent = send(clientFd, buffer, bytesRcvd, 0);
        if (bytesSent < 0 || bytesSent != bytesRcvd) {
            Display_printf(display, 0, 0, "send failed.\n");
            break;
        }
    }
    Display_printf(display, 0, 0, "tcpWorker stop clientFd = 0x%x\n", clientFd);

    close(clientFd);
}

/*
 *  ======== tcpHandler ========
 *  Creates new Task to handle new TCP connections.
 */
void tcpHandler(uint32_t arg0, uint32_t arg1)
{
    void *             thread = NULL;
    int                status = 0;
    int                clientFd;
    int                serverFd;
    uint16_t           clientSd;
    uint16_t           serverSd;
    socklen_t          sdlen = sizeof(serverSd);
    struct sockaddr_in localAddr;
    struct sockaddr_in clientAddr;
    int                optval;
    int                optlen = sizeof(optval);
    socklen_t          addrlen = sizeof(clientAddr);
    SlNetSockSecAttrib_t *secAttribHdl = NULL;

    Display_printf(display, 0, 0, "TCP Echo TLS example started\n");

    /*  Use SNTP to get the current time, as needed for SSL authentication */
    startSNTP();

    serverFd = socket(AF_INET, SOCK_STREAM, 0);
    if (serverFd == -1) {
        Display_printf(display, 0, 0, "tcpHandler: socket failed\n");
        goto shutdown;
    }

    status = SlNetIf_loadSecObj(SLNETIF_SEC_OBJ_TYPE_CERTIFICATE,
            ROOT_CA_CERT_FILE, strlen(ROOT_CA_CERT_FILE), srvCAPem,
            srvCAPemLen, SLNETIF_ID_1);
    status |= SlNetIf_loadSecObj(SLNETIF_SEC_OBJ_TYPE_CERTIFICATE,
            TRUSTED_CERT_FILE, strlen(TRUSTED_CERT_FILE), srvCertPem,
            srvCertPemLen, SLNETIF_ID_1);
    status |= SlNetIf_loadSecObj(SLNETIF_SEC_OBJ_TYPE_RSA_PRIVATE_KEY,
            PRIVATE_KEY_FILE, strlen(PRIVATE_KEY_FILE)-1, srvKeyPem,
            srvKeyPemLen, SLNETIF_ID_1);
    if(status < 0) {
        Display_printf(display, 0, 0, "tcpHandler: failed to load objects\n");
        goto shutdown;
    }

    if (getsockopt(serverFd, SLNETSOCK_LVL_SOCKET, SLNETSOCK_OPSOCK_SLNETSOCKSD,
            &serverSd, &sdlen) < 0) {
        Display_printf(display, 0, 0, "tcpHandler: getsockopt failed\n");
        goto shutdown;
    }

    secAttribHdl = SlNetSock_secAttribCreate();
    /*status |= SlNetSock_secAttribSet(secAttribHdl,
            SLNETSOCK_SEC_ATTRIB_PEER_ROOT_CA, ROOT_CA_CERT_FILE,
            sizeof(ROOT_CA_CERT_FILE));*/
    status |= SlNetSock_secAttribSet(secAttribHdl,
            SLNETSOCK_SEC_ATTRIB_PRIVATE_KEY, PRIVATE_KEY_FILE,
            sizeof(PRIVATE_KEY_FILE));
    /* Setting up a chain (root set first) */
  /*  status |= SlNetSock_secAttribSet(secAttribHdl,
            SLNETSOCK_SEC_ATTRIB_LOCAL_CERT, ROOT_CA_CERT_FILE,
            sizeof(ROOT_CA_CERT_FILE));*/
    status |= SlNetSock_secAttribSet(secAttribHdl,
            SLNETSOCK_SEC_ATTRIB_LOCAL_CERT, TRUSTED_CERT_FILE,
            sizeof(TRUSTED_CERT_FILE));

    status |= SlNetSock_startSec(serverSd, secAttribHdl,
            SLNETSOCK_SEC_BIND_CONTEXT_ONLY | SLNETSOCK_SEC_IS_SERVER);
    if(status < 0) {
        Display_printf(display, 0, 0,
                "tcpHandler: startSec failed to bind context\n");
        goto shutdown;
    }

    memset(&localAddr, 0, sizeof(localAddr));
    localAddr.sin_family = AF_INET;
    localAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    localAddr.sin_port = htons(arg0);

    status = bind(serverFd, (struct sockaddr *)&localAddr, sizeof(localAddr));
    if (status == -1) {
        Display_printf(display, 0, 0, "tcpHandler: bind failed\n");
        goto shutdown;
    }

    status = listen(serverFd, NUMTCPWORKERS);
    if (status == -1) {
        Display_printf(display, 0, 0, "tcpHandler: listen failed\n");
        goto shutdown;
    }

    optval = 1;
    if (setsockopt(serverFd, SOL_SOCKET, SO_KEEPALIVE, &optval, optlen) < 0) {
        Display_printf(display, 0, 0, "tcpHandler: setsockopt failed\n");
        goto shutdown;
    }

    while ((clientFd =
            accept(serverFd, (struct sockaddr *)&clientAddr, &addrlen)) != -1) {

        Display_printf(display, 0, 0,
                "tcpHandler: Creating thread clientFd = %x\n", clientFd);

        if (getsockopt(clientFd, SLNETSOCK_LVL_SOCKET,
                SLNETSOCK_OPSOCK_SLNETSOCKSD,
                &clientSd, &sdlen) < 0) {
            Display_printf(display, 0, 0, "tcpHandler: getsockopt failed\n");
            goto shutdown;
        }

        status = SlNetSock_startSec(clientSd, secAttribHdl,
                SLNETSOCK_SEC_START_SECURITY_SESSION_ONLY |
                SLNETSOCK_SEC_IS_SERVER);
        if(status < 0) {
            Display_printf(display, 0, 0,
                    "tcpHandler: startSec failed to start session\n");
            goto shutdown;
        }

        thread = TaskCreate(tcpWorker, NULL, 3, 2048, (uintptr_t) clientFd,
                0, 0);

        if (!thread) {
            Display_printf(display, 0, 0,
                    "tcpHandler: Error - Failed to create new Task.\n");
            close(clientFd);
        }

        /* addrlen is a value-result param, must reset for next accept call */
        addrlen = sizeof(clientAddr);
    }

    Display_printf(display, 0, 0, "tcpHandler: accept failed.\n");

shutdown:
    if (serverFd != -1) {
        close(serverFd);
    }
    if (secAttribHdl != NULL) {
        SlNetSock_secAttribDelete(secAttribHdl);
    }
}
