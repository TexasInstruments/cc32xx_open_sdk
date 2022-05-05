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

   Application Name     -   MQTT Server
   Application Overview -   The device is running a MQTT server which allows
                           local MQTT client to communicate with each other.
                           Simultaneously, it is also running a client which is
                           connected to the online broker. The interface
                           between the on-board client and the server is such
                           that the local clients can also communicate with the
                           remote MQTT clients, which are connected to the same
                           online broker as the on-board client.

   Application Details  - Refer to 'MQTT Server' README.html

*****************************************************************************/
//*****************************************************************************
//
//! \addtogroup mqtt_server
//! @{
//
//*****************************************************************************
/* Standard includes                                                         */
#include <stdlib.h>
#include <pthread.h>
#include <mqueue.h>
#include <time.h>
#include <unistd.h>

/* Simplelink includes                                                       */
#include <ti/drivers/net/wifi/simplelink.h>

/* SlNetSock includes                                                        */
#include <ti/drivers/net/wifi/slnetifwifi.h>

/* TI-Driver includes                                                        */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>

/* MQTT Library includes                                                     */
#include <ti/net/mqtt/mqttclient.h>
#include <ti/net/mqtt/mqttserver.h>

/* Common interface includes                                                 */
#include "network_if.h"
#include "uart_term.h"

/* TI-DRIVERS Header files */
#include "ti_drivers_config.h"

/* Application includes                                                      */
#include "server_client_cbs.h"

//*****************************************************************************
//                          LOCAL DEFINES
//*****************************************************************************
#define ENABLE_SERVER            1

#if ENABLE_SERVER
/* enables secured server                                                    */
//#define SECURE_SERVER

/* enables authentication of Clients by the Server                           */
/* Server authentication must be enable in order to enable the user name and */
/* password authentication mechanism                                         */
//#define SRVR_USR_PWD
#endif

#define ENABLE_CLIENT            1

#if ENABLE_CLIENT
/* enables secured client                                                    */
//#define SECURE_CLIENT

/* enables client authentication by the server                               */
//#define CLNT_USR_PWD
#endif

#define CLIENT_INIT_STATE        (0x01)
#define SERVER_INIT_STATE        (0x02)
#define MQTT_INIT_STATE          (0x04)

#define APPLICATION_VERSION      "1.1.1"
#define APPLICATION_NAME         "MQTT client server"

#define SLNET_IF_WIFI_PRIO       (5)

/* Operate Lib in MQTT 3.1 mode.                                             */
#define MQTT_3_1_1               false
#define MQTT_3_1                 true

#define WILL_TOPIC               "Client"
#define WILL_MSG                 "Client Stopped"
#define WILL_QOS                 MQTT_QOS_2
#define WILL_RETAIN              false

/* Defining Broker IP address and port Number                                */
//#define SERVER_ADDRESS           "messagesight.demos.ibm.com"
#define SERVER_ADDRESS           "m2m.eclipse.org"
#define SERVER_IP_ADDRESS        "192.168.178.67"
#define PORT_NUMBER              1883
#define SECURED_PORT_NUMBER      8883

/* Clean session flag                                                        */
#define CLEAN_SESSION            true

/* Retain Flag. Used in publish message.                                     */
#define RETAIN_ENABLE            1

/* Defining Number of subscription topics                                    */
#define SUBSCRIPTION_TOPIC_COUNT 1

/* Defining Subscription Topic Values                                        */
#define SUBSCRIPTION_TOPIC0      "/Broker/To/cc32xx"

/* Defining Publish Topic Values                                             */
#define PUBLISH_TOPIC0           "/cc32xx/ButtonPressEvtSw2"
#define PUBLISH_TOPIC0_DATA \
    "Push Button SW2 has been pressed on CC32xx device"

/* Defining Enrolled Topic Values.                                           */
#define ENROLLED_TOPIC           "/cc32xx/To/Broker"

/* Spawn task priority and Task and Thread Stack Size                        */
#define TASKSTACKSIZE            2048
#define RXTASKSIZE               4096
#define MQTTTHREADSIZE           2048
#define SPAWN_TASK_PRIORITY      9

/* secured client requires time configuration, in order to verify server     */
/* certificate validity (date).                                              */

/* Day of month (DD format) range 1-31                                       */
#define DAY                      1
/* Month (MM format) in the range of 1-12                                    */
#define MONTH                    5
/* Year (YYYY format)                                                        */
#define YEAR                     2017
/* Hours in the range of 0-23                                                */
#define HOUR                     12
/* Minutes in the range of 0-59                                              */
#define MINUTES                  33
/* Seconds in the range of 0-59                                              */
#define SEC                      21

/* Number of files used for secure connection                                */
#define CLIENT_NUM_SECURE_FILES  1

/* Expiration value for the timer that is being used to toggle the Led.      */
#define TIMER_EXPIRATION_VALUE   100 * 1000000

//*****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//*****************************************************************************
void pushButtonInterruptHandler2(uint_least8_t index);
void pushButtonInterruptHandler3(uint_least8_t index);
void TimerPeriodicIntHandler(sigval val);
void LedTimerConfigNStart();
void LedTimerDeinitStop();
static void DisplayBanner(char * AppName);
void * MqttClientServer(void *pvParameters);
void Mqtt_ClientStop(uint8_t disconnect);
void Mqtt_ServerStop();
void Mqtt_Stop();
void Mqtt_start();
int32_t Mqtt_IF_Connect();
int32_t MqttServer_start();
int32_t MqttClient_start();
int32_t MQTT_SendMsgToQueue(struct msgQueue *queueElement);

//****************************************************************************
//                         EXTERNAL FUNTIONS
//****************************************************************************
extern int32_t ti_net_SlNet_initConfig();

//*****************************************************************************
//                 GLOBAL VARIABLES
//*****************************************************************************

/* Connection state: (0) - connected, (negative) - disconnected              */
int32_t gApConnectionState = -1;
uint32_t gInitState = 0;
uint32_t memPtrCounterfree = 0;
bool gResetApplication = false;
static MQTTClient_Handle gMqttClient;
static MQTTServer_Handle gMqttServer;
MQTTClient_Params MqttClientExmple_params;
MQTTServer_Params MqttServerExmple_params;
unsigned short g_usTimerInts;

/* Receive task handle                                                       */
pthread_t g_rx_task_hndl = (pthread_t) NULL;
pthread_t g_server_task_hndl = (pthread_t) NULL;
uint32_t gUiConnFlag = 0;

/* AP Security Parameters                                                    */
SlWlanSecParams_t SecurityParams = { 0 };

/* Client ID                                                                 */
/* If ClientId isn't set, the MAC address of the device will be copied into  */
/* the ClientID parameter.                                                   */
char ClientId[13] = {'\0'};

/* Client User Name and Password                                             */
const char *ClientUsername = "username1";
const char *ClientPassword = "pwd1";

/* Server User Name and Password                                             */
const char *ServerUsername = "username1";
const char *ServerPassword = "pwd1";

/* Subscription topics and qos values                                        */
char *topic[SUBSCRIPTION_TOPIC_COUNT] = { SUBSCRIPTION_TOPIC0 };

unsigned char qos[SUBSCRIPTION_TOPIC_COUNT] =
{ MQTT_QOS_2 };

/* Publishing topics and messages                                            */
const char *publish_topic = { PUBLISH_TOPIC0 };
const char *publish_data = { PUBLISH_TOPIC0_DATA };

/* Message Queue                                                             */
mqd_t g_PBQueue;
pthread_t mqttThread = (pthread_t) NULL;
pthread_t appThread = (pthread_t) NULL;
timer_t g_timer;

/* Printing new line                                                         */
char lineBreak[] = "\n\r";

//*****************************************************************************
//                 Banner VARIABLES
//*****************************************************************************
#ifdef  SECURE_CLIENT

char *Mqtt_Client_secure_files[CLIENT_NUM_SECURE_FILES] = {"ca-cert.pem"};

/* Initialization structure to be used with sl_ExtMqtt_Init API. In order to */
/* use secured socket method, the flag MQTTCLIENT_NETCONN_SEC, cipher,       */
/* n_files and secure_files must be configured.                              */
/* certificates also must be programmed  ("ca-cert.pem").                    */
/* The first parameter is a bit mask which configures server address type and*/
/* security mode.                                                            */
/* Server address type: IPv4, IPv6 and URL must be declared with The         */
/* corresponding flag.                                                       */
/* Security mode: The flag MQTTCLIENT_NETCONN_SEC enables the security (TLS) */
/* which includes domain name verification and certificate catalog           */
/*verification, those verifications can be disabled by adding to the bit mask*/
/* MQTTCLIENT_NETCONN_SKIP_DOMAIN_NAME_VERIFICATION and                      */
/* MQTTCLIENT_NETCONN_SKIP_CERTIFICATE_CATALOG_VERIFICATION flags            */
/* Example: MQTTCLIENT_NETCONN_IP6 | MQTTCLIENT_NETCONN_SEC |                */
/* MQTTCLIENT_NETCONN_SKIP_CERTIFICATE_CATALOG_VERIFICATION                  */
/* For this bit mask, the IPv6 address type will be in use, the security     */
/* feature will be enable and the certificate catalog verification will be   */
/* skipped.                                                                  */
/* Note: The domain name verification requires URL Server address type       */
/*       otherwise, this verification will be disabled.                      */
MQTTClient_ConnParams Mqtt_ClientCtx =
{
    MQTTCLIENT_NETCONN_IP4 | MQTTCLIENT_NETCONN_SEC,
    SERVER_IP_ADDRESS,  //SERVER_ADDRESS,
    SECURED_PORT_NUMBER, //  PORT_NUMBER
    SLNETSOCK_SEC_METHOD_SSLv3_TLSV1_2,
    SLNETSOCK_SEC_CIPHER_FULL_LIST,
    CLIENT_NUM_SECURE_FILES,
    Mqtt_Client_secure_files
};

void setTime()
{
    SlDateTime_t dateTime = {0};
    dateTime.tm_day = (uint32_t)DAY;
    dateTime.tm_mon = (uint32_t)MONTH;
    dateTime.tm_year = (uint32_t)YEAR;
    dateTime.tm_hour = (uint32_t)HOUR;
    dateTime.tm_min = (uint32_t)MINUTES;
    dateTime.tm_sec = (uint32_t)SEC;
    sl_DeviceSet(SL_DEVICE_GENERAL, SL_DEVICE_GENERAL_DATE_TIME,
                 sizeof(SlDateTime_t), (uint8_t *)(&dateTime));
}

#else
MQTTClient_ConnParams Mqtt_ClientCtx =
{
    MQTTCLIENT_NETCONN_URL,
    SERVER_ADDRESS,
    PORT_NUMBER, 0, 0, 0,
    NULL
};
#endif

#ifdef  SECURE_SERVER
/* In order to use secured socket method, cipher, n_files and secure_files   */
/* must be configured. certificates also must be programmed                  */
/* ("server-key.pem", "server-cert.pem")                                     */
#define SERVER_NUM_SECURE_FILES 2
char *Mqtt_Server_secure_files[SERVER_NUM_SECURE_FILES] =
{   "server-key.pem", "server-cert.pem"};
MQTTServer_ConnParams Mqtt_Server =
{
    SECURED_PORT_NUMBER,
    SLNETSOCK_SEC_METHOD_SSLv3_TLSV1_2,
    SLNETSOCK_SEC_CIPHER_FULL_LIST,
    SERVER_NUM_SECURE_FILES,
    Mqtt_Server_secure_files
};
#else
MQTTServer_ConnParams Mqtt_Server =
{
    PORT_NUMBER,
    0,
    0,
    0,
    NULL
};
#endif

/* Initialize the will_param structure to the default will parameters        */
MQTTClient_Will will_param =
{
    WILL_TOPIC,
    WILL_MSG,
    WILL_QOS,
    WILL_RETAIN
};

//*****************************************************************************
//
//! MQTT_SendMsgToQueue - Utility function that receive msgQueue parameter and
//! tries to push it the queue with minimal time for timeout of 0.
//! If the queue isn't full the parameter will be stored and the function
//! will return 0.
//! If the queue is full and the timeout expired (because the timeout parameter
//! is 0 it will expire immediately), the parameter is thrown away and the
//! function will return -1 as an error for full queue.
//!
//! \param[in] struct msgQueue *queueElement
//!
//! \return 0 on success, -1 on error
//
//*****************************************************************************
int32_t MQTT_SendMsgToQueue(struct msgQueue *queueElement)
{
    struct timespec abstime = {0};

    clock_gettime(CLOCK_REALTIME, &abstime);

    if(g_PBQueue)
    {
        /* send message to the queue                                         */
        if(mq_timedsend(g_PBQueue, (char *) queueElement,
                        sizeof(struct msgQueue), 0, &abstime) == 0)
        {
            return(0);
        }
    }
    return(-1);
}

//*****************************************************************************
//
//! Push Button Handler1(GPIOSW2). Press push button1 (GPIOSW2) Whenever user
//! wants to publish a message. Write message into message queue signaling the
//! event publish messages
//!
//! \param none
//!
//! return none
//
//*****************************************************************************
void pushButtonInterruptHandler2(uint_least8_t index)
{
    struct msgQueue queueElement;

    /* Disable the SW2 interrupt */
    GPIO_disableInt(CONFIG_GPIO_BUTTON_0); // SW2

    queueElement.event = PUBLISH_PUSH_BUTTON_PRESSED;
    queueElement.msgPtr = NULL;

    /* write message indicating publish message                              */
    if(MQTT_SendMsgToQueue(&queueElement))
    {
        UART_PRINT("\n\n\rQueue is full\n\n\r");
    }
}

//*****************************************************************************
//
//! Push Button Handler2(GPIOSW3). Press push button3 Whenever user wants to
//! disconnect from the remote broker. Write message into message queue
//! indicating disconnect from broker.
//!
//! \param none
//!
//! return none
//
//*****************************************************************************
void pushButtonInterruptHandler3(uint_least8_t index)
{
    struct msgQueue queueElement;
    struct msgQueue queueElemRecv;

    queueElement.event = DISC_PUSH_BUTTON_PRESSED;
    queueElement.msgPtr = NULL;

    /* write message indicating disconnect push button pressed message       */
    if(MQTT_SendMsgToQueue(&queueElement))
    {
        UART_PRINT(
            "\n\n\rQueue is full, throw first msg and send the new one\n\n\r");
        mq_receive(g_PBQueue, (char*) &queueElemRecv, sizeof(struct msgQueue),
                   NULL);
        MQTT_SendMsgToQueue(&queueElement);
    }
}

//*****************************************************************************
//
//! Periodic Timer Interrupt Handler
//!
//! \param None
//!
//! \return None
//
//*****************************************************************************
void TimerPeriodicIntHandler(sigval val)
{
    /* Increment our interrupt counter.                                      */
    g_usTimerInts++;

    if(!(g_usTimerInts & 0x1))
    {
        /* Turn Led Off                                                      */
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
    }
    else
    {
        /* Turn Led On                                                       */
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
    }
}

//*****************************************************************************
//
//! Function to configure and start timer to blink the LED while device is
//! trying to connect to an AP
//!
//! \param none
//!
//! return none
//
//*****************************************************************************
void LedTimerConfigNStart()
{
    struct itimerspec value;
    sigevent sev;

    /* Create Timer                                                          */
    sev.sigev_notify = SIGEV_SIGNAL;
    sev.sigev_notify_function = &TimerPeriodicIntHandler;
    timer_create(2, &sev, &g_timer);

    /* start timer                                                           */
    value.it_interval.tv_sec = 0;
    value.it_interval.tv_nsec = TIMER_EXPIRATION_VALUE;
    value.it_value.tv_sec = 0;
    value.it_value.tv_nsec = TIMER_EXPIRATION_VALUE;

    timer_settime(g_timer, 0, &value, NULL);
}

//*****************************************************************************
//
//! Disable the LED blinking Timer as Device is connected to AP
//!
//! \param none
//!
//! return none
//
//*****************************************************************************
void LedTimerDeinitStop()
{
    /* Disable the LED blinking Timer as Device is connected to AP.          */
    timer_delete(g_timer);
}

//*****************************************************************************
//
//! Application startup display on UART
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
static void DisplayBanner(char * AppName)
{
    UART_PRINT("\n\n\n\r");
    UART_PRINT("\t\t *************************************************\n\r");
    UART_PRINT("\t\t    CC32xx %s Application       \n\r", AppName);
    UART_PRINT("\t\t *************************************************\n\r");
    UART_PRINT("\n\n\n\r");
}

void * MqttClientThread(void * pvParameters)
{
    struct msgQueue queueElement;
    struct msgQueue queueElemRecv;

    MQTTClient_run((MQTTClient_Handle)pvParameters);

    queueElement.event = LOCAL_CLIENT_DISCONNECTION;
    queueElement.msgPtr = NULL;

    /* write message indicating disconnect Broker message.                   */
    if(MQTT_SendMsgToQueue(&queueElement))
    {
        UART_PRINT(
            "\n\n\rQueue is full, throw first msg and send the new one\n\n\r");
        mq_receive(g_PBQueue, (char*) &queueElemRecv, sizeof(struct msgQueue),
                   NULL);
        MQTT_SendMsgToQueue(&queueElement);
    }

    pthread_exit(0);

    return(NULL);
}

void * MqttServerThread(void *pvParameters)
{
    MQTTServer_run(gMqttServer);
    pthread_exit(0);

    return(NULL);
}

//*****************************************************************************
//
//! Task implementing MQTT Server plus client bridge
//!
//! This function
//!    1. Initializes network driver and connects to the default AP
//!    2. Initializes the mqtt client ans server libraries and set up MQTT
//!       with the remote broker.
//!    3. set up the button events and their callbacks(for publishing)
//!    4. handles the callback signals
//!
//! \param  none
//!
//! \return None
//!
//*****************************************************************************
void * MqttClientServer(void *pvParameters)
{
    struct msgQueue queueElemRecv;
    struct publishMsgHeader msgHead;
    uint32_t uiTopOffset = 0;
    uint32_t uiPayloadOffset = 0;
    long lRetVal = -1;
    uint32_t flags;

    memset(&msgHead, 0, sizeof(msgHead));

#if ENABLE_SERVER
    lRetVal = MqttServer_start();
    if(lRetVal == -1)
    {
        UART_PRINT("MQTT Server lib initialization failed\n\r");
        pthread_exit(0);
        return(NULL);
    }
#endif

    /* Initializing Client and Subscribing to the Broker.                    */
#if ENABLE_CLIENT
    if(gApConnectionState >= 0)
    {
        lRetVal = MqttClient_start();
        if(lRetVal == -1)
        {
            UART_PRINT("MQTT Client lib initialization failed\n\r");
            pthread_exit(0);
            return(NULL);
        }
    }
#endif

    /*handling the signals from various callbacks including the push button  */
    /*prompting the client to publish a msg on PUB_TOPIC OR msg received by  */
    /*the server on enrolled topic(for which the on-board client ha enrolled)*/
    /*from a local client(will be published to the remote broker by the      */
    /*client) OR msg received by the client from the remote broker (need to  */
    /*be sent to the server to see if any local client has subscribed on the */
    /*same topic).                                                           */
    for(;; )
    {
        /* waiting for signals                                               */
        mq_receive(g_PBQueue, (char*) &queueElemRecv, sizeof(struct msgQueue),
                   NULL);

        switch(queueElemRecv.event)
        {
        case PUBLISH_PUSH_BUTTON_PRESSED:

            /* send publish message                                       */
            lRetVal =
                MQTTClient_publish(gMqttClient, (char*) publish_topic, strlen(
                                      (char*)publish_topic),
                                  (char*)publish_data,
                                  strlen((char*) publish_data), MQTT_QOS_2 |
                                  ((RETAIN_ENABLE) ? MQTT_PUBLISH_RETAIN : 0));

            UART_PRINT("\n\r CC3200 Publishes the following message \n\r");
            UART_PRINT("Topic: %s\n\r", publish_topic);
            UART_PRINT("Data: %s\n\r", publish_data);

            /* Clear and enable again the SW2 interrupt */
            GPIO_clearInt(CONFIG_GPIO_BUTTON_0);     // SW2
            GPIO_enableInt(CONFIG_GPIO_BUTTON_0);     // SW2

            break;

        /* msg received by server (on the enrolled topic by on-board  */
        /* client) publish it to the remote broker                    */
        case MSG_RECV_BY_SERVER:
            if(gUiConnFlag == 0)
            {
                free(queueElemRecv.msgPtr);
                memPtrCounterfree++;
                break;
            }

            memcpy(&msgHead, queueElemRecv.msgPtr, sizeof(msgHead));
            uiTopOffset = sizeof(msgHead);
            uiPayloadOffset = uiTopOffset + msgHead.topicLen + 1;
            flags = msgHead.qos;
            if(msgHead.retain)
            {
                flags = flags | MQTT_PUBLISH_RETAIN;
            }

            lRetVal =
                MQTTClient_publish(gMqttClient,
                                   ((char *) (queueElemRecv.msgPtr) +
                                    uiTopOffset),
                                   strlen(
                                       ((char *) (queueElemRecv.msgPtr) +
                                        uiTopOffset)),
                                   ((char*) (queueElemRecv.msgPtr) +
                                    uiPayloadOffset), msgHead.payLen,
                                   flags);
            free(queueElemRecv.msgPtr);
            memPtrCounterfree++;
            break;

        /* msg received by client from remote broker (on a topic      */
        /* subscribed by local client)                                */
        case MSG_RECV_BY_CLIENT:
        {
            flags = msgHead.qos;
            if(msgHead.retain)
            {
                flags = flags | MQTT_PUBLISH_RETAIN;
            }
            memcpy(&msgHead, queueElemRecv.msgPtr, sizeof(msgHead));
            uiTopOffset = sizeof(msgHead);
            uiPayloadOffset = uiTopOffset + msgHead.topicLen + 1;
            MQTTServer_publish(gMqttServer,
                               ((char*) (queueElemRecv.msgPtr) + uiTopOffset),
                               strlen(
                                   ((char*) (queueElemRecv.msgPtr) +
                                    uiTopOffset)),
                               ((char*) (queueElemRecv.msgPtr) +
                                uiPayloadOffset),
                               msgHead.payLen, flags);
            free(queueElemRecv.msgPtr);
            memPtrCounterfree++;
        }
        break;

        /* On-board client disconnected from remote broker, only      */
        /* local MQTT network will work                               */
        case LOCAL_CLIENT_DISCONNECTION:
            UART_PRINT("\n\rOn-board Client Disconnected\n\r");
            UART_PRINT(
                "In order to reconnect local client, Reset board.\n\r\n\r");
            gUiConnFlag = 0;
            break;

        /* Remote client disconnected from local broker               */
        /* local MQTT network continue to work                        */
        case REMOTE_CLIENT_DISCONNECTION:
            UART_PRINT("\n\rRemote Client Disconnected\n\r\n\r");
            break;

        /* Push button for full restart check                         */
        case DISC_PUSH_BUTTON_PRESSED:
            gResetApplication = true;
            break;

        case THREAD_TERMINATE_REQ:
            gUiConnFlag = 0;
            pthread_exit(0);
            return(NULL);

        default:
            sleep(1);
            break;
        }
    }
}

//*****************************************************************************
//
//! This function connect the MQTT device to an AP with the SSID which was
//! configured in SSID_NAME definition which can be found in Network_if.h file,
//! if the device can't connect to to this AP a request from the user for other
//! SSID will appear.
//!
//! \param  none
//!
//! \return None
//!
//*****************************************************************************
int32_t Mqtt_IF_Connect()
{
    int32_t lRetVal;
    char SSID_Remote_Name[32];
    int8_t Str_Length;

    memset(SSID_Remote_Name, '\0', sizeof(SSID_Remote_Name));
    Str_Length = strlen(SSID_NAME);

    if(Str_Length)
    {
        /* Copy the Default SSID to the local variable                       */
        strncpy(SSID_Remote_Name, SSID_NAME, Str_Length);
    }

    /* Display Application Banner                                            */
    DisplayBanner(APPLICATION_NAME);

    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
    GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
    GPIO_write(CONFIG_GPIO_LED_2, CONFIG_GPIO_LED_OFF);

    /* Reset The state of the machine                                        */
    Network_IF_ResetMCUStateMachine();

    /* Start the driver                                                      */
    lRetVal = Network_IF_InitDriver(ROLE_STA);
    if(lRetVal < 0)
    {
        UART_PRINT("Failed to start SimpleLink Device\n\r", lRetVal);
        return(-1);
    }

    /* switch on CONFIG_GPIO_LED_2 to indicate Simplelink is properly up.      */
    GPIO_write(CONFIG_GPIO_LED_2, CONFIG_GPIO_LED_ON);

    /* Start Timer to blink CONFIG_GPIO_LED_0 till AP connection               */
    LedTimerConfigNStart();

    /* Initialize AP security params                                         */
    SecurityParams.Key = (signed char *) SECURITY_KEY;
    SecurityParams.KeyLen = strlen(SECURITY_KEY);
    SecurityParams.Type = SECURITY_TYPE;

    /* Connect to the Access Point                                           */
    lRetVal = Network_IF_ConnectAP(SSID_Remote_Name, SecurityParams);
    if(lRetVal < 0)
    {
        UART_PRINT("Connection to an AP failed\n\r");
        return(-1);
    }

    /* Disable the LED blinking Timer as Device is connected to AP.          */
    LedTimerDeinitStop();

    /* Switch ON CONFIG_GPIO_LED_0 to indicate that Device acquired an IP.     */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

    sleep(1);

    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
    GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
    GPIO_write(CONFIG_GPIO_LED_2, CONFIG_GPIO_LED_OFF);

    return(0);
}

//*****************************************************************************
//!
//! MQTT Start - Initialize and create all the items required to run the MQTT
//! protocol
//!
//! \param  none
//!
//! \return None
//!
//*****************************************************************************
void Mqtt_start()
{
    pthread_attr_t pAttrs;
    struct sched_param priParam;
    int32_t retc = 0;
    mq_attr attr;
    unsigned mode = 0;

    /* sync object for inter thread communication                            */
    attr.mq_maxmsg = 10;
    attr.mq_msgsize = sizeof(struct msgQueue);
    g_PBQueue = mq_open("g_PBQueue", O_CREAT, mode, &attr);

    if(((int)g_PBQueue) <= 0)
    {
        UART_PRINT("MQTT Message Queue create fail\n\r");
        gInitState &= ~MQTT_INIT_STATE;
        return;
    }

    /* Set priority and stack size attributes                                */
    pthread_attr_init(&pAttrs);
    priParam.sched_priority = 2;
    retc = pthread_attr_setschedparam(&pAttrs, &priParam);
    retc |= pthread_attr_setstacksize(&pAttrs, MQTTTHREADSIZE);
    retc |= pthread_attr_setdetachstate(&pAttrs, PTHREAD_CREATE_DETACHED);

    if(retc != 0)
    {
        gInitState &= ~MQTT_INIT_STATE;
        UART_PRINT("MQTT thread create fail\n\r");
        return;
    }

    retc = pthread_create
    (&mqttThread, &pAttrs, MqttClientServer, (void *) NULL);
    if(retc != 0)
    {
        gInitState &= ~MQTT_INIT_STATE;
        UART_PRINT("MQTT thread create fail\n\r");
        return;
    }

    /* enable interrupt for the GPIO 13 (SW3) and GPIO 22 (SW2).             */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, pushButtonInterruptHandler2);
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0); // SW2

    GPIO_setCallback(CONFIG_GPIO_BUTTON_1, pushButtonInterruptHandler3);
    GPIO_enableInt(CONFIG_GPIO_BUTTON_1); // SW3

    gInitState &= ~MQTT_INIT_STATE;
}

//*****************************************************************************
//!
//! MQTT Stop - Close the server and or client instance and free all the items
//! required to run the MQTT protocol
//!
//! \param  none
//!
//! \return None
//!
//*****************************************************************************

void Mqtt_Stop()
{
    struct msgQueue queueElement;
    struct msgQueue queueElemRecv;

#if ENABLE_CLIENT
    if(gApConnectionState >= 0)
    {
        Mqtt_ClientStop(1);
    }
#endif

#if ENABLE_SERVER
    Mqtt_ServerStop();
#endif
    queueElement.event = THREAD_TERMINATE_REQ;
    queueElement.msgPtr = NULL;

    /* write message indicating publish message                              */
    if(MQTT_SendMsgToQueue(&queueElement))
    {
        UART_PRINT(
            "\n\n\rQueue is full, throw first msg and send the new one\n\n\r");
        mq_receive(g_PBQueue, (char*) &queueElemRecv, sizeof(struct msgQueue),
                   NULL);
        MQTT_SendMsgToQueue(&queueElement);
    }

    sleep(2);

    mq_close(g_PBQueue);
    g_PBQueue = NULL;

    sl_Stop(SL_STOP_TIMEOUT);
    UART_PRINT("\n\r Client Stop completed\r\n");

    /* Disable the SW2 and SW3 interrupt */
    GPIO_disableInt(CONFIG_GPIO_BUTTON_0); // SW2
    GPIO_disableInt(CONFIG_GPIO_BUTTON_1); // SW3
}

int32_t MqttServer_start()
{
    int32_t lRetVal;
    pthread_attr_t pAttrs;
    struct sched_param priParam;

    MQTTServer_SubscribeParams params = { ENROLLED_TOPIC, NULL, 0 };

    gInitState |= SERVER_INIT_STATE;
    MqttServerExmple_params.connParams = &Mqtt_Server;

    /* Initializing server and registering callbacks                         */
    gMqttServer = MQTTServer_create(MqttServerCallback,
                                    &MqttServerExmple_params);

    /* start the Server Run task                                             */
    pthread_attr_init(&pAttrs);
    priParam.sched_priority = 2;
    lRetVal = pthread_attr_setschedparam(&pAttrs, &priParam);
    lRetVal |= pthread_attr_setstacksize(&pAttrs, 4096);
    lRetVal |= pthread_attr_setdetachstate(&pAttrs, PTHREAD_CREATE_DETACHED);

    lRetVal |=
        pthread_create(&g_server_task_hndl, &pAttrs, MqttServerThread,
                       (void *) NULL);
    gInitState &= ~SERVER_INIT_STATE;
    if(lRetVal != 0)
    {
        UART_PRINT("MQTT Server lib initialization failed\n\r");
        return(-1);
    }

#ifdef SRVR_USR_PWD
    /* Set user name for client connection                                   */
    MQTTServer_set(gMqttClient, MQTTServer_USER_NAME, (void *)ServerUsername,
                   strlen(
                       (char*)ServerUsername));

    /* Set password                                                          */
    MQTTServer_set(gMqttClient, MQTTServer_PASSWORD, (void *)ServerPassword,
                   strlen(
                       (char*)ServerPassword));
#endif

    /* on-board client is enrolling to server. Server will treated this      */
    /* enrollment as a subscription.                                         */
    MQTTServer_subscribe(gMqttServer, &params, 1);
    return(0);
}

void Mqtt_ServerStop()
{
    MQTTServer_UnSubscribeParams params = { ENROLLED_TOPIC,  0 };

    MQTTServer_unsubscribe(gMqttServer, &params, 1);
    MQTTServer_delete(gMqttServer);
}

int32_t MqttClient_start()
{
    int32_t lRetVal = -1;
    int32_t iCount = 0;

    pthread_attr_t pAttrs;
    struct sched_param priParam;

    MqttClientExmple_params.clientId = ClientId;
    MqttClientExmple_params.connParams = &Mqtt_ClientCtx;
    MqttClientExmple_params.mqttMode31 = MQTT_3_1;
    MqttClientExmple_params.blockingSend = true;

    gInitState |= CLIENT_INIT_STATE;

    /* Initialize MQTT client lib                                            */
    gMqttClient = MQTTClient_create(MqttClientCallback,
                                    &MqttClientExmple_params);
    if(gMqttClient == NULL)
    {
        /* lib initialization failed                                         */
        gInitState &= ~CLIENT_INIT_STATE;
        return(-1);
    }

    /* Open Client Receive Thread start the receive task. Set priority and   */
    /* stack size attributes                                                 */
    pthread_attr_init(&pAttrs);
    priParam.sched_priority = 2;
    lRetVal = pthread_attr_setschedparam(&pAttrs, &priParam);
    lRetVal |= pthread_attr_setstacksize(&pAttrs, RXTASKSIZE);
    lRetVal |= pthread_attr_setdetachstate(&pAttrs, PTHREAD_CREATE_DETACHED);
    lRetVal |=
        pthread_create(&g_rx_task_hndl, &pAttrs, MqttClientThread,
                       (void *) NULL);
    if(lRetVal != 0)
    {
        UART_PRINT("Client Thread Create Failed failed\n\r");
        gInitState &= ~CLIENT_INIT_STATE;
        return(-1);
    }
#ifdef SECURE_CLIENT
    setTime();
#endif

    /* setting will parameters                                               */
    MQTTClient_set(gMqttClient, MQTTClient_WILL_PARAM, &will_param,
                   sizeof(will_param));

#ifdef CLNT_USR_PWD
    /* Set user name for client connection                                   */
    MQTTClient_set(gMqttClient, MQTTClient_USER_NAME, (void *)ClientUsername,
                   strlen(
                       (char*)ClientUsername));

    /* Set password                                                          */
    MQTTClient_set(gMqttClient, MQTTClient_PASSWORD, (void *)ClientPassword,
                   strlen(
                       (char*)ClientPassword));
#endif
    /* Initiate MQTT Connect                                                 */
    if(gApConnectionState >= 0)
    {
#if CLEAN_SESSION == false
        bool clean = CLEAN_SESSION;
        MQTTClient_set(gMqttClient, MQTTClient_CLEAN_CONNECT, (void *)&clean,
                       sizeof(bool));
#endif
        /* The return code of MQTTClient_connect is the ConnACK value that
           returns from the server */
        lRetVal = MQTTClient_connect(gMqttClient);

        /* negative lRetVal means error,
           0 means connection successful without session stored by the server,
           greater than 0 means successful connection with session stored by
           the server */
        if(0 > lRetVal)
        {
            /* lib initialization failed                                     */
            UART_PRINT("Connection to broker failed, Error code: %d\n\r",
                       lRetVal);

            gUiConnFlag = 0;
        }
        else
        {
            gUiConnFlag = 1;
        }
        /* Subscribe to topics when session is not stored by the server      */
        if((gUiConnFlag == 1) && (0 == lRetVal))
        {
            uint8_t subIndex;
            MQTTClient_SubscribeParams subscriptionInfo[
                SUBSCRIPTION_TOPIC_COUNT];

            for(subIndex = 0; subIndex < SUBSCRIPTION_TOPIC_COUNT; subIndex++)
            {
                subscriptionInfo[subIndex].topic = topic[subIndex];
                subscriptionInfo[subIndex].qos = qos[subIndex];
            }

            if(MQTTClient_subscribe(gMqttClient, subscriptionInfo,
                                    SUBSCRIPTION_TOPIC_COUNT) < 0)
            {
                UART_PRINT("\n\r Subscription Error \n\r");
                MQTTClient_disconnect(gMqttClient);
                gUiConnFlag = 0;
            }
            else
            {
                for(iCount = 0; iCount < SUBSCRIPTION_TOPIC_COUNT; iCount++)
                {
                    UART_PRINT("Client subscribed on %s\n\r,", topic[iCount]);
                }
            }
        }
    }

    gInitState &= ~CLIENT_INIT_STATE;

    return(0);
}

//*****************************************************************************
//!
//! MQTT Client stop - Unsubscribe from the subscription topics and exit the
//! MQTT client lib.
//!
//! \param  none
//!
//! \return None
//!
//*****************************************************************************

void Mqtt_ClientStop(uint8_t disconnect)
{
    uint32_t iCount;

    MQTTClient_UnsubscribeParams subscriptionInfo[SUBSCRIPTION_TOPIC_COUNT];

    for(iCount = 0; iCount < SUBSCRIPTION_TOPIC_COUNT; iCount++)
    {
        subscriptionInfo[iCount].topic = topic[iCount];
    }

    MQTTClient_unsubscribe(gMqttClient, subscriptionInfo,
                           SUBSCRIPTION_TOPIC_COUNT);
    for(iCount = 0; iCount < SUBSCRIPTION_TOPIC_COUNT; iCount++)
    {
        UART_PRINT("Unsubscribed from the topic %s\r\n", topic[iCount]);
    }
    gUiConnFlag = 0;

    /* exiting the Client library                                             */
    MQTTClient_delete(gMqttClient);
}

//*****************************************************************************
//!
//! Utility function which prints the borders
//!
//! \param[in] ch  -  hold the charater for the border.
//! \param[in] n   -  hold the size of the border.
//!
//! \return none.
//!
//*****************************************************************************

void printBorder(char ch,
                 int n)
{
    int i = 0;

    for(i = 0; i < n; i++)
    {
        putch(ch);
    }
}

//*****************************************************************************
//!
//! Set the ClientId with its own mac address
//! This routine converts the mac address which is given
//! by an integer type variable in hexadecimal base to ASCII
//! representation, and copies it into the ClientId parameter.
//!
//! \param  macAddress  -   Points to string Hex.
//!
//! \return void.
//!
//*****************************************************************************
int32_t SetClientIdNamefromMacAddress()
{
    int32_t ret = 0;
    uint8_t Client_Mac_Name[2];
    uint8_t Index;
    uint16_t macAddressLen = SL_MAC_ADDR_LEN;
    uint8_t macAddress[SL_MAC_ADDR_LEN];

    /*Get the device Mac address */
    ret = sl_NetCfgGet(SL_NETCFG_MAC_ADDRESS_GET, 0, &macAddressLen,
                       &macAddress[0]);

   /* When ClientID isn't set, use the mac address as ClientID               */
    if(ClientId[0] == '\0')
    {
        /* 6 bytes is the length of the mac address                          */
        for(Index = 0; Index < SL_MAC_ADDR_LEN; Index++)
        {
            /* Each mac address byte contains two hexadecimal characters     */
            /* Copy the 4 MSB - the most significant character               */
            Client_Mac_Name[0] = (macAddress[Index] >> 4) & 0xf;
            /* Copy the 4 LSB - the least significant character              */
            Client_Mac_Name[1] = macAddress[Index] & 0xf;

            if(Client_Mac_Name[0] > 9)
            {
                /* Converts and copies from number that is greater than 9 in */
                /* hexadecimal representation (a to f) into ascii character  */
                ClientId[2 * Index] = Client_Mac_Name[0] + 'a' - 10;
            }
            else
            {
                /* Converts and copies from number 0 - 9 in hexadecimal      */
                /* representation into ascii character                       */
                ClientId[2 * Index] = Client_Mac_Name[0] + '0';
            }
            if(Client_Mac_Name[1] > 9)
            {
                /* Converts and copies from number that is greater than 9 in */
                /* hexadecimal representation (a to f) into ascii character  */
                ClientId[2 * Index + 1] = Client_Mac_Name[1] + 'a' - 10;
            }
            else
            {
                /* Converts and copies from number 0 - 9 in hexadecimal      */
                /* representation into ascii character                       */
                ClientId[2 * Index + 1] = Client_Mac_Name[1] + '0';
            }
        }
    }
    return(ret);
}

//*****************************************************************************
//!
//! Utility function which Display the app banner
//!
//! \param[in] appName     -  holds the application name.
//! \param[in] appVersion  -  holds the application version.
//!
//! \return none.
//!
//*****************************************************************************

int32_t DisplayAppBanner(char* appName,
                         char* appVersion)
{
    int32_t ret = 0;
    uint8_t macAddress[SL_MAC_ADDR_LEN];
    uint16_t macAddressLen = SL_MAC_ADDR_LEN;
    uint16_t ConfigSize = 0;
    uint8_t ConfigOpt = SL_DEVICE_GENERAL_VERSION;
    SlDeviceVersion_t ver = {0};

    ConfigSize = sizeof(SlDeviceVersion_t);

    /* Print device version info. */
    ret =
        sl_DeviceGet(SL_DEVICE_GENERAL, &ConfigOpt, &ConfigSize,
                     (uint8_t*)(&ver));

    /* Print device Mac address */
    ret = sl_NetCfgGet(SL_NETCFG_MAC_ADDRESS_GET, 0, &macAddressLen,
                       &macAddress[0]);

    UART_PRINT(lineBreak);
    UART_PRINT("\t");
    printBorder('=', 44);
    UART_PRINT(lineBreak);
    UART_PRINT("\t   %s Example Ver: %s",appName, appVersion);
    UART_PRINT(lineBreak);
    UART_PRINT("\t");
    printBorder('=', 44);
    UART_PRINT(lineBreak);
    UART_PRINT(lineBreak);
    UART_PRINT("\t CHIP: 0x%x",ver.ChipId);
    UART_PRINT(lineBreak);
    UART_PRINT("\t MAC:  %d.%d.%d.%d",ver.FwVersion[0],ver.FwVersion[1],
               ver.FwVersion[2],
               ver.FwVersion[3]);
    UART_PRINT(lineBreak);
    UART_PRINT("\t PHY:  %d.%d.%d.%d",ver.PhyVersion[0],ver.PhyVersion[1],
               ver.PhyVersion[2],
               ver.PhyVersion[3]);
    UART_PRINT(lineBreak);
    UART_PRINT("\t NWP:  %d.%d.%d.%d",ver.NwpVersion[0],ver.NwpVersion[1],
               ver.NwpVersion[2],
               ver.NwpVersion[3]);
    UART_PRINT(lineBreak);
    UART_PRINT("\t ROM:  %d",ver.RomVersion);
    UART_PRINT(lineBreak);
    UART_PRINT("\t HOST: %s", SL_DRIVER_VERSION);
    UART_PRINT(lineBreak);
    UART_PRINT("\t MAC address: %02x:%02x:%02x:%02x:%02x:%02x",
               macAddress[0],
               macAddress[1],
               macAddress[2], macAddress[3], macAddress[4],
               macAddress[5]);
    UART_PRINT(lineBreak);
    UART_PRINT(lineBreak);
    UART_PRINT("\t");
    printBorder('=', 44);
    UART_PRINT(lineBreak);
    UART_PRINT(lineBreak);


    return(ret);
}

void mainThread(void * args)
{
    uint32_t count = 0;
    pthread_t spawn_thread = (pthread_t) NULL;
    pthread_attr_t pAttrs_spawn;
    struct sched_param priParam;
    int32_t retc = 0;
    UART_Handle tUartHndl;

    /* Initialize SlNetSock layer with CC31xx/CC32xx interface */
    retc = ti_net_SlNet_initConfig();
    if(0 != retc)
    {
        UART_PRINT("Failed to initialize SlNetSock\n\r");
    }

    GPIO_init();
    SPI_init();

    /* Configure the UART */
    tUartHndl = InitTerm();
    /* remove uart receive from LPDS dependency */
    UART_control(tUartHndl, UART_CMD_RXDISABLE, NULL);

    /* Create the sl_Task */
    pthread_attr_init(&pAttrs_spawn);
    priParam.sched_priority = SPAWN_TASK_PRIORITY;
    retc = pthread_attr_setschedparam(&pAttrs_spawn, &priParam);
    retc |= pthread_attr_setstacksize(&pAttrs_spawn, TASKSTACKSIZE);
    retc |= pthread_attr_setdetachstate(&pAttrs_spawn, PTHREAD_CREATE_DETACHED);

    retc = pthread_create(&spawn_thread, &pAttrs_spawn, sl_Task, NULL);

    if(retc != 0)
    {
        UART_PRINT("could not create simplelink task\n\r");
        while(1)
        {
            ;
        }
    }

    retc = sl_Start(0, 0, 0);
    if(retc < 0)
    {
        /* Handle Error */
        UART_PRINT("\n sl_Start failed\n");
        while(1)
        {
            ;
        }
    }

    /* Output device information to the UART terminal */
    retc = DisplayAppBanner(APPLICATION_NAME, APPLICATION_VERSION);
    /* Set the ClientId with its own mac address */
    retc |= SetClientIdNamefromMacAddress();

    retc = sl_Stop(SL_STOP_TIMEOUT);
    if(retc < 0)
    {
        /* Handle Error */
        UART_PRINT("\n sl_Stop failed\n");
        while(1)
        {
            ;
        }
    }

    if(retc < 0)
    {
        /* Handle Error */
        UART_PRINT("mqtt_client - Unable to retrieve device information \n");
        while(1)
        {
            ;
        }
    }

    while(1)
    {
        gResetApplication = false;
        topic[0] = SUBSCRIPTION_TOPIC0;
        gInitState = 0;

        /* Connect to AP */
        gApConnectionState = Mqtt_IF_Connect();

        gInitState |= MQTT_INIT_STATE;
        /* Run MQTT Main Thread (it will open the Client and Server) */
        Mqtt_start();

        /* Wait for init to be completed!!! */
        while(gInitState != 0)
        {
            UART_PRINT(".");
            sleep(1);
        }
        UART_PRINT(".\r\n");

        while(gResetApplication == false)
        {
            sleep(1);
        }

        UART_PRINT("TO Complete - Closing all threads and resources\r\n");

        /* Stop the MQTT Process */
        Mqtt_Stop();

        sleep(1);
        UART_PRINT("reopen MQTT # %d  \r\n", ++count);
    }
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
