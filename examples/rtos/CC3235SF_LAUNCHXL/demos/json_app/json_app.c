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

//*******************************************************
//                  INCLUDES
//*******************************************************
/* Standard includes */
#include <stdarg.h>
#include <unistd.h>

/* TI-DRIVERS Header files */
#include <ti/drivers/net/wifi/simplelink.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/Power.h>

#include <ti/utils/json/json.h>
#include "uart_term.h"
#include "pthread.h"

//*******************************************************
//                  DEFINES - CONSTS
//*******************************************************
#define APPLICATION_NAME                        "JSON"
#define APPLICATION_VERSION                     "1.0.0"
#define SL_STOP_TIMEOUT                         (200)
#define SPAWN_TASK_PRIORITY                     (9)
#define TASKSTACKSIZE                           (4096)
#define TEMPLATE_FILENAME                       "template1"
#define JSON_FILENAME                           "json1"

#define ASCI_0                                  48
#define ASCI_9                                  57
#define OBJ_BUFFER_SIZE                         6
#define CMD_BUFFER_SIZE                         100
#define SELECT_BUFFER_SIZE                      2
//*******************************************************
//                  STRUCTs
//*******************************************************

typedef struct
{
    char *fileBuffer;
} Json_Filename_t;

typedef enum
{
    JSON_CREATE_TEMPLATE = 0,
    JSON_CREATE_OBJECT = 1,
    JSON_PARSE = 2,
    JSON_GET_VALUE = 3,
    JSON_SET_VALUE = 4,
    JSON_GET_ARRAY_MEMBER_COUNT = 5,
    JSON_BUILD = 6,
    JSON_DESTROY_TEMPLATE = 7,
    JSON_DESTROY_JSON_OBJECT = 8
}json_action;

typedef enum
{
    INT32 = 0,
    STRING_RAW = 1,
    BOOLEAN = 2
}json_value_t;
//*******************************************************
//                     GLOBAL VARIABLES
//*******************************************************
Json_Handle templateHandle;
Json_Handle jsonObjHandle;
Json_Filename_t templateBuff;
Json_Filename_t jsonBuffer;
int16_t templateSize;
uint16_t objSize;

//*******************************************************
//                      LOCAL FUNCTION PROTOTYPES
//*******************************************************
int16_t readFile(Json_Filename_t  * pBufferFile,
                 char *FileName);
void removeUnwantedChars(char * pBuf);
void validateForPrint(char *pBuf);
void createTemplate(void);
void createObject(void);
void parse(void);
void getValue(void);
void setValue(void);
void destroyJsonObject(void);
void destroyTemplate(void);
void build(void);
void getArrayMemberCount(void);

void SimpleLinkNetAppRequestMemFreeEventHandler(uint8_t *buffer)
{
    // do nothing...
}

void SimpleLinkNetAppRequestEventHandler(SlNetAppRequest_t *pNetAppRequest,
                                         SlNetAppResponse_t *pNetAppResponse)
{
    // do nothing...
}

//*****************************************************************************
//
//! \brief The Function Handles WLAN Events
//!
//! \param[in]  pWlanEvent - Pointer to WLAN Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent)
{
    // do nothing...
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
    // do nothing...
}

//*****************************************************************************
//
//! \brief This function handles network events such as IP acquisition, IP
//!           leased, IP released etc.
//!
//! \param[in]  pNetAppEvent - Pointer to NetApp Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    // do nothing...
}

//*****************************************************************************
//
//! \brief This function handles HTTP server events
//!
//! \param[in]  pServerEvent - Contains the relevant event information
//! \param[in]    pServerResponse - Should be filled by the user with the
//!                                      relevant response information
//!
//! \return None
//!
//****************************************************************************
void SimpleLinkHttpServerEventHandler(
    SlNetAppHttpServerEvent_t *pHttpEvent,
    SlNetAppHttpServerResponse_t *
    pHttpResponse)
{
    // Unused in this application
}

//*****************************************************************************
//
//! \brief This function handles General Events
//!
//! \param[in]     pDevEvent - Pointer to General Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
    // do nothing...
}

//*****************************************************************************
//
//! This function handles socket events indication
//!
//! \param[in]      pSock - Pointer to Socket Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
    // do nothing...
}

//*****************************************************************************
//
//! \brief This function reads a file from the file system and stores it
//!        in a buffer.
//! \param[out]    pBufferFile - pointer to a buffer which will be filled with
//!  a file
//! \param[in]     FileName    - pointer to filename which needs to be
//!                              read.
//!
//! \return on success number of bytes read from file system
//!         on failure error code.
//****************************************************************************
int16_t readFile(Json_Filename_t *pBufferFile,
                 char *FileName)
{
    int32_t FileHdl = 0;
    int32_t Status = 0;
    SlFsFileInfo_t *FsFileInfo;
    uint32_t FileSize;
    int16_t retVal = 0;

    FsFileInfo = malloc(sizeof(SlFsFileInfo_t));

    if(FsFileInfo)
    {
        /* Get the file size */
        Status = sl_FsGetInfo((unsigned char *)FileName,0,FsFileInfo);
        if(Status < 0)
        {
            UART_PRINT("FS - Couldn't get info on file. error status %d \n\r",
                       Status);
            return(Status);
        }
        FileSize = FsFileInfo->Len;
        free(FsFileInfo);

        FileHdl = sl_FsOpen((unsigned char *)FileName, SL_FS_READ,0);
        if(FileHdl < 0)
        {
           UART_PRINT("FS - Couldn't open file. error status %d \n\r",FileHdl);
           return(FileHdl);
        }
        else
        {
            pBufferFile->fileBuffer = malloc(FileSize + 1);
            if(pBufferFile->fileBuffer != NULL)
            {
                memset(pBufferFile->fileBuffer,'\0',FileSize + 1);
                /* Read the entire file */
                Status =
                    sl_FsRead(FileHdl, 0,
                              (unsigned char *)pBufferFile->fileBuffer,
                              FileSize);
                if(Status < 0)
                {
                    UART_PRINT("FS - Couldn't read file. error status %d \n\r",
                               Status);
                    return(Status);
                }
                retVal = sl_FsClose(FileHdl,NULL,NULL,0);
                if(retVal < 0)
                {
                   UART_PRINT("FS - Couldn't close file. error status %d \n\r",
                               retVal);
                    return(retVal);
                }
                return(Status);
            }
            else
            {
                UART_PRINT("Couldn't allocate memory \n\r");
                return(JSON_RC__MEMORY_ALLOCATION_ERROR);
            }
        }
    }
    else
    {
        UART_PRINT("Couldn't allocate memory \n\r");
        return(JSON_RC__MEMORY_ALLOCATION_ERROR);
    }
}

//*****************************************************************************
//
//! \brief This function removes from the buffer '\n' and ' '
//!
//! \param[inout]      pBuf pointer to a buffer
//!
//! \return none
//****************************************************************************
void removeUnwantedChars(char *pBuf)
{
    char * str_tmp;
    uint16_t i = 0,j = 0;
    str_tmp = pBuf;

    for(i = 0; str_tmp[i] != '\0'; ++i)
    {
        while((!(str_tmp[i] != '\n') ||
               !(str_tmp[i] != ' '))&& (str_tmp[i] != '\0'))
        {
            for(j = i; str_tmp[j] != '\0'; ++j)
            {
                str_tmp[j] = str_tmp[j + 1];
            }
            str_tmp[j] = '\0';
        }
    }
}

//*****************************************************************************
//
//! \brief This function validates and changes the Json text into readable
//!        Json convention
//! \param[inout]      pBuf pointer to a buffer
//!
//! \return none
//****************************************************************************
void validateForPrint(char *pBuf)
{
    char * str_tmp = pBuf;
    char * pre = NULL;
    uint16_t i = 0,j = 0;
    int16_t ident = 0;

    for(i = 0; str_tmp[i] != '\0'; ++i)
    {
        if((str_tmp[i] == ']')|| (str_tmp[i] == '}'))
        {
            ident--;
        }
        if(pre != NULL)
        {
            if((*pre == '[')|| (*pre == '{')|| (*pre == ',')||
               (str_tmp[i] == ']')||(str_tmp[i] == '}'))
            {
                UART_PRINT("\n\r");
                for(j = 0; j < ident; j++)
                {
                    UART_PRINT(" ");
                }
            }
        }
        UART_PRINT("%c",str_tmp[i]);
        if((str_tmp[i] == '[')||(str_tmp[i] == '{'))
        {
            ident++;
        }
        pre = &str_tmp[i];
    }
}

void createTemplate(void)
{
    int16_t retVal;

    retVal = Json_createTemplate(&templateHandle, 
                                  templateBuff.fileBuffer,
                                  templateSize);
    if(retVal < 0)
    {
        UART_PRINT("Error: %d  , Couldn't create template \n\r",retVal);
    }
    else
    {
        UART_PRINT("Template object created successfully. \n\n\r");
    }
}

void createObject(void)
{
    int16_t retVal;
    char objSizeBuffer[OBJ_BUFFER_SIZE];
    /* initialize object size buffer */
    memset(objSizeBuffer,'\0',OBJ_BUFFER_SIZE);
    UART_PRINT("Please enter object size in bytes [0 - default size ]\n\r");
    retVal = GetCmd((char *)objSizeBuffer, OBJ_BUFFER_SIZE);
    if(retVal <= 0)
    {
        UART_PRINT("Buffer length exceeded\n\r");
        return;
    }
    UART_PRINT("\n");
    /* convert object size received from uart into integer */
    objSize = atoi(objSizeBuffer);
    retVal = Json_createObject(&jsonObjHandle,templateHandle,objSize);
    if(retVal < 0)
    {
        UART_PRINT("Error: %d  , Couldn't create json object \n\r", retVal);
    }
    else
    {
        UART_PRINT("Json object created successfully. \n\n\r");
    }
}

void parse(void)
{
    int16_t retVal;

    retVal =
        Json_parse(jsonObjHandle,jsonBuffer.fileBuffer,
                   strlen(jsonBuffer.fileBuffer));
    if(retVal < 0)
    {
        UART_PRINT("Error: %d  , Couldn't parse the Json file \n\r", retVal);
    }
    else
    {
        UART_PRINT("Json was parsed successfully \n\n\r");
    }
}

void getValue(void)
{
    char getBuffer[CMD_BUFFER_SIZE];
    char keyBuffer[CMD_BUFFER_SIZE];
    char valueType[SELECT_BUFFER_SIZE];
    uint16_t valueSize = CMD_BUFFER_SIZE;
    int16_t retVal;
    int32_t numValue;
    /* initialize key and set buffers to null terminated chars */
    memset(keyBuffer,'\0',CMD_BUFFER_SIZE);
    memset(getBuffer,'\0',CMD_BUFFER_SIZE);

    UART_PRINT("Please enter a key to the value\n\r");
    retVal = GetCmd((char *)keyBuffer, CMD_BUFFER_SIZE);
    if(retVal <= 0)
    {
        UART_PRINT("Buffer length exceeded\n\r");
        return;
    }
    UART_PRINT("\n");
    UART_PRINT(
        "Please choose value type [0 - Int32, 1 - String / "
        "RAW , 2 - Boolean ]. \n\r");
    retVal = GetCmd((char *)valueType, SELECT_BUFFER_SIZE);
    if(retVal <= 0)
    {
        UART_PRINT("Buffer length exceeded\n\r");
        return;
    }
    UART_PRINT("\n");
    UART_PRINT("Processing the request ... \n\r");
    if(atoi(valueType) == INT32)
    {
        retVal = Json_getValue(jsonObjHandle,keyBuffer,&numValue,&valueSize);
        if(retVal == JSON_RC__VALUE_IS_NULL)
        {
            UART_PRINT("The value is null\n\r");
            return;
        }
        else if(retVal < 0)
        {
            UART_PRINT("Error: %d  , Couldn't get the data \n\r",retVal);
            return;
        }
        UART_PRINT("The value is : %d \n\r",numValue);
    }
    else if(atoi(valueType) == STRING_RAW)
    {
        retVal = Json_getValue(jsonObjHandle,keyBuffer,getBuffer,&valueSize);
        if(retVal == JSON_RC__VALUE_IS_NULL)
        {
            UART_PRINT("The value is null\n\r");
            return;
        }
        else if(retVal < 0)
        {
            UART_PRINT("Error: %d  , Couldn't get the data \n\r",retVal);
            return;
        }
        UART_PRINT("The value is : %s \n\r",getBuffer);
    }
    else if(atoi(valueType) == BOOLEAN)
    {
        retVal = Json_getValue(jsonObjHandle,keyBuffer,&numValue,&valueSize);
        if(retVal == JSON_RC__VALUE_IS_NULL)
        {
            UART_PRINT("The value is null\n\r");
            return;
        }
        else if(retVal < 0)
        {
            UART_PRINT("Error: %d  , Couldn't get the data \n\r",retVal);
            return;
        }
        UART_PRINT("The value is : %s \n\r",
                   ((uint8_t)numValue == 0) ? "false" : "true");
    }
    else
    {
        UART_PRINT("Invalid value type.\n\r");
    }
}

void setValue(void)
{
    char setBuffer[CMD_BUFFER_SIZE];
    char keyBuffer[CMD_BUFFER_SIZE];
    char valueType[SELECT_BUFFER_SIZE];
    uint16_t valueSize = CMD_BUFFER_SIZE;
    int16_t retVal;
    int32_t numValue;
    /* initialize key and set buffers to null terminated chars */
    memset(keyBuffer,'\0',100);
    memset(setBuffer,'\0',100);
    UART_PRINT("Please enter a key to the value\n\r");
    retVal = GetCmd((char *)keyBuffer, CMD_BUFFER_SIZE);
    if(retVal <= 0)
    {
        UART_PRINT("Buffer length exceeded\n\r");
        return;
    }
    UART_PRINT("\n");
    UART_PRINT(
        "Please choose value type "
        "[0 - Int32, 1 - String / RAW , 2 - Boolean ]. \n\r");
    retVal = GetCmd((char *)valueType, SELECT_BUFFER_SIZE);
    if(retVal <= 0)
    {
        UART_PRINT("Buffer length exceeded\n\r");
        return;
    }
    UART_PRINT("\n");
    UART_PRINT("Please enter a value to set \n\r");
    retVal = GetCmd((char *)setBuffer, CMD_BUFFER_SIZE);
    if(retVal <= 0)
    {
        UART_PRINT("Buffer length exceeded\n\r");
        return;
    }
    UART_PRINT("\n");
    UART_PRINT("Processing the request ... \n\r");
    if(atoi(valueType) == INT32)
    {
        numValue = atoi(setBuffer);
        valueSize = sizeof(numValue);
        retVal = Json_setValue(jsonObjHandle, keyBuffer,&numValue,valueSize);
        if(retVal < 0)
        {
            UART_PRINT("Error: %d  , Couldn't set the data \n\r",retVal);
            return;
        }
        UART_PRINT("The value has been set: %d\n\r",numValue);
    }
    else if(atoi(valueType) == STRING_RAW)
    {
        valueSize = strlen(setBuffer);
        retVal = Json_setValue(jsonObjHandle, keyBuffer,setBuffer,valueSize);
        if(retVal < 0)
        {
            UART_PRINT("Error: %d  , Couldn't set the data \n\r",retVal);
            return;
        }
        UART_PRINT("The value has been set: %s\n\r",setBuffer);
    }
    else if(atoi(valueType) == BOOLEAN)
    {
        numValue = atoi(setBuffer);
        valueSize = sizeof(uint16_t);
        /* verify that the value is valid boolean */
        if((numValue != 0) && (numValue != 1))
        {
            UART_PRINT("Wrong boolean value. 0 - false , 1 - true \n\r ");
            return;
        }
        retVal = Json_setValue(jsonObjHandle,keyBuffer,&numValue,valueSize);
        if(retVal < 0)
        {
            UART_PRINT("Error: %d  , Couldn't set the data \n\r",retVal);
            return;
        }
        UART_PRINT("The value has been set: %s\n\r",
                   ((uint8_t)numValue == 0) ? "false" : "true");
    }
    else
    {
        UART_PRINT("Invalid value type.\n\r");
    }
}

void getArrayMemberCount(void)
{
    char keyBuffer[CMD_BUFFER_SIZE];
    int16_t retVal;
    /* Initialize the key buffer to null terminated chars */
    memset(keyBuffer,'\0',CMD_BUFFER_SIZE);
    UART_PRINT("Please enter a key to the array? \n\r");
    retVal = GetCmd((char *)keyBuffer, CMD_BUFFER_SIZE);
    if(retVal <= 0)
    {
        UART_PRINT("Buffer length exceeded\n\r");
        return;
    }
    UART_PRINT("\n");
    UART_PRINT("Processing the request ... \n\r");
    retVal = Json_getArrayMembersCount(jsonObjHandle,keyBuffer);
    if(retVal < 0)
    {
        UART_PRINT("Error: %d  , Couldn't get array member count.  \n\r",
                   retVal);
        return;
    }

    UART_PRINT("Number of members in array %d \r\n",retVal);
}

void build(void)
{
    char        *builtText;
    int16_t retVal;
    uint16_t builtTextSize;
    /* set object size to default size if zero was chosen */
    builtTextSize = (objSize == 0) ? JSON_DEFAULT_SIZE : objSize;
    /* creates buffer for building the json */
    builtText = (char *)malloc(builtTextSize);
    if(builtText)
    {
        retVal = Json_build(jsonObjHandle,builtText,&builtTextSize);
        if(retVal < 0)
        {
            UART_PRINT("Error: %d  , Couldn't build the json.  \n\r", retVal);
            free(builtText);
            return;
        }
        removeUnwantedChars(builtText);
        /* prints json according to json convention */
        validateForPrint(builtText);
        free(builtText);
    }
    else
    {
        UART_PRINT("Couldn't allocate memory \n\r");
    }
}

void destroyTemplate(void)
{
    int16_t retVal;

    retVal = Json_destroyTemplate(templateHandle);
    if(retVal < 0)
    {
       UART_PRINT("Error: %d  , Couldn't destroy the template.  \n\r", retVal);
       return;
    }
    UART_PRINT("Template was destroyed successfully.  \n\r", retVal);
}

void destroyJsonObject(void)
{
    int16_t retVal;

    retVal = Json_destroyObject(jsonObjHandle);
    if(retVal < 0)
    {
        UART_PRINT("Error: %d  , Couldn't destroy the json.  \n\r", retVal);
        return;
    }
    UART_PRINT("Json was destroyed successfully.  \n\r", retVal);
}

//*****************************************************************************
//
//! \brief Display Application Banner
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
int32_t displayBanner(void)
{
    int32_t status = -1;
    uint8_t macAddress[SL_MAC_ADDR_LEN];
    uint16_t macAddressLen = SL_MAC_ADDR_LEN;
    uint16_t configSize = 0;
    uint8_t configOpt = SL_DEVICE_GENERAL_VERSION;
    SlDeviceVersion_t ver = {0};

    configSize = sizeof(SlDeviceVersion_t);
    status = sl_Start(0, 0, 0);

    /* Print device version info. */
    status =
        sl_DeviceGet(SL_DEVICE_GENERAL, &configOpt, &configSize,
                     (uint8_t*)(&ver));

    /* Print device Mac address */
    status = sl_NetCfgGet(SL_NETCFG_MAC_ADDRESS_GET, 0, &macAddressLen,
                          &macAddress[0]);

    UART_PRINT("\n\n\n\r");
    UART_PRINT("\t ==============================================\n\r");
    UART_PRINT("\t    %s Example Ver: %s\n\r",APPLICATION_NAME,
               APPLICATION_VERSION);
    UART_PRINT("\t ==============================================\n\r");
    UART_PRINT("\n\r");
    UART_PRINT("\t CHIP: 0x%x",ver.ChipId);
    UART_PRINT("\n\r");
    UART_PRINT("\t MAC:  %d.%d.%d.%d",ver.FwVersion[0],ver.FwVersion[1],
               ver.FwVersion[2],
               ver.FwVersion[3]);
    UART_PRINT("\n\r");
    UART_PRINT("\t PHY:  %d.%d.%d.%d",ver.PhyVersion[0],ver.PhyVersion[1],
               ver.PhyVersion[2],
               ver.PhyVersion[3]);
    UART_PRINT("\n\r");
    UART_PRINT("\t NWP:  %d.%d.%d.%d",ver.NwpVersion[0],ver.NwpVersion[1],
               ver.NwpVersion[2],
               ver.NwpVersion[3]);
    UART_PRINT("\n\r");
    UART_PRINT("\t ROM:  %d",ver.RomVersion);
    UART_PRINT("\n\r");
    UART_PRINT("\t HOST: %s", SL_DRIVER_VERSION);
    UART_PRINT("\n\r");
    UART_PRINT("\t MAC address: %02x:%02x:%02x:%02x:%02x:%02x", macAddress[0],
               macAddress[1], macAddress[2], macAddress[3], macAddress[4],
               macAddress[5]);
    UART_PRINT("\n\r");
    UART_PRINT("\n\r");
    UART_PRINT("\t ==============================================\n\r");
    UART_PRINT("\n\r");
    UART_PRINT("\n\r");
    status = sl_Stop(SL_STOP_TIMEOUT);

    return(status);
}

//*****************************************************************************
//
//!  \brief this function is the main function which is running and
//!         receiving actions from the user.
//!
//!  \param  none
//!  \return none
//!
//
//*****************************************************************************
void readCmd(void)
{
    char menuInput[SELECT_BUFFER_SIZE];
    int16_t retVal = 0;
    json_action actionSelect;

    while(1)
    {
        UART_PRINT("JSON Menu: \n\n\r");
        UART_PRINT("=======================\n\n\r");
        UART_PRINT("0.Create template object \n\n\r");
        UART_PRINT("1.Create json object \n\n\r");
        UART_PRINT("2.Parse \n\n\r");
        UART_PRINT("3.Get value \n\n\r");
        UART_PRINT("4.Set value \n\n\r");
        UART_PRINT("5.Get array member count \n\n\r");
        UART_PRINT("6.Build the json \n\n\r");
        UART_PRINT("7.Destroy template object \n\n\r");
        UART_PRINT("8.Destroy json object \n\n\r");
        UART_PRINT("Choose number:  \n\r");

        retVal = GetCmd((char *)menuInput, SELECT_BUFFER_SIZE);
        if(retVal <= 0)
        {
            UART_PRINT("\n\r");
            continue;
        }
        UART_PRINT("\n\r");
        if((menuInput[0] < ASCI_0) || (menuInput[0] > ASCI_9))
        {
            UART_PRINT("Invalid action chosen...  \n\r");
            UART_PRINT("\n\r");
            UART_PRINT("Press any key to continue.... \r\n");
            getch();
            continue;
        }
        /* convert menu input received from uart into integer */
        actionSelect = (json_action)atoi(menuInput);
        switch(actionSelect)
        {
        case JSON_CREATE_TEMPLATE:
            createTemplate();
            break;
        case JSON_CREATE_OBJECT:
            createObject();
            break;
        case JSON_PARSE:
            parse();
            break;
        case JSON_GET_VALUE:
            getValue();
            break;
        case JSON_SET_VALUE:
            setValue();
            break;
        case JSON_GET_ARRAY_MEMBER_COUNT:
            getArrayMemberCount();
            break;
        case JSON_BUILD:
            build();
            break;
        case JSON_DESTROY_TEMPLATE:
            destroyTemplate();
            break;
        case JSON_DESTROY_JSON_OBJECT:
            destroyJsonObject();
            break;
        default: UART_PRINT("Invalid action chosen...  \n\r");
            break;
        }
        UART_PRINT("\n\r");
        UART_PRINT("Press any key to continue.... \r\n");
        getch();
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

void mainThread(void *pvParameters)
{
    int32_t status;
    pthread_attr_t pAttrs_spawn;
    struct sched_param priParam;
    int16_t retVal = 0;
    int32_t mode = -1;
    pthread_t spawn_thread = (pthread_t)NULL;

    GPIO_init();
    SPI_init();

    /* Configure the UART */
    InitTerm();

    /* Start the SimpleLink Host */
    pthread_attr_init(&pAttrs_spawn);
    priParam.sched_priority = SPAWN_TASK_PRIORITY;
    status = pthread_attr_setschedparam(&pAttrs_spawn, &priParam);
    status |= pthread_attr_setstacksize(&pAttrs_spawn, TASKSTACKSIZE);

    status = pthread_create(&spawn_thread, &pAttrs_spawn, sl_Task, NULL);

    if(status != 0)
    {
        UART_PRINT("could not create simplelink task\n\r");
        while(1)
        {
            ;
        }
    }
    displayBanner();
    mode = sl_Start(0, 0, 0);
    if(mode < 0)
    {
        sl_Stop(SL_STOP_TIMEOUT);
        UART_PRINT("[Common] CC32xx NWP reset request\r\n");

        /* Reset the MCU in order to test the bundle */
        sl_Start(0, 0, 0);
    }

    UART_PRINT("Loading template from file system....\n\n\r");
    /* Load the template file from the file system */
    templateSize = readFile(&templateBuff,TEMPLATE_FILENAME);
    if(templateSize < 0)
    {
        UART_PRINT(
            "Error loading template file, verify the template file and reset "
            "the platform. \n\r");
        while(1)
        {
            ;
        }
    }
    UART_PRINT("Loading json from file system....\n\n\r");
    /* Load the json file from the file system */
    retVal = readFile(&jsonBuffer,JSON_FILENAME);
    if(retVal < 0)
    {
        UART_PRINT(
            "Error loading Json file, verify the Json file and reset the "
            "platform. \n\r");
        while(1)
        {
            ;
        }
    }
    readCmd();
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
