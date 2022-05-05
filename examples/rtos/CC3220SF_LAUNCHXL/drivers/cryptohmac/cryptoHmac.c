/*
 * Copyright (c) 2015-2021, Texas Instruments Incorporated
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
// Application Name     - Crypto HMAC Application
// Application Overview - The application is a reference to usage of HMAC
//                        SHA MD5 DriverLib functions on CC3220.
//                        Developer/User can refer to this simple application
//                        and re-use the functions in their applications
// Application Details  - docs\examples\CC32xx_SHA-MD5_Demo_Application.pdf
//
//*****************************************************************************
// Standard includes
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* TI-DRIVERS Header files */
#include <ti/drivers/UART2.h>
#include <ti/drivers/crypto/CryptoCC32XX.h>

// Driver Configuration
#include "ti_drivers_config.h"

#define APPLICATION_VERSION  "0.01.00.00"
#define APP_NAME             "Crypto HMAC"

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
UART2_Handle uart2Handle;
uint32_t CryptoHmac_useHmac;
uint32_t CryptoHmac_keyInput[16];

/* Make these static to reduce stack usage */
static CryptoCC32XX_HmacParams params;
static uint8_t                 result[CryptoCC32XX_MAX_DIGEST_SIZE];

char *CryptoHmac_key1 = "p$d0Kotrp$d0Kotrp$d0Kotrp$d0Kotrp$d0Kotrp$d0Kotrp$d0Kotrp$d0Kotr";
char *CryptoHmac_key2 = "abababababababababababababababababababababababababababababababab";
char *CryptoHmac_key3 = "cdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcd";

extern int vsnprintf (char * s, size_t n, const char * format, va_list arg );

//*****************************************************************************
//
// Sample key for generating HMAC.  This array contains 64 bytes (512 bits)
// of randomly generated data.
//
//*****************************************************************************
uint32_t CryptoHmac_keySample[16] =
{
    0x8a5f1b22, 0xcb935d29, 0xcc1ac092, 0x5dad8c9e, 0x6a83b39f, 0x8607dc60,
    0xda0ba4d2, 0xf49b0fa2, 0xaf35d524, 0xffa8001d, 0xbcc931e8, 0x4a2c99ef,
    0x7fa297ab, 0xab943bae, 0x07c61cc4, 0x47c8627d
};


uint32_t CryptoHmac_MD5Result[4] =
{
    0x6a1403a1, 0xd408e103, 0x8d232598, 0xfee34b16
};

uint32_t CryptoHmac_sha224Result[7] =
{
    0x78da2914, 0x718803aa, 0x4e6ea25e, 0x0b38d856, 0x86982ba9, 0x9f7f1840,
    0xc69fdf04
};


//*****************************************************************************
//
// Source data for producing hashes. This array contains 1024 bytes of
// randomly generated data.
//
//*****************************************************************************
uint32_t CryptoHmac_randomData[] =
{
    0x7c68c9ec, 0x72af34b3, 0xca0edf2e, 0x60f4860d, 0x50cfa1dc, 0x9a2b538c,
    0x98450274, 0x60f5c272, 0x7317d78e, 0x2361ca0e, 0xfa4a52b1, 0x658f729b,
    0x5267f9d9, 0x1bccd3ca, 0x2f0bb993, 0x1be38a3d, 0x00bd2d2a, 0x97405e63,
    0xe3efd585, 0xb02d1588, 0xe55d71c8, 0x43a27ecf, 0x5fd275db, 0x73ad8f06,
    0x88f55495, 0x68922493, 0x03ea6039, 0xe40a678a, 0x052847ce, 0xf7a28b46,
    0x3b60c73e, 0x3f08dbd4, 0x2a66b3a6, 0xcf398b15, 0xacbfc6d8, 0x6c15a285,
    0x997d0e01, 0xbfd12e26, 0xa26bc485, 0xb8946d2f, 0x0f84742b, 0x5be82a2f,
    0x8d2e2cc7, 0xc7a1dea6, 0xcfaa6cb6, 0xe706434c, 0x079810d0, 0x5eca9400,
    0x7b92dd1c, 0x1ec552e8, 0xa74ae9c3, 0x2e859af5, 0x8d9d1a35, 0x07ff6040,
    0xc0b19670, 0x2e348aa8, 0xed89efea, 0x3262e8f0, 0x45093372, 0x8f8bae5c,
    0x505d64bb, 0x9a172079, 0x327b5f67, 0xa3a12ba8, 0x7f573054, 0xd3d5f778,
    0xbc1bd124, 0x0d0ad1c6, 0x24ac345b, 0x4f50084a, 0x302a5985, 0xfa3e8b86,
    0x2022c497, 0xd297e4b4, 0xd1c53c01, 0x6e541890, 0x93ec53c6, 0x24c5ce2b,
    0xdd38e334, 0x078a0334, 0x2a470b22, 0xadad86b4, 0x7b2041db, 0xc74ce30b,
    0x8e6dc4ca, 0x273b85c8, 0x339d2334, 0x86d1dacc, 0xd588e165, 0xcee15221,
    0x8e11a0a1, 0x9315a6c2, 0x53e9fa9a, 0xf4bb6d7a, 0x421cb9ec, 0x1f370567,
    0xfd8c880f, 0xd20797cd, 0x90aee852, 0x2a2f966a, 0x126ffcdd, 0x44a2f09a,
    0xbac72ac4, 0x77d588c5, 0x77b53c09, 0x275b8828, 0x778a2be5, 0x40167d1e,
    0x550c0c94, 0x14e070e7, 0x597ff5a3, 0xbef40dc2, 0x8306d119, 0x6a8d29a6,
    0xb5d8e740, 0x52a37fe2, 0xdf34ad27, 0x1bb885fd, 0x6dd352f8, 0x8b0d62b5,
    0x5c82d35f, 0x0eb84312, 0xd2c7823a, 0x494f7a00, 0x30680642, 0x01fa9460,
    0xdc63956f, 0x70fa0b53, 0xd0865e78, 0x3a52e983, 0x318a881c, 0x4d113947,
    0xc0f302df, 0x6b2027fb, 0x1078566d, 0xd71d39a6, 0xcdd00388, 0x119e3c4e,
    0x4ddbf1c6, 0xb371eb0f, 0xdcbd768f, 0x2fc5b5e8, 0xc67a2efe, 0x29d18630,
    0xb389d68f, 0x26a71f13, 0x43583b57, 0x56f5eae8, 0x2edc7cd5, 0xcc93d41e,
    0xab691f87, 0x51ab1d8e, 0x37c2966e, 0x19ccd9ec, 0xb782124a, 0xdefc2804,
    0xea3bde3c, 0x46d81e08, 0xf828d58e, 0x757a39d3, 0xc92f1b5f, 0x56a2b368,
    0x1bbbb9b9, 0x46086ac7, 0x8a343144, 0x1675157a, 0x28ac0cf1, 0xb8695178,
    0x25fc4cec, 0x3f23a44e, 0x0a697977, 0x525794ad, 0xf920e15c, 0x49a0a7a7,
    0x1f54cafb, 0x7357b64c, 0x6d3a19c6, 0x5efb526d, 0x3d37f6e2, 0xd4f5835b,
    0x6ff454ee, 0x4f2a311c, 0x83cc4a40, 0x003036e9, 0xd481bf33, 0x38868b3c,
    0x63ee4445, 0x58426a29, 0xa022ae59, 0x07deb8ce, 0xfe3e673d, 0x176aa368,
    0xf2b18641, 0xbadeccd8, 0xea7a72b4, 0x72ccf0a0, 0xcdee3b08, 0x1689c54f,
    0xd577085a, 0xd9d79bd1, 0x089fa69a, 0x03fdaf65, 0x855e5697, 0x5788c00c,
    0x1139e03e, 0x48f4305f, 0x2d8ad2fd, 0x71ab04b5, 0xf5c7871c, 0x76801f21,
    0x329a590e, 0xe8e982a2, 0xdb67783e, 0x26ebf88b, 0x13ac5de7, 0x69b07707,
    0x2bc54e92, 0xc2556f94, 0x6d21bc3b, 0x3a230d0c, 0x4e02eeec, 0x53605beb,
    0x3a31e796, 0x6e186887, 0x8f93356e, 0xfa2342e4, 0xfbf2f519, 0x7ae95455,
    0xad6e9d94, 0xd942c7ab, 0x624f7aed, 0xd4158624, 0x82a0c0a9, 0x6d79b262,
    0xa7b9c84d, 0x2015bfeb, 0x462c7267, 0x44a17743, 0x7d207f71, 0xc2ab7566,
    0xaa833e65, 0x0a6c385e, 0x3b2d85f1, 0x8a4821a8, 0x62bf5742, 0xf55cf0e1,
    0xfc07d0d9, 0x54910235, 0xe8ae66c9, 0x9beb7306, 0xe5671f9e, 0x3332ad03,
    0xdb2343b6, 0x124332ac, 0xf595c7fb, 0xda2c72b0
};

//*****************************************************************************
//
// Test Vector
//
//*****************************************************************************
typedef struct _CryptoHmac_TestVector_t_
{
    uint8_t *hmacKey;
    uint32_t dataLength;
    uint8_t * puiPlainText;
    uint8_t *puiExpectedHash;
    uint32_t hashLength;
} CryptoHmac_TestVector_t;

CryptoHmac_TestVector_t CryptoHmac_testVectors =
{
    (uint8_t *)CryptoHmac_keySample,
    1024,
    (uint8_t *)CryptoHmac_randomData,
    (uint8_t *)CryptoHmac_MD5Result,
    16
};


//*****************************************************************************
// Function Prototypes
//*****************************************************************************

//*****************************************************************************
//
//! Outputs a character string to the console
//!
//! This function
//!        1. prints the input string character by character on to the console.
//!
//! \param[in]  str - is the pointer to the string to be printed
//!
//! \return none
//
//*****************************************************************************
void consoleString(const char *str)
{
    UART2_write(uart2Handle, str, strlen(str), NULL);
}

//*****************************************************************************
//
//! Clear the console window
//!
//! This function
//!        1. clears the console window.
//!
//! \param  none
//!
//! \return none
//
//*****************************************************************************
void consoleClear(void)
{
    consoleString("\33[2J\r");
}

//*****************************************************************************
//
//! prints the formatted string on to the console
//!
//! \param[in]  format  - is a pointer to the character string specifying the
//!                       format in the following arguments need to be
//!                       interpreted.
//! \param[in]  [variable number of] arguments according to the format in the
//!            first parameters
//!
//! \return count of characters printed
//
//*****************************************************************************
int consolePrintf(const char *pFormat, ...)
{
    int         status = 0;
    char        *pBuff;
    char        *pTemp;
    int         size = 256;
    va_list     list;


    pBuff = (char*)malloc(size);
    if(pBuff == NULL)
    {
        return -1;
    }
    while(1)
    {
        va_start(list,pFormat);
        status = vsnprintf(pBuff, size, pFormat, list);
        va_end(list);
        if((status > -1) && (status < size))
        {
            break;
        }
        else
        {
            size *= 2;
            if((pTemp = realloc(pBuff, size)) == NULL)
            {
                consoleString("Could not reallocate memory\n\r");
                status = -1;
                break;
            }
            else
            {
                pBuff = pTemp;
            }
        }
    }
    consoleString(pBuff);
    free(pBuff);

    return status;
}

//*****************************************************************************
//
//! Get the Command string from UART
//!
//! \param[in]  pucBuffer   - is the command store to which command will be
//!                           populated
//! \param[in]  ucBufLen    - is the length of buffer store available
//!
//! \return Length of the bytes received. -1 if buffer length exceeded.
//!
//*****************************************************************************
int consoleGetCmd(char *pBuffer, unsigned int bufLen)
{
    char    ch;
    int     len = 0;
    char    backspace;


    UART2_read(uart2Handle, &ch, 1, NULL);

    len = 0;

    //
    // Checking the end of Command
    //
    while(1)
    {
        //
        // Handling overflow of buffer
        //
        if(len >= bufLen)
        {
            return -1;
        }

        //
        // Copying Data from UART into a buffer
        //
        if((ch == '\r') || (ch =='\n'))
        {
            UART2_write(uart2Handle, &ch, 1, NULL);
            break;
        }
        else if(ch == '\b')
        {
            //
            // Deleting last character when you hit backspace
            //
            UART2_write(uart2Handle, &ch, 1, NULL);
            backspace = ' ';
            UART2_write(uart2Handle, &backspace, 1, NULL);
            if(len)
            {
                UART2_write(uart2Handle, &ch, 1, NULL);
                len--;
            }
            else
            {
                backspace = '\a';
                UART2_write(uart2Handle, &backspace, 1, NULL);
            }
        }
        else
        {
            //
            // Echo the received character
            //
            UART2_write(uart2Handle, &ch, 1, NULL);

            *(pBuffer + len) = ch;
            len++;
        }

        UART2_read(uart2Handle, &ch, 1, NULL);
    }

    *(pBuffer + len) = '\0';

    return len;
}

//*****************************************************************************
//
//! Application startup display
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
void displayBanner(char * AppName)
{

    consoleString("\n\n\n\r");
    consoleString("\t\t *************************************************\n\r");
    consolePrintf("\t\t      CC32XX %s Application       \n\r", AppName);
    consoleString("\t\t *************************************************\n\r");
    consoleString("\n\n\n\r");
}

//*****************************************************************************
//
//! Display main menu
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
void usageDisplay(void)
{
    consoleString("Crypto HMAC use case Options : \n\r");
    consoleString("    1) for HMAC MD5   \n\r");
    consoleString("    2) for HMAC SHA1  \n\r");
    consoleString("    3) for HMAC SHA224\n\r");
    consoleString("    4) for HMAC SHA256\n\r");
    consoleString("    5) for MD5   \n\r");
    consoleString("    6) for SHA1  \n\r");
    consoleString("    7) for SHA224\n\r");
    consoleString("    8) for SHA256\n\r");
    consoleString("\n\r");
}

//*****************************************************************************
//
//! Get Key - Gets the Key into the Buffer from User
//!
//! \param  pKeyBuff is the Key buffer to which Key will be populated
//!
//! \return Success or Failure
//!
//*****************************************************************************
bool getKey(char *pKeyBuff)
{
    char ch;
    int32_t len;

    if(CryptoHmac_useHmac)
    {
        consoleString("\n\rDo you want to use Pre-Defined Key or not (y/n) ? \n\r");
        //
        // Get the option
        //
        UART2_read(uart2Handle, &ch, 1, NULL);
        //
        // Echo the received character
        //
        UART2_write(uart2Handle, &ch, 1, NULL);

        consoleString("\n\r");
        if(ch=='y' || ch=='Y' )
        {
            //
            // Fetch the key
            //
            consolePrintf("\n\r Press 1 for Key - %s\n\r Press 2 for Key - %s \n\r"
                        " Press 3 for Key - %s\n\r",CryptoHmac_key1,CryptoHmac_key2,CryptoHmac_key3);
            UART2_read(uart2Handle, &ch, 1, NULL);
            //
            // Echo the received character
            //
            UART2_write(uart2Handle, &ch, 1, NULL);

            consoleString("\n\r");
            switch (ch)
            {
                case '1':
                    memcpy(&CryptoHmac_keyInput,CryptoHmac_key1,64);
                break;
                case '2':
                    memcpy(&CryptoHmac_keyInput,CryptoHmac_key2,64);
                break;
                case '3':
                    memcpy(&CryptoHmac_keyInput,CryptoHmac_key3,64);
                break;
                default:
                    consoleString("\n\r Wrong Key \n\r");
                    return false;
            }
        }
        else if(ch=='n' || ch=='N')
        {
            //
            // Ask for the Key
            //
            memset(pKeyBuff, 0x00, 64);
            consoleString("Enter the Key \n\r");
            len = consoleGetCmd(pKeyBuff,(64+1));
            if(len<=0)
            {
                consoleString("\n\r Enter Valid Key of length 64\n\r");
                return false;
            }
        }
        else
        {
            consoleString("\n\r Invalid Input \n\r");
            return false;
        }
    }
    else
    {
        pKeyBuff = NULL;
    }

    return true;
}

//*****************************************************************************
//
//! Get Msg - Gets the Message into the Buffer from User
//!
//! \param  pMsgBuff is the Msg buffer to which Plain Text will be populated
//! \param  pDataLength is the Length of the Data entered
//!
//! \return Pointer to Plain Text
//!
//*****************************************************************************
uint8_t * getMsg(uint8_t *pMsgBuff,uint32_t *pDataLength)
{
    int32_t len;

    consoleString("\n\r Enter the Message \n\r");

    //
    // Get Message
    //
    len = consoleGetCmd((char*)pMsgBuff, 520);
    if(len <= 0)
    {
        consolePrintf("Message length exceeded max limit of %d char\n", 520);
        return NULL;
    }
    *pDataLength = len;

    return pMsgBuff;
}

//*****************************************************************************
//
//! parseCommand - Populates the parameters from User
//!
//! \param  pCommand is the Command buffer to which Command will be populated
//! \param  pConfig is the Configuration Value
//! \param  pHashLength is the Hash Length
//!
//! \return Success or Failure
//!
//*****************************************************************************
bool parseCommand( char *pCommand,CryptoCC32XX_HmacMethod *pConfig,uint32_t *pHashLength)
{
    char *pInpString;

    pInpString = strtok(pCommand, " ");

    //
    // Get which Algorithm is using
    //
    if((pInpString != NULL) && (!strcmp(pInpString,"1")))
    {
        *pConfig=CryptoCC32XX_HMAC_MD5;
        *pHashLength=16;
        CryptoHmac_useHmac=1;
    }
    else if((pInpString != NULL) && (!strcmp(pInpString,"2")))
    {
        *pConfig=CryptoCC32XX_HMAC_SHA1;
        *pHashLength=20;
        CryptoHmac_useHmac=1;
    }
    else if((pInpString != NULL) && (!strcmp(pInpString,"3")))
    {
        *pConfig=CryptoCC32XX_HMAC_SHA224;
        *pHashLength=28;
        CryptoHmac_useHmac=1;
    }
    else if((pInpString != NULL) && (!strcmp(pInpString,"4")))
    {
        *pConfig=CryptoCC32XX_HMAC_SHA256;
        *pHashLength=32;
        CryptoHmac_useHmac=1;
    }
    else if((pInpString != NULL) && (!strcmp(pInpString,"5")))
    {
        *pConfig=CryptoCC32XX_HMAC_MD5;
        *pHashLength=16;
        CryptoHmac_useHmac=0;
    }
    else if((pInpString != NULL) && (!strcmp(pInpString,"6")))
    {
        *pConfig=CryptoCC32XX_HMAC_SHA1;
        *pHashLength=20;
        CryptoHmac_useHmac=0;
    }
    else if((pInpString != NULL) && (!strcmp(pInpString,"7")))
    {
        *pConfig=CryptoCC32XX_HMAC_SHA224;
        *pHashLength=28;
        CryptoHmac_useHmac=0;
    }
    else if((pInpString != NULL) && (!strcmp(pInpString,"8")))
    {
        *pConfig=CryptoCC32XX_HMAC_SHA256;
        *pHashLength=32;
        CryptoHmac_useHmac=0;
    }

    else
    {
        consolePrintf("\n\r Invalid Algorithm\n\r");
        return false;
    }
    return true;
}

//*****************************************************************************
//
//! readFromUser - Populates the parameters from User
//!
//! \param  pConfig Configuration Value
//! \param  pHashLength is the Length of Hash Value
//! \param  pKey is the Key Used
//! \param  pDataLength is the Length of Data
//!
//! \return pointer to plain text
//!
//*****************************************************************************
uint8_t *readFromUser(CryptoCC32XX_HmacMethod *pConfig,uint32_t *pHashLength,
                        uint8_t **pKey,uint32_t *pDataLength)
{
    char        commandBuffer[2];
    uint8_t     *pMsgBuff;
    uint8_t     *pKeyBuff;
    uint8_t     *pData;
    int32_t     len;

    //
    // POinting KeyBuffer into appropriate Keys.
    //
    pKeyBuff=(uint8_t *)&CryptoHmac_keyInput[0];
    pMsgBuff=(uint8_t *)&CryptoHmac_randomData[0];

    //
    // Usage Display
    //
    usageDisplay();

    //
    // Get the Command
    //
    consoleString("Please enter your Crypto use case selection:  ");
    len = consoleGetCmd(commandBuffer,sizeof(commandBuffer));
    if(len < 0)
    {
        consolePrintf("\n\rCommand length exceed max limit of %d char\n\r", sizeof(commandBuffer));
        return NULL;
    }

    if(parseCommand(commandBuffer,pConfig,pHashLength))
    {
        if(getKey((char*)pKeyBuff))
        {
            if(!CryptoHmac_useHmac)
            {
                pKeyBuff = NULL;
            }
            pData=getMsg(pMsgBuff,pDataLength);
        }
        else
        {
            consoleString("\n\r Invalid Key \n\r");
            return NULL;
        }
    }
    else
    {
        consoleString("\n\r Wrong Input \n\r");
        return NULL;
    }
    *pKey = pKeyBuff;
    return pData;
}

//*****************************************************************************
//
//! main - populate the parameters from predefines Test Vector or User
//!
//! \param None
//!
//! \return None
//
//*****************************************************************************
void mainThread(void *arg0)
{
    CryptoCC32XX_Handle     cryptoHandle;
    CryptoCC32XX_HmacMethod config;
    uint32_t                hashLength;
    uint32_t                dataLength;
    uint8_t                 *pKey1;
    uint8_t                 *pData;
    int32_t                 count;
    UART2_Params            uart2Params;

    //
    // Initialize Crypto module
    CryptoCC32XX_init();

    //
    // Initialize Uart parameters
    UART2_Params_init(&uart2Params);
    uart2Params.readReturnMode = UART2_ReadReturnMode_FULL;

    //
    // Open Uart2 module
    uart2Handle = UART2_open(CONFIG_UART2_0, &uart2Params);

    if (uart2Handle == NULL) {
        /* UART2_open() failed */
        while (1);
    }


    //
    // Display Banner
    //
    consoleClear();
    displayBanner(APP_NAME);

    CryptoCC32XX_HmacParams_init(&params);

    //
    // Open Crypto HMAC handle
    //
    cryptoHandle = CryptoCC32XX_open(CONFIG_CRYPTO_0, CryptoCC32XX_HMAC);
    if (cryptoHandle == NULL)
    {
        return;
    }

    while(1)
    {
        //
        // Read values either from User or from Vector based on macro USER_INPUT
        // defined or not
        //

        //
        // Read the values from the user over uart and Populate the variables
        //
        pData=readFromUser(&config,&hashLength,&pKey1,&dataLength);
        if(pData==NULL)
        {
            continue;
        }

        //
        // Generate Hash Value
        //
        consoleString("\n\r Hashing in Progress... \n\r");
        params.pKey = (uint8_t *)pKey1;
        params.moreData = 0;

        CryptoCC32XX_sign(cryptoHandle, config , pData, dataLength, (uint8_t *)&result, &params);
        consoleString("\n\r Hash Value is generated\n\r");

        //
        // Display Hash Value Generated
        //
        consoleString("\n\r The Hash Value in Hex is: 0x");
        for(count=0;count<hashLength;count++)
        {
          consolePrintf("%02x ",*(result+count));
        }
        consoleString("\n\r");

        //
        // Verify Hash Value Generated
        //
        consoleString("\n\r Verify Hash Value....\n\r");
        if (CryptoCC32XX_verify(cryptoHandle, config , pData, dataLength, (uint8_t *)&result, &params) == CryptoCC32XX_STATUS_SUCCESS)
        {
            consoleString("\n\r Hashing verified successfully.\n\r");
        }
        else
        {
            consoleString("\n\r Error in Hashing computation.\n\r");
        }
        consoleString("\n\r");
    }
}


//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
