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

 /*
 *  Terminal
 */

// Standard includes
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

#include "uart_term.h"

extern int vsnprintf(char * s,
                     size_t n,
                     const char * format,
                     va_list arg);

//*****************************************************************************
//                          LOCAL DEFINES
//*****************************************************************************
#define IS_SPACE(x)             (x == 32 ? 1 : 0)
#define true                    1
#define false                   0

//*****************************************************************************
//                          LOCAL FUNCTIONS
//*****************************************************************************
uint8_t *retCommand(int8_t increment);
void pushCommand(char *command);

//*****************************************************************************
//                 GLOBAL VARIABLES
//*****************************************************************************
static UART_Handle uartHandle;
static const char uSAVE_CURSOR_STR[3] =     {0x1B,0x37,0};
static const char uRESTORE_CURSOR_STR[3] =  {0x1B,0x38,0};
static const char uERASE_LINE[5] =          {0x1B,0x5B,0x30,0x4B,0};

typedef struct commandHistory_t
{
    /* 2-D Array of past commands*/
    uint8_t *uHistory[CMD_HISTORY_LEN];
    /*Index of Max + 1*/
    int8_t iTop;
    /*Index that keeps track of where user is viewing the command history*/
    int8_t iCurrent;
    /*Index of one plus the last command added to command history*/
    int8_t iNext;

}commandHistory;

commandHistory app_CH;

uint8_t gSaveCursorPosition = true;
uint8_t gStackFull          = false;

//*****************************************************************************
//
//! Initialization
//!
//! This function
//!        1. Configures the UART to be used.
//!        2. Initializes command history.
//!
//! \param  none
//!
//! \return none
//
//*****************************************************************************
UART_Handle InitTerm(void)
{
    int8_t i;
    UART_Params uartParams;

    UART_init();
    UART_Params_init(&uartParams);

    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = 115200;

    uartHandle = UART_open(CONFIG_UART_0, &uartParams);
    /* remove uart receive from LPDS dependency */
    UART_control(uartHandle, UART_CMD_RXDISABLE, NULL);

    /* initialize command history */
    app_CH.iCurrent = 0;
    app_CH.iTop = 0;
    app_CH.iNext = 0;


    for(i = 0; i < app_CH.iTop; i++)
    {
        app_CH.uHistory[i] = NULL;
    }

    return(uartHandle);
}

//*****************************************************************************
//
//! prints the formatted string on to the console
//!
//! \param[in]  format  - is a pointer to the character string specifying the
//!                       format in the following arguments need to be
//!                       interpreted.
//! \param[in]  [variable number of] arguments according to the format in the
//!             first parameters
//!
//! \return count of characters printed
//
//*****************************************************************************
int Report(const char *pcFormat,
           ...)
{
    int iRet = 0;
    char        *pcBuff;
    char        *pcTemp;
    int iSize = 256;
    va_list list;

    pcBuff = (char*)malloc(iSize);
    if(pcBuff == NULL)
    {
        return(-1);
    }
    while(1)
    {
        va_start(list,pcFormat);
        iRet = vsnprintf(pcBuff, iSize, pcFormat, list);
        va_end(list);
        if((iRet > -1) && (iRet < iSize))
        {
            break;
        }
        else
        {
            iSize *= 2;
            if((pcTemp = realloc(pcBuff, iSize)) == NULL)
            {
                Message("Could not reallocate memory\n\r");
                iRet = -1;
                break;
            }
            else
            {
                pcBuff = pcTemp;
            }
        }
    }
    Message(pcBuff);
    free(pcBuff);

    return(iRet);
}

//*****************************************************************************
//
//! Trim the spaces from left and right end of given string
//!
//! \param  pcInput - string on which trimming happens
//!
//! \return length of trimmed string
//
//*****************************************************************************
int TrimSpace(char * pcInput)
{
    size_t size;
    char        *endStr;
    char        *strData = pcInput;
    char index = 0;

    size = strlen(strData);

    if(!size)
    {
        return(0);
    }

    endStr = strData + size - 1;
    while((endStr >= strData) && (IS_SPACE(*endStr)))
    {
        endStr--;
    }
    *(endStr + 1) = '\0';

    while(*strData && IS_SPACE(*strData))
    {
        strData++;
        index++;
    }
    memmove(pcInput, strData, strlen(strData) + 1);

    return(strlen(pcInput));
}

//*****************************************************************************
//
//! Get the command string, and allow for it to be edited via UART
//!
//! \param[in]  pcBuffer    - Null terminated string, treated as already
//! 			containing useful data
//! \param[in]  uiBufLen    - is the length of buffer store available
//!
//! \return Length of the bytes received. -1 if buffer length exceeded.
//!
//*****************************************************************************
int GetCmd(char *pcBuffer,
           unsigned int uiBufLen)
{
    uint8_t uChar;
    uint8_t uCh;
    int8_t iLen = 0;
    /* If the command prompt is clear, save the end of command prompt for use later */
    if(gSaveCursorPosition != false)
    {
        UART_writePolling(uartHandle, uSAVE_CURSOR_STR, strlen((char *) uSAVE_CURSOR_STR));
    }
    UART_readPolling(uartHandle, &uChar, 1);
    /* Checking the end of Command */
    while(1)
    {
        /* Handling overflow of buffer */
        if(iLen >= uiBufLen)
        {
            return(-1);
        }
        /* Copying Data from UART into a buffer
         * Checking for use of return/enter key */
        if((uChar == '\r') || (uChar == '\n'))
        {
            UART_writePolling(uartHandle, &uChar, 1);
            gSaveCursorPosition = true;
            break;
        }
        /* Catches escape sequences for the up and down arrow keys */
        else if(uChar == 0x1B)
        {
            UART_readPolling(uartHandle, &uChar, 1);
            /* included with up and down arrows */
            if(uChar == '[')
            {
                UART_readPolling(uartHandle, &uChar, 1);
                if(uChar == 'A') /* up arrow*/
                {
                    uint8_t *uReturnedString = retCommand(-1);
                    uint8_t size = 0;
                    /* retCommand returns a Null pointer if there is nothing
                     * in the command history at the selected point,
                     * this makes sure nothing is read into the CmdBuffer */
                    if(uReturnedString != NULL)
                    {
                        size = strlen((char *)uReturnedString);
                        memcpy(pcBuffer, uReturnedString , strlen((char *)uReturnedString) + 1);
                    }
                    else
                    {
                        pcBuffer[0] = 0;
                    }
                    iLen = size;
                    ReplaceLine(pcBuffer);
                }
                else if(uChar == 'B') /* Down arrow */
                {
                    uint8_t *uReturnedString = retCommand(1);
                    uint8_t size = 0;
                    /* retCommand returns a Null pointer if there is nothing
                     * in the command history at the selected point,
                     * this makes sure nothing is read into the CmdBuffer */
                    if(uReturnedString != NULL)
                    {
                        size = strlen((char *)uReturnedString);
                        memcpy(pcBuffer, uReturnedString , strlen((char *)uReturnedString) + 1);
                    }
                    else
                    {
                        pcBuffer[0] = 0;
                    }
                    iLen = size;
                    ReplaceLine(pcBuffer);
                }
            }
        }
        /* This catches the backspace or delete chars and deletes the last character,
         * 0x7F is delete which can be sent by serial terminal instead of backspace */
        else if((uChar == '\b') || (uChar == 0x7F))
        {
            if(iLen != 0)
            {
                UART_writePolling(uartHandle, &uChar, 1);
                uCh = ' ';
                UART_writePolling(uartHandle, &uCh, 1);
                UART_writePolling(uartHandle, &uChar, 1);
                pcBuffer[iLen] = 0;
                iLen--;
            }
            else
            {
                uCh = '\a';
                UART_writePolling(uartHandle, &uCh, 1);
            }
        }
        else
        {
            /* Echo the received character */
            UART_writePolling(uartHandle, &uChar, 1);

            /* Add the received character to the command string */
            *(pcBuffer + iLen) = uChar;
            iLen++;
        }
        UART_readPolling(uartHandle, &uChar, 1);
    }
    /* Null termination */
    *(pcBuffer + iLen) = '\0';
    if(iLen > 0)
    {
        pushCommand(pcBuffer);
    }
    return(iLen);
}

/*Local functions to implement command history*/
//*****************************************************************************
/*!
    \brief          Return command from command history

    This function takes either 1 or -1 as its argument, and returns the command in
    the command history data structure either before or after the current location.
    The current location is kept track of seperately from where commands are added
    to the data structure.

    \param          Increment, must be 1 or -1

    \return         Address to the beginning of the command string, will be NULL if
                        there is not a command in the data structure


*/
//*****************************************************************************
uint8_t *retCommand(int8_t increment)
{
    app_CH.iCurrent += increment;

    /* wrap around if index reaches a bound */
    if(app_CH.iCurrent < 0)
    {
        app_CH.iCurrent = app_CH.iTop - 1;
    }
    if (abs(app_CH.iCurrent) >= app_CH.iTop)
    {
        app_CH.iCurrent = 0;
    }
    uint8_t *uReturnVal = app_CH.uHistory[app_CH.iCurrent];
    return(uReturnVal);
}

//*****************************************************************************
/*!
    \brief          Write command to command history.

    This function will record the command passed to it in the command history
    It will overwrite values when the structure is filled up.

    \param          Command, pointer to a string.

    \return         N/A

*/
//*****************************************************************************

void pushCommand(char *command)
{
    /* Determine amount of data to allocate */
    size_t size = strlen(command) + 1;
    /* check if command already exist in history array */
    int8_t i;
    for(i = 0; i < CMD_HISTORY_LEN; i++)
      {
          if (!strcmp((char *)app_CH.uHistory[i], command))
          {
              return;
          }
      }
    /* wrap around if reaches the end of the command history structure */
    if(app_CH.iNext >= CMD_HISTORY_LEN)
    {
        app_CH.iNext = 0;
        gStackFull  = true;
    }
    /* entry will be null if nothing is stored in it */
    if(app_CH.uHistory[app_CH.iNext] != NULL)
    {
        app_CH.uHistory[app_CH.iNext] = realloc(app_CH.uHistory[app_CH.iNext], size);
        /* resize to new string length */
    }
    else
    {
        app_CH.uHistory[app_CH.iNext] = malloc(size);
    }

    if(app_CH.uHistory[app_CH.iNext] == NULL)
    {
        /* Failure to allocate memory */
    }
    else
    {   /* initialize newly allocated memory */
        memset(app_CH.uHistory[app_CH.iNext], 0, size);
        memcpy(app_CH.uHistory[app_CH.iNext], command, size);
    }
        app_CH.iNext++;
        if(gStackFull != true)
        {
            app_CH.iTop++;
        }
        app_CH.iCurrent = app_CH.iNext;
}
//*****************************************************************************
//
//! Clears current line of terminal and outputs <str>, while preserving the
//!       command prompt
//!
//! This function
//!        1. Clears current line of terminal and outputs <str>
//!
//! \param[in]  str - is the pointer to the string to be printed
//!
//! \return none
//!
//
//*****************************************************************************

void ReplaceLine(const char *str)
{
    /* Erases entered data, preserves command prompt when using command history */
    UART_writePolling(uartHandle, uRESTORE_CURSOR_STR, strlen(uRESTORE_CURSOR_STR));
    UART_writePolling(uartHandle, uERASE_LINE, strlen(uERASE_LINE));
    UART_writePolling(uartHandle, uSAVE_CURSOR_STR, strlen(uSAVE_CURSOR_STR));
    gSaveCursorPosition = false;
    UART_writePolling(uartHandle, str, strlen(str));
}
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
//!
//! \note If UART_NONPOLLING defined in than Message or UART write should be
//!       called in task/thread context only.
//
//*****************************************************************************
void Message(const char *str)
{
#ifdef UART_NONPOLLING
    UART_write(uartHandle, str, strlen(str));
#else
    UART_writePolling(uartHandle, str, strlen(str));
#endif
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
void ClearTerm()
{
    Message("\33[2J\r");
}

//*****************************************************************************
//
//! Read a character from the console
//!
//! \param none
//!
//! \return Character
//
//*****************************************************************************
char getch(void)
{
    char ch;

    UART_readPolling(uartHandle, &ch, 1);
    return(ch);
}

//*****************************************************************************
//
//! Outputs a character to the console
//!
//! \param[in]  char    - A character to be printed
//!
//! \return none
//
//*****************************************************************************
void putch(char ch)
{
    UART_writePolling(uartHandle, &ch, 1);
}
