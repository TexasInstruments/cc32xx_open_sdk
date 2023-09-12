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
 *  ======== NVSCC32XX.c ========
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h> /* for string support */
#include <stdlib.h>
#include <stdio.h> /* for snprintf() */

#include <xdc/runtime/Log.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>

#include <ti/drivers/NVS.h>
#include <ti/drivers/nvs/NVSCC32XX.h>

#include <simplelink/include/simplelink.h>

#define NVSCC32XX_MAXFILESIZE 0x1000

#define PATHFORMAT "ti_drivers_nvs_%d"

extern const NVS_Config NVS_config[];
extern const uint8_t NVS_count;

/* NVSCC26XX functions */
void NVSCC32XX_close(NVS_Handle handle);
int NVSCC32XX_control(NVS_Handle handle, unsigned int cmd, uintptr_t arg);
void NVSCC32XX_exit(NVS_Handle handle);
int NVSCC32XX_getAttrs(NVS_Handle handle, NVS_Attrs *attrs);
void NVSCC32XX_init(NVS_Handle handle);
NVS_Handle NVSCC32XX_open(NVS_Handle handle, NVS_Params *params);
int NVSCC32XX_read(NVS_Handle handle, size_t offset, void *buffer, size_t bufferSize);
int NVSCC32XX_write(NVS_Handle handle, size_t offset, void *buffer, size_t bufferSize, unsigned int flags);

/* NVS function table for NVSCC26XX implementation */
const NVS_FxnTable NVSCC32XX_fxnTable = {NVSCC32XX_close,
                                         NVSCC32XX_control,
                                         NVSCC32XX_exit,
                                         NVSCC32XX_getAttrs,
                                         NVSCC32XX_init,
                                         NVSCC32XX_open,
                                         NVSCC32XX_read,
                                         NVSCC32XX_write};

/*
 *  Semaphore to synchronize access to flash block.
 */
static Semaphore_Struct writeSem;
static bool isInitialized = false;

/*
 *  ======== NVSCC32XX_close ========
 */
void NVSCC32XX_close(NVS_Handle handle)
{}

/*
 *  ======== NVSCC32XX_control ========
 */
int NVSCC32XX_control(NVS_Handle handle, unsigned int cmd, uintptr_t arg)
{
    NVSCC32XX_HWAttrs *hwAttrs             = (NVSCC32XX_HWAttrs *)(handle->hwAttrs);
    NVSCC32XX_CmdSetCopyBlockArgs *cmdArgs = (NVSCC32XX_CmdSetCopyBlockArgs *)arg;
    uint8_t *copyBlock                     = (uint8_t *)(cmdArgs->copyBlock);

    if (cmd == NVSCC32XX_CMD_SET_COPYBLOCK)
    {
        if ((copyBlock == NULL) || ((uint32_t)copyBlock & 0x3))
        {
            return (NVSCC32XX_STATUS_ECOPYBLOCK);
        }

        hwAttrs->copyBlock = cmdArgs->copyBlock;

        return (NVS_STATUS_SUCCESS);
    }

    return (NVS_STATUS_UNDEFINEDCMD);
}

/*
 *  ======== NVSCC32XX_exit ========
 */
void NVSCC32XX_exit(NVS_Handle handle)
{}

/*
 *  ======== NVSCC32XX_getAttrs ========
 */
int NVSCC32XX_getAttrs(NVS_Handle handle, NVS_Attrs *attrs)
{
    NVSCC32XX_HWAttrs const *hwAttrs = handle->hwAttrs;
    attrs->pageSize                  = 4096; // TODO: Does this make sense for CC32XX?
    attrs->blockSize                 = hwAttrs->blockSize;

    return (NVS_SOK);
}

/*
 *  ======== NVSCC32XX_init ========
 */
void NVSCC32XX_init(NVS_Handle handle)
{
    int i;

    if (!isInitialized)
    {
        Semaphore_construct(&writeSem, 1, NULL);

        for (i = 0; i < NVS_count; i++)
        {
            ((NVSCC32XX_Object *)(NVS_config[i].object))->id = i;
        }

        ((NVSCC32XX_Object *)(handle->object))->opened = false;
        isInitialized                                  = true;
    }
}

/*
 *  ======== NVSCC32XX_open =======
 */
NVS_Handle NVSCC32XX_open(NVS_Handle handle, NVS_Params *params)
{
    NVSCC32XX_Object *object = handle->object;

    Semaphore_pend(Semaphore_handle(&writeSem), BIOS_WAIT_FOREVER);

    if (object->opened == true)
    {
        Semaphore_post(Semaphore_handle(&writeSem));

        Log_warning1("NVS:(%p) already in use.", (IArg)(object->id));
        return (NULL);
    }

    object->opened = true;
    Semaphore_post(Semaphore_handle(&writeSem));

    return (handle);
}

/*
 *  ======== NVSCC32XX_read =======
 */
int NVSCC32XX_read(NVS_Handle handle, size_t offset, void *buffer, size_t bufferSize)
{
    NVSCC32XX_Object *object = handle->object;
    char filename[32];
    long fsHandle;
    long bytesRead;
    int status;
    int retval = NVS_EFAIL;

    snprintf(filename, sizeof(filename), PATHFORMAT, object->id);

    /*
     *  Get exclusive access to the block.  We don't want someone
     *  else to erase the block while we are reading it.
     */
    Semaphore_pend(Semaphore_handle(&writeSem), BIOS_WAIT_FOREVER);

    //    fsHandle = sl_FsOpen((unsigned char *)filename, SL_FS_FILE_MODE_OPEN_READ,
    //                         NULL);
    status = sl_FsOpen((unsigned char *)filename, FS_MODE_OPEN_READ, NULL, &fsHandle);
    if (/* fsHandle > 0 */ status == 0)
    {
        bytesRead = sl_FsRead(fsHandle, offset, buffer, bufferSize);

        if (bytesRead > 0)
        {
            retval = NVS_SOK;
        }

        status = sl_FsClose(fsHandle, 0, 0, 0);
        if (status != 0)
        {
            Log_warning1("NVS:(%d) sl_FsClose() failed.", (IArg)(object->id));
        }
    }
    else
    {
        Log_warning1("NVS:(%d) sl_FsOpen() failed.", (IArg)(object->id));
    }

    Semaphore_post(Semaphore_handle(&writeSem));

    return (retval);
}

/*
 *  ======== NVSCC32XX_write =======
 */
int NVSCC32XX_write(NVS_Handle handle, size_t offset, void *buffer, size_t bufferSize, unsigned int flags)
{
    NVSCC32XX_HWAttrs const *hwAttrs = handle->hwAttrs;
    NVSCC32XX_Object *object         = handle->object;
    char filename[32];
    long fsHandle;
    uint8_t *srcBuf = (uint8_t *)(hwAttrs->copyBlock);
    long bytesWritten;
    long bytesRead;
    int status;
    int retval = NVS_EFAIL;

    snprintf(filename, sizeof(filename), PATHFORMAT, object->id);

    /* If the buffer is null, delete the file. */
    if (buffer == NULL)
    {
        status = sl_FsOpen((unsigned char *)filename, FS_MODE_OPEN_READ, NULL, &fsHandle);
        if (status == 0)
        {
            status = sl_FsClose(fsHandle, 0, 0, 0);
            if (status == 0)
            {
                status = sl_FsDel((unsigned char *)filename, 0);
            }
        }
        if (status == 0)
        {
            retval = NVS_SOK;
        }
        else
        {
            Log_warning1("NVS:(%d) sl_FsDel() failed.", (IArg)(object->id));
        }

        return (retval);
    }

    if (offset + bufferSize > NVSCC32XX_MAXFILESIZE)
    {
        Log_warning1("NVS:(%d) offset + bufferSize exceeds maximum.", (IArg)(object->id));
        return (NVS_EOFFSET);
    }

    if (hwAttrs->copyBlock == NULL)
    {
        Log_warning1("NVS:(%d) copy block must not be null.", (IArg)(object->id));
        return (NVS_ECOPYBLOCK);
    }

    memset(hwAttrs->copyBlock, 0, NVSCC32XX_MAXFILESIZE);

    Semaphore_pend(Semaphore_handle(&writeSem), BIOS_WAIT_FOREVER);

    /* see if the file exists */
    //    fsHandle = sl_FsOpen((unsigned char *)filename, SL_FS_FILE_MODE_OPEN_READ,
    //                         NULL);
    status = sl_FsOpen((unsigned char *)filename, FS_MODE_OPEN_READ, NULL, &fsHandle);

    if ((fsHandle >= 0) && (flags & NVS_WRITE_EXCLUSIVE))
    {
        // TODO: How can we tell if area from offset to offset + bufferSize
        // was already written?
        status = sl_FsClose(fsHandle, 0, 0, 0);
        Semaphore_post(Semaphore_handle(&writeSem));
        return (NVS_EALREADYWRITTEN);
    }

    if (/* fsHandle < 0 */ status != 0)
    {
        /* The file didn't exist, so create it. */
        //        fsHandle = sl_FsOpen((unsigned char *)filename,
        //                SL_FS_FILE_MODE_OPEN_CREATE(NVSCC32XX_MAXFILESIZE,
        //                                  SL_FS_FILE_OPEN_FLAG_FAILSAFE |
        //                                  SL_FS_FILE_OPEN_FLAG_PUBLIC_WRITE |
        //                                  SL_FS_FILE_OPEN_FLAG_PUBLIC_READ),
        status = sl_FsOpen((unsigned char *)filename,
                           FS_MODE_OPEN_CREATE(NVSCC32XX_MAXFILESIZE,
                                               _FS_FILE_OPEN_FLAG_COMMIT | _FS_FILE_PUBLIC_WRITE |
                                                   _FS_FILE_PUBLIC_READ),
                           NULL,
                           &fsHandle);
    }
    else
    {
        /* The file already existed, so read its conntents into copyBlock */
        bytesRead = sl_FsRead(fsHandle, 0, hwAttrs->copyBlock, NVSCC32XX_MAXFILESIZE);

        /* Close the file since we will need to open it again for writing */
        status = sl_FsClose(fsHandle, 0, 0, 0);
        if (bytesRead == NVSCC32XX_MAXFILESIZE)
        {
            //            fsHandle = sl_FsOpen((unsigned char *)filename,
            //                    SL_FS_FILE_MODE_OPEN_WRITE, NULL);
            status = sl_FsOpen((unsigned char *)filename, FS_MODE_OPEN_WRITE, NULL, &fsHandle);
        }
        else
        {
            Semaphore_post(Semaphore_handle(&writeSem));
            Log_warning1("NVS:(%d) sl_FsRead() failed.", (IArg)(object->id));
            return (NVS_EFAIL);
        }
    }

    if ((fsHandle < 0) || (status != 0))
    {
        Semaphore_post(Semaphore_handle(&writeSem));
        Log_warning1("NVS:(%d) sl_FsOpen() failed.", (IArg)(object->id));
        return (NVS_EFAIL);
    }

    memcpy((void *)(srcBuf + offset), buffer, bufferSize);
    bytesWritten = sl_FsWrite(fsHandle, 0, srcBuf, NVSCC32XX_MAXFILESIZE);
    if (bytesWritten != NVSCC32XX_MAXFILESIZE)
    {
        Log_warning1("NVS:(%d) sl_FsWrite() failed.", (IArg)(object->id));
    }
    else
    {
        retval = NVS_SOK;
    }

    status = sl_FsClose(fsHandle, 0, 0, 0);
    if (status != 0)
    {
        Log_warning1("NVS:(%d) sl_FsClose() failed.", (IArg)(object->id));
        retval = NVS_EFAIL;
    }

    Semaphore_post(Semaphore_handle(&writeSem));

    return (retval);
}
