/*
 * Copyright (c) 2020-2022, Texas Instruments Incorporated
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
 *  ======== UART2CC32XX.c ========
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <ti/drivers/dpl/ClockP.h>
#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/dpl/SemaphoreP.h>

#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC32XX.h>
#include <ti/drivers/dma/UDMACC32XX.h>

#include <ti/drivers/uart2/UART2CC32XX.h>
#include <ti/drivers/uart2/UART2Support.h>

/* driverlib header files */
#include <ti/devices/cc32xx/inc/hw_memmap.h>
#include <ti/devices/cc32xx/inc/hw_ocp_shared.h>
#include <ti/devices/cc32xx/inc/hw_ints.h>
#include <ti/devices/cc32xx/inc/hw_types.h>
#include <ti/devices/cc32xx/inc/hw_udma.h>
#include <ti/devices/cc32xx/inc/hw_uart.h>
#include <ti/devices/cc32xx/driverlib/rom.h>
#include <ti/devices/cc32xx/driverlib/rom_map.h>
#include <ti/devices/cc32xx/driverlib/uart.h>
#include <ti/devices/cc32xx/driverlib/pin.h>
#include <ti/devices/cc32xx/driverlib/udma.h>

/* Headers required for intrinsics */
#if defined(__TI_COMPILER_VERSION__)
    #include <arm_acle.h>
#elif defined(__GNUC__)
    #include <arm_acle.h>
#elif defined(__IAR_SYSTEMS_ICC__)
    #include <intrinsics.h>
#else
    #error "Unsupported compiler"
#endif

/* Pad configuration defines */
#define PAD_CONFIG_BASE (OCP_SHARED_BASE + OCP_SHARED_O_GPIO_PAD_CONFIG_0)
#define PAD_RESET_STATE 0xC61

/* Maximum number of bytes that DMA can transfer */
#define MAX_SIZE 1024

/* Options for DMA write and read */
#define TX_CONTROL_OPTS (UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE | UDMA_ARB_4)
#define RX_CONTROL_OPTS (UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 | UDMA_ARB_4)

/* Static functions */
static uint_fast16_t UART2CC32XX_getPowerMgrId(uint32_t baseAddr);
static void UART2CC32XX_eventCallback(UART2_Handle handle, uint32_t event, uint32_t data, void *userArg);
static void UART2CC32XX_hwiIntFxn(uintptr_t arg);
static void UART2CC32XX_initHw(UART2_Handle handle);
static int UART2CC32XX_postNotifyFxn(unsigned int eventType, uintptr_t eventArg, uintptr_t clientArg);
static void UART2CC32XX_hwiIntWrite(uintptr_t arg);
static void UART2CC32XX_hwiIntRead(uintptr_t arg, uint32_t status);

/* Map UART2 data length to driverlib data length */
static const uint8_t dataLength[] = {
    UART_CONFIG_WLEN_5, /* UART2_DataLen_5 */
    UART_CONFIG_WLEN_6, /* UART2_DataLen_6 */
    UART_CONFIG_WLEN_7, /* UART2_DataLen_7 */
    UART_CONFIG_WLEN_8  /* UART2_DataLen_8 */
};

/* Map UART2 stop bits to driverlib stop bits */
static const uint8_t stopBits[] = {
    UART_CONFIG_STOP_ONE, /* UART2_StopBits_1 */
    UART_CONFIG_STOP_TWO  /* UART2_StopBits_2 */
};

/* Map UART2 parity type to driverlib parity type */
static const uint8_t parityType[] = {
    UART_CONFIG_PAR_NONE, /* UART2_Parity_NONE */
    UART_CONFIG_PAR_EVEN, /* UART2_Parity_EVEN */
    UART_CONFIG_PAR_ODD,  /* UART2_Parity_ODD */
    UART_CONFIG_PAR_ZERO, /* UART2_Parity_ZERO */
    UART_CONFIG_PAR_ONE   /* UART2_Parity_ONE */
};

/*
 *  ======== UART2CC32XX_getRxData ========
 *  Must be called with HWI disabled.
 */
static inline size_t UART2CC32XX_getRxData(UART2_Handle handle, size_t size)
{
    UART2CC32XX_Object *object         = handle->object;
    UART2CC32XX_HWAttrs const *hwAttrs = handle->hwAttrs;
    size_t consumed                    = 0;
    uint8_t data;

    while (!(HWREG(hwAttrs->baseAddr + UART_O_FR) & UART_FR_RXFE) && size)
    {
        data = HWREG(hwAttrs->baseAddr + UART_O_DR);
        RingBuf_put(&object->rxBuffer, data);
        ++consumed;
        --size;
    }

    return (consumed);
}

/*
 *  ======== UART2CC32XX_getRxStatus ========
 *  Get the left-most bit set in the RX error status (OE, BE, PE, FE)
 *  read from the RSR register:
 *      bit#   3   2   1   0
 *             OE  BE  PE  FE
 *  e.g., if OE and FE are both set, OE wins.  This will make it easier
 *  to convert an RX error status to a UART2 error code.
 */
static inline uint32_t UART2CC32XX_getRxStatus(uint32_t bitMask)
{
#if defined(__TI_COMPILER_VERSION__)
    return ((uint32_t)(bitMask & (0x80000000 >> __clz(bitMask))));
#elif defined(__GNUC__)
    return ((uint32_t)(bitMask & (0x80000000 >> __builtin_clz(bitMask))));
#elif defined(__IAR_SYSTEMS_ICC__)
    return ((uint32_t)(bitMask & (0x80000000 >> __CLZ(bitMask))));
#else
    #error "Unsupported compiler"
#endif
}

/*
 * Function for checking whether flow control is enabled.
 */
static inline bool UART2CC32XX_isFlowControlEnabled(UART2CC32XX_HWAttrs const *hwAttrs)
{
    return (hwAttrs->flowControl == UART2_FLOWCTRL_HARDWARE);
}

/*
 *  ======== UART2_close ========
 */
void UART2_close(UART2_Handle handle)
{
    UART2CC32XX_Object *object         = handle->object;
    UART2CC32XX_HWAttrs const *hwAttrs = handle->hwAttrs;
    uint32_t padRegister;

    /* Disable UART and interrupts. */
    UARTIntDisable(hwAttrs->baseAddr,
                   UART_INT_RX | UART_INT_RT | UART_INT_OE | UART_INT_BE | UART_INT_PE | UART_INT_FE | UART_INT_EOT);

    /* Disable UART and interrupts. */
    UARTDMADisable(hwAttrs->baseAddr, UART_DMA_TX | UART_DMA_RX);

    /* Releases Power constraint if rx is enabled. */
    UART2_rxDisable(handle);

    /* Disable UART. This function will wait until TX FIFO is empty before
     * shutting down peripheral, otherwise the BUSY-bit will remain set and
     * can cause the peripheral to get stuck.
     */
    UARTDisable(hwAttrs->baseAddr);
    object->state.txEnabled = false;

    HwiP_destruct(&(object->hwi));
    SemaphoreP_destruct(&(object->writeSem));
    SemaphoreP_destruct(&(object->readSem));

    if (object->udmaHandle)
    {
        UDMACC32XX_close(object->udmaHandle);
    }

    Power_unregisterNotify(&object->postNotify);
    Power_releaseDependency(object->powerMgrId);

    if (object->txPin != (uint16_t)-1)
    {
        PowerCC32XX_restoreParkState((PowerCC32XX_Pin)object->txPin, object->prevParkTX);
        object->txPin = (uint16_t)-1;
    }

    if (object->rtsPin != (uint16_t)-1)
    {
        PowerCC32XX_restoreParkState((PowerCC32XX_Pin)object->rtsPin, object->prevParkRTS);
        object->rtsPin = (uint16_t)-1;
    }

    /* Restore pin pads to their reset states */
    padRegister        = (PinToPadGet((hwAttrs->rxPin) & 0xff) << 2) + PAD_CONFIG_BASE;
    HWREG(padRegister) = PAD_RESET_STATE;
    padRegister        = (PinToPadGet((hwAttrs->txPin) & 0xff) << 2) + PAD_CONFIG_BASE;
    HWREG(padRegister) = PAD_RESET_STATE;
    if (UART2CC32XX_isFlowControlEnabled(hwAttrs))
    {
        if (hwAttrs->ctsPin != UART2CC32XX_PIN_UNASSIGNED)
        {
            padRegister        = (PinToPadGet((hwAttrs->ctsPin) & 0xff) << 2) + PAD_CONFIG_BASE;
            HWREG(padRegister) = PAD_RESET_STATE;
        }

        if (hwAttrs->rtsPin != UART2CC32XX_PIN_UNASSIGNED)
        {
            padRegister        = (PinToPadGet((hwAttrs->rtsPin) & 0xff) << 2) + PAD_CONFIG_BASE;
            HWREG(padRegister) = PAD_RESET_STATE;
        }
    }

    object->state.opened = false;
}

/*
 *  ======== UART2_flushRx ========
 */
void UART2_flushRx(UART2_Handle handle)
{
    UART2CC32XX_Object *object         = handle->object;
    UART2CC32XX_HWAttrs const *hwAttrs = handle->hwAttrs;
    uintptr_t key;

    key = HwiP_disable();
    UART2Support_dmaStopRx(handle);
    HwiP_restore(key);

    RingBuf_flush(&object->rxBuffer);

    /* Read RX FIFO until empty */
    while (((int32_t)MAP_UARTCharGetNonBlocking(hwAttrs->baseAddr)) != -1) {}

    /* Clear any read errors */
    MAP_UARTRxErrorClear(hwAttrs->baseAddr);

    key = HwiP_disable();
    /* Start DMA RX */
    UART2Support_dmaStartRx(handle);
    HwiP_restore(key);
}

/*
 *  ======== UART2_open ========
 */
UART2_Handle UART2_open(uint_least8_t index, UART2_Params *params)
{
    UART2_Handle handle = NULL;
    UART2CC32XX_Object *object;
    UART2CC32XX_HWAttrs const *hwAttrs;
    HwiP_Params hwiParams;
    uintptr_t key;
    uint16_t pin;
    uint16_t mode;

    if (index < UART2_count)
    {
        handle  = (UART2_Handle) & (UART2_config[index]);
        hwAttrs = handle->hwAttrs;
        object  = handle->object;
    }
    else
    {
        return (NULL);
    }

    /* Check for callback when in UART2_Mode_CALLBACK */
    if (((params->readMode == UART2_Mode_CALLBACK) && (params->readCallback == NULL)) ||
        ((params->writeMode == UART2_Mode_CALLBACK) && (params->writeCallback == NULL)))
    {
        return (NULL);
    }

    key = HwiP_disable();

    if (object->state.opened)
    {
        HwiP_restore(key);
        return (NULL);
    }
    object->state.opened = true;

    HwiP_restore(key);

    object->state.rxEnabled       = false;
    object->state.txEnabled       = false;
    object->state.rxCancelled     = false;
    object->state.txCancelled     = false;
    object->state.overrunActive   = false;
    object->state.inReadCallback  = false;
    object->state.inWriteCallback = false;
    object->state.overrunCount    = 0;
    object->state.readMode        = params->readMode;
    object->state.writeMode       = params->writeMode;
    object->state.readReturnMode  = params->readReturnMode;
    object->readCallback          = params->readCallback;
    object->writeCallback         = params->writeCallback;
    object->eventCallback         = params->eventCallback;
    object->eventMask             = params->eventMask;
    object->baudRate              = params->baudRate;
    object->stopBits              = params->stopBits;
    object->dataLength            = params->dataLength;
    object->parityType            = params->parityType;
    object->userArg               = params->userArg;

    /* Set UART transaction variables to defaults. */
    object->writeBuf   = NULL;
    object->readBuf    = NULL;
    object->writeCount = 0;
    object->readCount  = 0;
    object->writeSize  = 0;
    object->readSize   = 0;
    object->rxStatus   = 0;
    object->txStatus   = 0;
    object->rxSize     = 0;
    object->txSize     = 0;
    object->readInUse  = false;
    object->writeInUse = false;
    object->udmaHandle = NULL;

    object->txPin  = (uint16_t)-1;
    object->rtsPin = (uint16_t)-1;

    /* Determine the Power resource Id from the UART base address */
    object->powerMgrId = UART2CC32XX_getPowerMgrId(hwAttrs->baseAddr);
    if (object->powerMgrId >= PowerCC32XX_NUMRESOURCES)
    {
        object->state.opened = false;
        return (NULL);
    }

    /* Register power dependency. Keeps the clock running in SLP and DSLP modes. */
    Power_setDependency(object->powerMgrId);

    /* Do a software reset of the peripheral */
    PowerCC32XX_reset(object->powerMgrId);

    /* Set the event mask to 0 if the callback is NULL to simplify checks */
    if (object->eventCallback == NULL)
    {
        object->eventCallback = UART2CC32XX_eventCallback;
        object->eventMask     = 0;
    }

    RingBuf_construct(&object->rxBuffer, hwAttrs->rxBufPtr, hwAttrs->rxBufSize);
    RingBuf_construct(&object->txBuffer, hwAttrs->txBufPtr, hwAttrs->txBufSize);
    pin  = (hwAttrs->rxPin) & 0xff;
    mode = (hwAttrs->rxPin >> 8) & 0xff;

    MAP_PinTypeUART((unsigned long)pin, (unsigned long)mode);

    pin  = (hwAttrs->txPin) & 0xff;
    mode = (hwAttrs->txPin >> 8) & 0xff;

    MAP_PinTypeUART((unsigned long)pin, (unsigned long)mode);

    /* Read and save TX pin park state; set to "don't park" while UART is
     * open as device default is logic '1' during LPDS
     */
    object->prevParkTX = (PowerCC32XX_ParkState)PowerCC32XX_getParkState((PowerCC32XX_Pin)pin);
    PowerCC32XX_setParkState((PowerCC32XX_Pin)pin, ~1);
    object->txPin = pin;

    if (UART2CC32XX_isFlowControlEnabled(hwAttrs))
    {
        if (hwAttrs->ctsPin != UART2CC32XX_PIN_UNASSIGNED)
        {
            pin  = (hwAttrs->ctsPin) & 0xff;
            mode = (hwAttrs->ctsPin >> 8) & 0xff;
            MAP_PinTypeUART((unsigned long)pin, (unsigned long)mode);
        }

        if (hwAttrs->rtsPin != UART2CC32XX_PIN_UNASSIGNED)
        {
            pin  = (hwAttrs->rtsPin) & 0xff;
            mode = (hwAttrs->rtsPin >> 8) & 0xff;
            MAP_PinTypeUART((unsigned long)pin, (unsigned long)mode);

            /* Read and save RTS pin park state; set to "don't park" while UART is
             * open as device default is logic '1' during LPDS
             */
            object->prevParkRTS = (PowerCC32XX_ParkState)PowerCC32XX_getParkState((PowerCC32XX_Pin)pin);
            PowerCC32XX_setParkState((PowerCC32XX_Pin)pin, ~1);
            object->rtsPin = pin;
        }

        /* Flow control will be enabled in UART2CC32XX_initHw() */
    }

    Power_registerNotify(&object->postNotify, PowerCC32XX_AWAKE_LPDS, UART2CC32XX_postNotifyFxn, (uintptr_t)handle);

    /* DMA first */
    UDMACC32XX_init();
    object->udmaHandle = UDMACC32XX_open();
    if (object->udmaHandle == NULL)
    {
        UART2_close(handle);
        return (NULL);
    }

    /* Initialize the hardware */
    UART2CC32XX_initHw(handle);

    HwiP_clearInterrupt(hwAttrs->intNum);

    HwiP_Params_init(&hwiParams);
    hwiParams.arg      = (uintptr_t)handle;
    hwiParams.priority = hwAttrs->intPriority;
    HwiP_construct(&(object->hwi), hwAttrs->intNum, UART2CC32XX_hwiIntFxn, &hwiParams);

    SemaphoreP_constructBinary(&(object->readSem), 0);
    SemaphoreP_constructBinary(&(object->writeSem), 0);

    return (handle);
}

/*
 *  ======== UART2Support_disableRx ========
 */
void UART2Support_disableRx(UART2_HWAttrs const *hwAttrs)
{
    MAP_UARTIntDisable(hwAttrs->baseAddr,
                       UART_INT_OE | UART_INT_BE | UART_INT_PE | UART_INT_FE | UART_INT_RT | UART_INT_RX);
    MAP_UARTIntClear(hwAttrs->baseAddr,
                     UART_INT_OE | UART_INT_BE | UART_INT_PE | UART_INT_FE | UART_INT_RT | UART_INT_RX);
    HWREG(hwAttrs->baseAddr + UART_O_CTL) &= ~UART_CTL_RXE;
}

/*
 *  ======== UART2Support_disableTx ========
 */
void UART2Support_disableTx(UART2_HWAttrs const *hwAttrs)
{
    HWREG(hwAttrs->baseAddr + UART_O_CTL) &= ~(UART_CTL_TXE);
}

/*
 *  ======== UART2Support_dmaStartRx ========
 *  For mutual exclusion, must be called with HWI disabled.
 */
void UART2Support_dmaStartRx(UART2_Handle handle)
{
    UART2CC32XX_Object *object         = handle->object;
    UART2CC32XX_HWAttrs const *hwAttrs = handle->hwAttrs;
    unsigned char *dstAddr;

    if (object->state.rxEnabled == false)
    {
        /* DMA RX not enabled. No need to return a status code,
         * this function should never be called if RX is disabled
         */
        return;
    }

    /* Stop DMA RX which updates the ring buffer count */
    UART2Support_dmaStopRx(handle);

    /* Decide whether to either read from the FIFO into the ring buffer, into the user-buffer, or do nothing */
    if ((object->rxSize == 0 && (object->readBuf == NULL || object->readCount == 0)) || /* a) */
        (RingBuf_getCount(&object->rxBuffer) >= object->readCount) ||                   /* b) */
        (object->state.readMode == UART2_Mode_NONBLOCKING))                             /* c) */
    {
        /* Setup a DMA transaction from FIFO to ring buffer if either
         * a) Currently no active DMA RX transaction, and (no user-buffer available or no more bytes left to read)
         * b) There are enough bytes in the ring buffer for this current read operation
         * c) The driver is setup for a nonblocking read
         */
        object->rxSize              = RingBuf_putPointer(&object->rxBuffer, &dstAddr);
        object->state.readToRingbuf = true;
    }
    else if (object->rxSize > 0 && (object->readBuf == NULL || object->readCount == 0))
    {
        /* If there is an active DMA transaction into the ring buffer, and there is no user-buffer available,
         * then keep doing that transaction. Do nothing further here
         */
        return;
    }
    else if (object->readBuf != NULL)
    {
        /* If there is a user-buffer available, and neither of the cases above are true, then the driver should setup
         * a DMA transaction into the the user-buffer. Offset the transaction with the number of bytes we already have
         * in the ring buffer. The bytes in the ring buffer will be copied out at a later stage, in UART2_read
         */
        object->rxSize              = object->readCount - RingBuf_getCount(&object->rxBuffer);
        dstAddr                     = object->readBuf + object->bytesRead + RingBuf_getCount(&object->rxBuffer);
        object->state.readToRingbuf = false;
    }
    else
    {
        /* It should not be possible to reach here */
        return;
    }

    if (object->rxSize > 0)
    {
        object->state.overrunActive = false;

        if (object->rxSize > MAX_SIZE)
        {
            object->rxSize = MAX_SIZE;
        }

        /* Set the transfer src, dst, and size in the control options */
        MAP_uDMAChannelControlSet(hwAttrs->rxDmaChannel, RX_CONTROL_OPTS);
        MAP_uDMAChannelTransferSet(hwAttrs->rxDmaChannel,
                                   UDMA_MODE_BASIC,
                                   (void *)(hwAttrs->baseAddr + UART_O_DR),
                                   (void *)dstAddr,
                                   object->rxSize);

        /* Enable burst mode */
        HWREG(UDMA_BASE + UDMA_O_USEBURSTSET) = 1 << hwAttrs->rxDmaChannel;
        MAP_uDMAChannelEnable(hwAttrs->rxDmaChannel);

        MAP_UARTIntEnable(hwAttrs->baseAddr, UART_INT_DMARX);

        MAP_UARTDMAEnable(hwAttrs->baseAddr, UART_DMA_RX);
    }
}

/*
 *  ======== UART2Support_dmaStartTx ========
 *  For mutual exclusion, must be called with HWI disabled.
 */
void UART2Support_dmaStartTx(UART2_Handle handle)
{
    UART2CC32XX_Object *object         = handle->object;
    UART2CC32XX_HWAttrs const *hwAttrs = handle->hwAttrs;
    unsigned char *srcAddr;

    if (object->txSize > 0)
    {
        /* DMA TX already in progress */
        return;
    }

    /* If nonblocking, set txSize to available bytes in ring buffer.
     * Set source address to the ring buffer.
     */
    if (object->state.writeMode == UART2_Mode_NONBLOCKING)
    {
        object->txSize = RingBuf_getPointer(&object->txBuffer, &srcAddr);
    }
    else
    {
        /* Blocking or callback mode */
        object->txSize = object->writeCount;
        srcAddr        = (unsigned char *)object->writeBuf + object->bytesWritten;
    }

    if (object->txSize > 0)
    {
        UARTIntDisable(hwAttrs->baseAddr, UART_INT_EOT);
        if ((object->eventMask & UART2_EVENT_TX_BEGIN) && object->eventCallback)
        {
            object->eventCallback(handle, UART2_EVENT_TX_BEGIN, 0, object->userArg);
        }
        if (object->txSize > MAX_SIZE)
        {
            object->txSize = MAX_SIZE;
        }

        /* Set the transfer src, dst and size in the control options */
        MAP_uDMAChannelControlSet(hwAttrs->txDmaChannel, TX_CONTROL_OPTS);
        MAP_uDMAChannelTransferSet(hwAttrs->txDmaChannel,
                                   UDMA_MODE_BASIC,
                                   (void *)srcAddr,
                                   (void *)(hwAttrs->baseAddr + UART_O_DR),
                                   object->txSize);

        MAP_uDMAChannelEnable(hwAttrs->txDmaChannel);

        MAP_UARTIntEnable(hwAttrs->baseAddr, UART_INT_DMATX);

        MAP_UARTDMAEnable(hwAttrs->baseAddr, UART_DMA_TX);

        if (object->state.txEnabled == false)
        {
            /* Set constraints to guarantee transaction */
            Power_setConstraint(PowerCC32XX_DISALLOW_LPDS);
            object->state.txEnabled = true;
        }
    }
}

/*
 *  ======== UART2Support_dmaStopRx ========
 *  For mutual exclusion, must be called with HWI disabled.
 */
void UART2Support_dmaStopRx(UART2_Handle handle)
{
    UART2CC32XX_Object *object         = handle->object;
    UART2CC32XX_HWAttrs const *hwAttrs = handle->hwAttrs;
    uint32_t bytesRemaining;
    uint32_t rxCount;

    if (object->rxSize > 0)
    {
        MAP_uDMAChannelDisable(hwAttrs->rxDmaChannel);
        MAP_UARTDMADisable(hwAttrs->baseAddr, UART_DMA_RX);
        bytesRemaining = MAP_uDMAChannelSizeGet(hwAttrs->rxDmaChannel);

        MAP_uDMAIntClear(hwAttrs->rxDmaChannel);
        rxCount = object->rxSize - bytesRemaining;

        /* If the driver is currently reading data into the ring buffer, update the ring buffer count with the
         * number of bytes transferred into it through DMA
         */
        if (object->state.readToRingbuf)
        {
            RingBuf_putAdvance(&object->rxBuffer, rxCount);
        }
        else
        {
            /* If the driver was reading data into the user-buffer, we update the number of bytes read,
             * and number of bytes still to read
             */
            object->readCount -= rxCount;
            object->bytesRead += rxCount;
        }

        object->rxSize = 0;
    }
}

/*
 *  ======== UART2Support_dmaStopTx ========
 *  For mutual exclusion, must be called with HWI disabled.
 */
uint32_t UART2Support_dmaStopTx(UART2_Handle handle)
{
    UART2CC32XX_Object *object         = handle->object;
    UART2CC32XX_HWAttrs const *hwAttrs = handle->hwAttrs;
    uint32_t bytesRemaining            = 0;
    uint32_t txCount;

    if (object->txSize > 0)
    {
        MAP_uDMAChannelDisable(hwAttrs->txDmaChannel);
        MAP_UARTDMADisable(hwAttrs->baseAddr, UART_DMA_TX);
        bytesRemaining = MAP_uDMAChannelSizeGet(hwAttrs->txDmaChannel);

        MAP_uDMAIntClear(hwAttrs->txDmaChannel);
        txCount = object->txSize - bytesRemaining;

        /* If the driver is currently doing a nonblocking write, update the ring buffer */
        if (object->state.writeMode == UART2_Mode_NONBLOCKING)
        {
            RingBuf_getConsume(&object->txBuffer, txCount);
        }
        else
        {
            /* If the driver was writing data from the user-buffer, we update the number of bytes written,
             * and number of bytes still to write
             */
            object->bytesWritten += txCount;
            object->writeCount -= txCount;
        }

        object->txSize = 0;
    }

    return (bytesRemaining);
}

/*
 *  ======== UART2Support_enableInts ========
 */
void UART2Support_enableInts(UART2_Handle handle)
{
    UART2CC32XX_Object *object         = handle->object;
    UART2CC32XX_HWAttrs const *hwAttrs = handle->hwAttrs;

    if (object->eventCallback)
    {
        if (object->eventMask & UART2_EVENT_OVERRUN)
        {
            MAP_UARTIntClear(hwAttrs->baseAddr, UART_INT_OE);
            MAP_UARTIntEnable(hwAttrs->baseAddr, UART_INT_OE);
        }
    }
    MAP_UARTIntEnable(hwAttrs->baseAddr, UART_INT_RT);
}

/*
 *  ======== UART2Support_enableRx ========
 *  Call with interrupts disabled
 */
void UART2Support_enableRx(UART2_HWAttrs const *hwAttrs)
{
    HWREG(hwAttrs->baseAddr + UART_O_CTL) |= UART_CTL_RXE;
}

/*
 *  ======== UART2Support_enableTx ========
 *  Call with interrupts disabled
 */
void UART2Support_enableTx(UART2_HWAttrs const *hwAttrs)
{
    HWREG(hwAttrs->baseAddr + UART_O_CTL) |= UART_CTL_TXE;
}

/*
 *  ======== UART2Support_powerRelConstraint ========
 */
void UART2Support_powerRelConstraint(__attribute__((unused)) UART2_Handle handle,
                                     __attribute__((unused)) bool relFlashConstraint)
{
    Power_releaseConstraint(PowerCC32XX_DISALLOW_LPDS);
}

/*
 *  ======== UART2Support_powerSetConstraint ========
 */
void UART2Support_powerSetConstraint(__attribute__((unused)) UART2_Handle handle,
                                     __attribute__((unused)) bool setFlashConstraint)
{
    Power_setConstraint(PowerCC32XX_DISALLOW_LPDS);
}

/*
 *  ======== UART2Support_rxStatus2ErrorCode ========
 *  Convert RX status (OE, BE, PE, FE) to a UART2 error code.
 */
int_fast16_t UART2Support_rxStatus2ErrorCode(uint32_t errorData)
{
    uint32_t status;

    status = UART2CC32XX_getRxStatus(errorData);
    return (-((int_fast16_t)status));
}

/*
 *  ======== UART2Support_sendData ========
 *  Function to send data
 */
uint32_t UART2Support_sendData(UART2_HWAttrs const *hwAttrs, size_t size, uint8_t *buf)
{
    uint32_t writeCount = 0;

    while (size)
    {
        if (!MAP_UARTCharPutNonBlocking(hwAttrs->baseAddr, *buf))
        {
            break;
        }
        buf++;
        writeCount++;
        size--;
    }

    return (writeCount);
}

/*
 *  ======== UART2Support_txDone ========
 */
bool UART2Support_txDone(UART2_HWAttrs const *hwAttrs)
{
    if (MAP_UARTBusy(hwAttrs->baseAddr))
    {
        return (false);
    }

    return (true);
}

/*
 *  ======== UART2Support_uartRxError ========
 *  Function to clear RX errors
 */
int UART2Support_uartRxError(UART2_HWAttrs const *hwAttrs)
{
    int status = UART2_STATUS_SUCCESS;
    uint32_t errStatus;

    /* Check for RX error since the last read */
    errStatus = UARTRxErrorGet(hwAttrs->baseAddr);
    status    = UART2Support_rxStatus2ErrorCode(errStatus);
    UARTRxErrorClear(hwAttrs->baseAddr); /* Clear receive errors */

    return (status);
}

/*
 *  ======== UART2CC32XX_eventCallback ========
 *  A dummy event callback function in case the user didn't provide one
 */
static void UART2CC32XX_eventCallback(UART2_Handle handle, uint32_t event, uint32_t data, void *userArg)
{}

/*
 *  ======== UART2CC32XX_getPowerMgrId ========
 */
static uint_fast16_t UART2CC32XX_getPowerMgrId(uint32_t baseAddr)
{
    switch (baseAddr)
    {
        case UARTA0_BASE:
            return (PowerCC32XX_PERIPH_UARTA0);
        case UARTA1_BASE:
            return (PowerCC32XX_PERIPH_UARTA1);
        default:
            return ((uint_fast16_t)(~0U));
    }
}

/*
 *  ======== UART2CC32XX_hwiIntWrite ========
 *  Function called by Hwi to handle read-related interrupt
 */
static void UART2CC32XX_hwiIntRead(uintptr_t arg, uint32_t status)
{
    UART2_Handle handle                = (UART2_Handle)arg;
    UART2CC32XX_HWAttrs const *hwAttrs = handle->hwAttrs;
    UART2CC32XX_Object *object         = handle->object;
    void *readBufCopy;

    /* Temporarily stop RX transaction */
    UART2Support_dmaStopRx(handle);

    /* Read timeout interrupt */
    if (status & UART_INT_RT)
    {
        /* If the read transaction has a user buffer as destination,
         * copy as much as we can from the FIFO to the buffer
         */
        if (!(object->state.readToRingbuf))
        {
            while (UARTCharsAvail(hwAttrs->baseAddr) && object->readCount)
            {
                uint8_t data                           = HWREG(hwAttrs->baseAddr + UART_O_DR);
                *(object->readBuf + object->bytesRead) = data;
                object->bytesRead++;
                object->readCount--;
            }
        }

        /* If the read transaction is setup with the ring buffer as destination then copy as much
         * as we can can from the FIFO into the ring buffer
         */
        if (object->state.readToRingbuf)
        {
            UART2CC32XX_getRxData(handle, RingBuf_space(&object->rxBuffer));
        }
    }

    /* Do not invoke callback or post semaphore if there is no active read, or readMode is nonblocking */
    if (object->readInUse && object->state.readMode != UART2_Mode_NONBLOCKING)
    {
        /* Read-transaction is complete if either
         * a) ReadReturnMode is partial and we received a read-timeout
         * b) There are no more bytes to read
         */
        if (((object->state.readReturnMode == UART2_ReadReturnMode_PARTIAL) && (status & UART_INT_RT)) ||
            (object->readCount == 0))
        {
            object->readInUse = false;
            object->readCount = 0;
            /* Set readBuf to NULL, but first make a copy to pass to the callback. We cannot set it to NULL after
             * the callback in case another read was issued from the callback.
             */
            readBufCopy       = object->readBuf;
            object->readBuf   = NULL;

            if (object->state.readMode == UART2_Mode_CALLBACK)
            {
                object->readCallback(handle, readBufCopy, object->bytesRead, object->userArg, UART2_STATUS_SUCCESS);
            }
            else
            {
                /* Blocking mode. Post semaphore to unblock reading task */
                SemaphoreP_post(&object->readSem);
            }

            /* Set readBuf to NULL *after* invoking readcallback */
            object->readBuf = NULL;
        }
    }

    /* Start another RX transaction */
    UART2Support_dmaStartRx(handle);
}

/*
 *  ======== UART2CC32XX_hwiIntWrite ========
 *  Function called by Hwi to handle write-related interrupt
 */
static void UART2CC32XX_hwiIntWrite(uintptr_t arg)
{
    UART2_Handle handle                = (UART2_Handle)arg;
    UART2CC32XX_HWAttrs const *hwAttrs = handle->hwAttrs;
    UART2CC32XX_Object *object         = handle->object;

    UARTDMADisable(hwAttrs->baseAddr, UART_DMA_TX);
    UARTIntDisable(hwAttrs->baseAddr, UART_INT_DMATX);
    UARTIntClear(hwAttrs->baseAddr, UART_INT_DMATX);

    /* DMA transaction finished. Restart in case there are more bytes to write */
    UART2Support_dmaStopTx(handle);
    UART2Support_dmaStartTx(handle);

    if ((object->state.writeMode == UART2_Mode_CALLBACK) && (object->writeCount == 0) && object->writeInUse)
    {
        object->writeInUse = false;
        object->writeCallback(handle,
                              (void *)object->writeBuf,
                              object->bytesWritten,
                              object->userArg,
                              UART2_STATUS_SUCCESS);
    }

    if (object->txSize == 0)
    {
        /* No more data pending in the TX buffer, wait for it to finish
         * shifting out of the transmit shift register.
         */
        UARTIntEnable(hwAttrs->baseAddr, UART_INT_EOT);
    }
}

/*
 *  ======== UART2CC32XX_hwiIntFxn ========
 *  Hwi function that processes UART interrupts.
 */
static void UART2CC32XX_hwiIntFxn(uintptr_t arg)
{
    uint32_t status;
    uint32_t errStatus = 0;
    uint32_t event;
    UART2_Handle handle                = (UART2_Handle)arg;
    UART2CC32XX_HWAttrs const *hwAttrs = handle->hwAttrs;
    UART2CC32XX_Object *object         = handle->object;

    /* Clear interrupts */
    status = MAP_UARTIntStatus(hwAttrs->baseAddr, true);
    MAP_UARTIntClear(hwAttrs->baseAddr, status);

    if (status & (UART_INT_OE | UART_INT_BE | UART_INT_PE | UART_INT_FE) && object->eventCallback)
    {
        if (status & UART_INT_OE)
        {
            /* Overrun error occurred, get what we can.  No need to stop
             * the DMA, should already have stopped by now.
             */
            UART2CC32XX_getRxData(handle, RingBuf_space(&object->rxBuffer));
            /* Throw away the rest in order to clear the overrun */
            while (!(HWREG(hwAttrs->baseAddr + UART_O_FR) & UART_FR_RXFE))
            {
                volatile uint8_t data = HWREG(hwAttrs->baseAddr + UART_O_DR);
                (void)data;
            }
            ++object->state.overrunCount;
            if (object->state.overrunActive == false)
            {
                object->state.overrunActive = true;
            }
        }

        errStatus = UARTRxErrorGet(hwAttrs->baseAddr);
        event     = UART2CC32XX_getRxStatus(errStatus & object->eventMask);

        if (event && object->eventCallback)
        {
            object->eventCallback(handle, event, object->state.overrunCount, object->userArg);
        }
        object->rxStatus = UART2Support_rxStatus2ErrorCode(errStatus);
    }

    /* Read data if characters are available. */
    if ((object->rxSize && !uDMAChannelIsEnabled(hwAttrs->rxDmaChannel)) || (status & UART_INT_RT))
    {
        UART2CC32XX_hwiIntRead(arg, status);
    }

    if (!MAP_uDMAChannelIsEnabled(hwAttrs->txDmaChannel) && object->txSize)
    {
        UART2CC32XX_hwiIntWrite(arg);
    }

    /* EOT interrupt received */
    if (status & (UART_INT_EOT))
    {
        /* End of Transmission occurred */
        if (object->state.txEnabled)
        {
            if ((object->eventMask & UART2_EVENT_TX_FINISHED) && object->eventCallback)
            {
                object->eventCallback(handle, UART2_EVENT_TX_FINISHED, 0, object->userArg);
            }

            Power_releaseConstraint(PowerCC32XX_DISALLOW_LPDS);

            object->state.txEnabled = false;
        }
        MAP_UARTIntDisable(hwAttrs->baseAddr, UART_INT_EOT);

        if (object->state.writeMode == UART2_Mode_BLOCKING)
        {
            SemaphoreP_post(&(object->writeSem));
        }
    }
}

/*
 *  ======== UART2CC32XX_initHw ========
 */
static void UART2CC32XX_initHw(UART2_Handle handle)
{
    UART2CC32XX_Object *object         = handle->object;
    UART2CC32XX_HWAttrs const *hwAttrs = handle->hwAttrs;
    ClockP_FreqHz freq;
    unsigned long mode = UART_FLOWCONTROL_NONE;

    ClockP_getCpuFreq(&freq);
    MAP_UARTConfigSetExpClk(hwAttrs->baseAddr,
                            freq.lo,
                            object->baudRate,
                            dataLength[object->dataLength] | stopBits[object->stopBits] |
                                parityType[object->parityType]);

    /* Clear all UART interrupts */
    UARTIntClear(hwAttrs->baseAddr,
                 UART_INT_OE | UART_INT_BE | UART_INT_PE | UART_INT_FE | UART_INT_RT | UART_INT_TX | UART_INT_RX |
                     UART_INT_CTS | UART_INT_EOT);

    UARTEnable(hwAttrs->baseAddr);

    UARTIntEnable(hwAttrs->baseAddr,
                  UART_INT_RX | UART_INT_RT | UART_INT_OE | UART_INT_BE | UART_INT_PE | UART_INT_FE | UART_INT_EOT);

    MAP_UARTFIFOLevelSet(hwAttrs->baseAddr, UART_FIFO_TX4_8, UART_FIFO_RX4_8);

    /* Set EOT-bit to trigger when transmission is fully complete */
    HWREG(hwAttrs->baseAddr + UART_O_CTL) |= UART_CTL_EOT;

    if (UART2CC32XX_isFlowControlEnabled(hwAttrs))
    {
        /* Set flow control */
        if (hwAttrs->ctsPin != UART2CC32XX_PIN_UNASSIGNED)
        {
            mode = UART_FLOWCONTROL_TX;
        }

        if (hwAttrs->rtsPin != UART2CC32XX_PIN_UNASSIGNED)
        {
            mode |= UART_FLOWCONTROL_RX;
        }
    }
    MAP_UARTFlowControlSet(hwAttrs->baseAddr, mode);

    if (hwAttrs->rxPin != UART2CC32XX_PIN_UNASSIGNED)
    {
        MAP_uDMAChannelAssign(hwAttrs->rxDmaChannel);

        uDMAChannelAttributeDisable(hwAttrs->rxDmaChannel,
                                    (UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST | UDMA_ATTR_REQMASK));
    }

    if (hwAttrs->txPin != UART2CC32XX_PIN_UNASSIGNED)
    {
        MAP_uDMAChannelAssign(hwAttrs->txDmaChannel);

        uDMAChannelAttributeDisable(hwAttrs->txDmaChannel,
                                    (UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST | UDMA_ATTR_REQMASK));
    }
}

/*
 *  ======== UART2CC32XX_postNotifyFxn ========
 *  Called by Power module when waking up from LPDS.
 */
static int UART2CC32XX_postNotifyFxn(unsigned int eventType, uintptr_t eventArg, uintptr_t clientArg)
{
    /* Reconfigure the hardware if returning from sleep */
    UART2CC32XX_initHw((UART2_Handle)clientArg);

    return (Power_NOTIFYDONE);
}
