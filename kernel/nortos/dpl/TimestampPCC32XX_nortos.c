/*
 * Copyright (c) 2022-2023, Texas Instruments Incorporated
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
 *  ======== TimestampPCC32XX.c ========
 */
#include <stdint.h>

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(inc/hw_types.h)
#include DeviceFamily_constructPath(driverlib/prcm.h)
#include DeviceFamily_constructPath(driverlib/rom.h)
#include DeviceFamily_constructPath(driverlib/rom_map.h)
#include <ti/drivers/dpl/TimestampP.h>

#if defined(__IAR_SYSTEMS_ICC__)
__root const TimestampP_Format
#elif defined(__TI_COMPILER_VERSION__) || (defined(__clang__) && defined(__ti_version__)) || defined(__GNUC__)
const TimestampP_Format __attribute__((used))
#endif
    TimestampP_nativeFormat64 = {
        .format = {.exponent = TimestampP_Exponent_Seconds, .fracBytes = 0, .intBytes = 6, .multiplier = -32768}};

#if defined(__IAR_SYSTEMS_ICC__)
__root const TimestampP_Format
#elif defined(__TI_COMPILER_VERSION__) || (defined(__clang__) && defined(__ti_version__)) || defined(__GNUC__)
const TimestampP_Format __attribute__((used))
#endif
    TimestampP_nativeFormat32 = {
        .format = {.exponent = TimestampP_Exponent_Seconds, .fracBytes = 0, .intBytes = 4, .multiplier = -32768}};

/* macro to pick two matching count values */
#define COUNT_WITHIN_TRESHOLD(a, b, c, th) ((((b) - (a)) <= (th)) ? (b) : (c))

uint64_t TimestampP_getNative64()
{
    uint64_t count[3];
    uint64_t curr;
    uint32_t i;

    /*
     *  get the current RTC count, using the fast interface; to use the
     *  fast interface the count must be read three times, and then
     *  the value that matches on at least two of the reads is chosen
     */
    for (i = 0; i < 3; i++)
    {
        count[i] = MAP_PRCMSlowClkCtrFastGet();
    }
    curr = COUNT_WITHIN_TRESHOLD(count[0], count[1], count[2], 1);
    return (curr);
}

uint32_t TimestampP_getNative32()
{
    uint64_t stmp = TimestampP_getNative64();

    return ((uint32_t)stmp);
}
