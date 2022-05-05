/*
 * Copyright (c) 2016-2020, Texas Instruments Incorporated
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
 *  ======== capturepwmdisplay.c ========
 */
 /* POSIX Header files */
 #include <mqueue.h>

/* Driver Header files */
#include <ti/drivers/Capture.h>
#include <ti/display/Display.h>
#include <ti/drivers/PWM.h>
#include <stddef.h>

/* Driver configuration */
#include "ti_drivers_config.h"

/* Message queue attributes */
#define MESSAGE_LEN  (sizeof(uint32_t))
#define MAX_MESSAGES (10)

/* Used to pass data between callback and main task */
static mqd_t msgQ;

/* Display Driver Handle */
static Display_Handle display;

/*
 *  ======== captureCallback ========
 *  Callback used for putting the measured interval into a message queue
 */
void captureCallback(Capture_Handle handle, uint32_t interval,
                     int_fast16_t status)
{
    mq_send(msgQ, (char *)&interval, MESSAGE_LEN, 0);
}

/*
 *  ======== mainThread ========
 *  Task that will capture two rising edges and output the time between the
 *  two edges
 */
void *mainThread(void *arg0)
{
    Capture_Params captureParams;
    Capture_Handle capture;
    PWM_Params pwmParams;
    PWM_Handle pwm0;
    PWM_Handle pwm1;
    struct mq_attr attr = {0};
    uint32_t intervalMsg;

    /* units in microseconds */
    uint32_t   pwmPeriod = 100000;
    uint32_t   duty = 50000;

    /* Create RTOS Queue */
    attr.mq_maxmsg = MAX_MESSAGES;
    attr.mq_msgsize = MESSAGE_LEN;

    msgQ = mq_open("PWMBuf", O_RDWR | O_CREAT | O_NONBLOCK, 0664, &attr);

    if (msgQ == (mqd_t)-1) {
        /* Failed to open message queue */
        while (1);
    }

    /* Driver Init Functions */
    Capture_init();
    Display_init();
    PWM_init();

    /* Open Display for Output */
    display = Display_open(Display_Type_UART, NULL);

    if (display == NULL)
    {
        /* Failed to open display driver */
        while (1);
    }

    /* PWM Params init */
    PWM_Params_init(&pwmParams);
    pwmParams.dutyUnits = PWM_DUTY_US;
    pwmParams.dutyValue = 0;
    pwmParams.periodUnits = PWM_PERIOD_US;
    pwmParams.periodValue = pwmPeriod;

    /* Open PWM0 */
    pwm0 = PWM_open(CONFIG_PWM_0, &pwmParams);

    if (!pwm0)
    {
        Display_printf(display, 0, 0, "Failed to initialized PWM0.\n");
        while (1);
    }

    PWM_start(pwm0);

    /* Open PWM1 */
    pwm1 = PWM_open(CONFIG_PWM_1, &pwmParams);

    if (!pwm1)
    {
        Display_printf(display, 0, 0, "Failed to initialized PWM1.\n");
        while (1);
    }

    PWM_start(pwm1);

    /*
     * Setting up the Capture driver to detect two rising edges and report
     * the result in microseconds
     */
    Capture_Params_init(&captureParams);
    captureParams.mode = Capture_RISING_EDGE;
    captureParams.periodUnit = Capture_PERIOD_US;
    captureParams.callbackFxn = captureCallback;

    capture = Capture_open(CONFIG_CAPTURE_0, &captureParams);
    if (capture == NULL)
    {
        Display_printf(display, 0, 0, "Failed to initialized Capture!\n");
        while(1);
    }

    Display_printf(display, 0, 0, "About to Capture!\n");

    /* Set the PWM duty and start the capture */
    PWM_setDuty(pwm0, duty);
    PWM_setDuty(pwm1, duty);
    Capture_start(capture);

    while(1)
    {
        /*
         * Receive interval from message queue and store in intervalMsg.
         * The value printed should be close to the period of the pwm
         */
        if ((mq_receive(msgQ, (char *)&intervalMsg, MESSAGE_LEN, 0)) == MESSAGE_LEN) {
            Display_printf(display, 0, 0, "Period: %d\n", intervalMsg);
        }
    }
}
