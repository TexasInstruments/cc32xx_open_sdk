/*
 * Copyright (c) 2022, Texas Instruments Incorporated
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

/* global xdc */
let Program = xdc.module('xdc.rov.Program');
let Monitor = xdc.module("xdc.rov.runtime.Monitor");

/* eslint-disable-next-line no-unused-vars */
let moduleName = "NVIC";

/* eslint-disable-next-line no-unused-vars */
let viewMap = [
    {name: "Table", fxn: "getNVIC", structName: "NvicInterrupt"}
];

function NvicInterrupt(){
    this.InterruptNum = null;
    this.Description  = null;
    this.VTORHandler  = null;
    this.HwiPDispatch = null;
    this.Priority     = null;
    this.Enabled      = null;
    this.Active       = null;
    this.Pending      = null;
}

/* ======== getDescription ========
 * Returns the type of interrupt based on the interrupt number */
function getDescription(interruptNum){
    /* See page 19 of version 1b of Cortex M4 Devices Generic User Guide */
    if (interruptNum >= 16) {
        return "IRQ";
    }

    switch (interruptNum) {
        case 0:
            return "ThreadMode";
        case 2:
            return "NMI";
        case 3:
            return "HardFault";
        case 4:
            return "MemManage";
        case 5:
            return "BusFault"
        case 6:
            return "UsageFault";
        case 11:
            return "SVCall";
        case 14:
            return "PendSV";
        case 15:
            return "SysTick";
        default:
            return "Reserved";
    }
}

function CC23XXNVIC(){
    this.ICSR  = null;
    this.VTOR  = null;
    this.SHCSR = null;
    this.STCSR = null;
    this.ISER  = null;
    this.IABR  = null;
    this.ISPR  = null;
    this.AIRCR = null;
    this.SHPR  = null;
    this.IPR   = null;
}

/* ======== parseNVICManually ========
 * Fetches the NVIC fields that are needed. This is done when a NVIC struct
 * is not present in the dpl implementation. */
function parseNVICManually(address){
    let NVIC = new CC23XXNVIC();

    NVIC.STCSR = Program.fetchFromAddr(address + 16,   "uint32_t", 1);
    NVIC.ISER  = Program.fetchFromAddr(address + 256,  "uint32_t", 8);
    NVIC.ISPR  = Program.fetchFromAddr(address + 512,  "uint32_t", 8);
    NVIC.IABR  = Program.fetchFromAddr(address + 768,  "uint32_t", 8);
    NVIC.IPR   = Program.fetchFromAddr(address + 1024, "uint8_t",  240);
    NVIC.ICSR  = Program.fetchFromAddr(address + 3332, "uint32_t", 1);
    NVIC.VTOR  = Program.fetchFromAddr(address + 3336, "uint32_t", 1);
    NVIC.AIRCR = Program.fetchFromAddr(address + 3340, "uint32_t", 1);
    NVIC.SHPR  = Program.fetchFromAddr(address + 3352, "uint8_t",  12);
    NVIC.SHCSR = Program.fetchFromAddr(address + 3364, "uint32_t", 1);

    return NVIC;
}

/* ======== parseNVIC ========
 * Parses the NVIC based on its memory address, CC23XX does not have
 * a NVIC struct (that mirrors the memory leayout). Thus we parse it
 * manually in that case */
function parseNVIC(){
    let NVIC;
    let addressOfNVIC = 0xE000E000;
    try{
        NVIC = Program.fetchFromAddr(addressOfNVIC, "HwiP_NVIC", 1);
    }
    catch(e){
        // CC23XX or struct name has been changed. Either way parse NVIC manually
        NVIC = parseNVICManually(addressOfNVIC);
    }
    return NVIC;
}


/* ======== getVTORTable ========
 * Fetches the dispatch function addresses that corresponds to each interrupt */
function getVTORTable(NVIC, numInterrupts){
    let vtor = [];

    /* Bits 7-29 contains the address (other bits reserved) */
    let vtorMask    = 0x3fffff80;
    let vtorAddress = NVIC.VTOR & vtorMask;

    /* Fetch the interrupt handler function addresses from VTOR.
     * This will be a list of pointers to handler functions. */
    vtor = Program.fetchFromAddr(vtorAddress, "int32_t", numInterrupts);

    return vtor;
}

/* ======== getHwipDispatchMap ========
 * Creates a map from interruptNumber to name and address of
 * dispatch function for fast lookup. This is needed to not
 * have to iterate through the whole dispatchMap
 * once for every interrupt*/
function getHwipDispatchMap(dispatchTable){
    let map = {};

    for(let i = 0; i < dispatchTable.length; i++){
        hwiObjAddr = dispatchTable[i];
        if(hwiObjAddr == 0) continue;
        let hwiObj = Program.fetchFromAddr(hwiObjAddr, "HwiP_Obj", 1);
        let dispatchFuncName = String(Program.lookupFuncName(Number(hwiObj.fxn)));
        let dispatchFuncAddr = helperGetHexString(Number(hwiObj.fxn));
        map[hwiObj.intNum]   = [dispatchFuncAddr, dispatchFuncName];
    }

    return map;
}


/* ======== getNVIC ========
 * Main function for getting the NVIC view */
/* eslint-disable-next-line no-unused-vars */
function getNVIC(){
    let view = new Array();
    let NVIC = parseNVIC();

    /* See page 618 of version E.e ARMv7-M Architecture Reference Manual
     * for how to find an upper limit to the number of interrupts.
     * "let numPossibleInterrupts = 32 * ((NVIC.ICTR & 7) + 1)"
     * However using the length of the dispatch table gets us
     * the exact answer */
    let dispatchTable = Program.fetchVariable("HwiP_dispatchTable");
    let maxInterrupts = dispatchTable.length;

    let vtorAddresses = getVTORTable(NVIC, maxInterrupts);
    let dispatchMap   = getHwipDispatchMap(dispatchTable);

    for (let i = 0; i < maxInterrupts; i++) {
        let NvicInt = new NvicInterrupt();
        NvicInt.InterruptNum = i;
        NvicInt.Description = getDescription(i);
        let vtorHandlerName = String(Program.lookupFuncName(vtorAddresses[i]));
        NvicInt.VTORHandler = helperGetHexString(vtorAddresses[i]) + " (" + vtorHandlerName + ")";

        if (vtorHandlerName == "HwiP_dispatch") {
            let [dispatchAddr, dispatchName] = dispatchMap[i];
            NvicInt.HwiPDispatch = dispatchAddr + " (" + dispatchName + ")";
        }

        let [enabled, active, pending] = helperGetEnabledActivePending(i, NVIC);
        NvicInt.Enabled = (enabled == "-") ? "-" : (enabled ? 1 : 0);
        NvicInt.Active  = (active  == "-") ? "-" : (active  ? 1 : 0);
        NvicInt.Pending = (pending == "-") ? "-" : (pending ? 1 : 0);

        NvicInt.Priority = helperGetPriorityGivenIndex(i, NVIC);

        view.push(NvicInt);
    }

    return view;
}