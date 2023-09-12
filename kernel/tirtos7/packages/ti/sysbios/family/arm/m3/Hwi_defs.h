/*
 * Copyright (c) 2020, Texas Instruments Incorporated
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

/* @cond LONG_NAMES */

/*
 * ======== Hwi_defs.h ========
 */

#if defined(do_hwi_undef_short_names)
#undef do_hwi_undef_short_names
#undef Hwi_Instance
#undef Hwi_Handle
#undef Hwi_Module
#undef Hwi_Object
#undef Hwi_Struct
#undef Hwi_FuncPtr
#undef Hwi_Irp
#undef Hwi_HookSet
#undef Hwi_MaskingOption
#undef Hwi_StackInfo
#undef Hwi_VectorFuncPtr
#undef Hwi_ExceptionHookFuncPtr
#undef Hwi_CCR
#undef Hwi_NVIC
#undef Hwi_ExcContext
#undef Hwi_ExcHandlerFuncPtr
#undef Hwi_HandlerFuncPtr
#undef Hwi_Instance_State
#undef Hwi_Module_State
#undef Hwi_MaskingOption_NONE
#undef Hwi_MaskingOption_ALL
#undef Hwi_MaskingOption_SELF
#undef Hwi_MaskingOption_BITMASK
#undef Hwi_MaskingOption_LOWER
#undef Hwi_dispatcherAutoNestingSupport
#undef Hwi_dispatcherSwiSupport
#undef Hwi_dispatcherTaskSupport
#undef Hwi_dispatcherIrpTrackingSupport
#undef Hwi_NUM_INTERRUPTS
#undef Hwi_NUM_PRIORITIES
#undef Hwi_nullIsrFunc
#undef Hwi_excHandlerFunc
#undef Hwi_excHookFunc
#undef Hwi_excHookFuncs
#undef Hwi_disablePriority
#undef Hwi_priGroup
#undef Hwi_numSparseInterrupts
#undef Hwi_swiDisable
#undef Hwi_swiRestore
#undef Hwi_swiRestoreHwi
#undef Hwi_taskDisable
#undef Hwi_taskRestoreHwi
#undef Hwi_ccr
#undef Hwi_hooks
#undef Hwi_Params
#undef Hwi_getStackInfo
#undef Hwi_getCoreStackInfo
#undef Hwi_startup
#undef Hwi_switchFromBootStack
#undef Hwi_post
#undef Hwi_getTaskSP
#undef Hwi_disableInterrupt
#undef Hwi_enableInterrupt
#undef Hwi_restoreInterrupt
#undef Hwi_clearInterrupt
#undef Hwi_getFunc
#undef Hwi_setFunc
#undef Hwi_getHookContext
#undef Hwi_setHookContext
#undef Hwi_getIrp
#undef Hwi_construct2
#undef Hwi_disableFxn
#undef Hwi_enableFxn
#undef Hwi_restoreFxn
#undef Hwi_plug
#undef Hwi_getHandle
#undef Hwi_setPriority
#undef Hwi_excSetBuffers
#undef Hwi_initNVIC
#undef Hwi_initStacks
#undef Hwi_flushVnvic
#undef Hwi_testStaticInlines
#undef Hwi_reconfig
#undef Hwi_Module_name
#undef Hwi_Module_id
#undef Hwi_Module_startup
#undef Hwi_Module_startupDone
#undef Hwi_construct
#undef Hwi_create
#undef Hwi_handle
#undef Hwi_struct
#undef Hwi_Handle_label
#undef Hwi_Handle_name
#undef Hwi_Instance_init
#undef Hwi_Object_first
#undef Hwi_Object_next
#undef Hwi_Object_sizeof
#undef Hwi_Params_copy
#undef Hwi_Params_init
#undef Hwi_Instance_finalize
#undef Hwi_delete
#undef Hwi_destruct
#undef Hwi_dispatch
#undef Hwi_dispatchC
#undef Hwi_pendSV
#undef Hwi_excHandlerAsm
#undef Hwi_excHandler
#undef Hwi_switchAndRunFunc
#undef Hwi_doSwiRestore
#undef Hwi_doTaskRestore
#undef Hwi_dispatchTable
#undef Hwi_excBusFault
#undef Hwi_excDebugMon
#undef Hwi_excDumpRegs
#undef Hwi_excFillContext
#undef Hwi_excHandlerMax
#undef Hwi_excHandlerMin
#undef Hwi_excHardFault
#undef Hwi_excMemFault
#undef Hwi_excNmi
#undef Hwi_excNoIsr
#undef Hwi_excReserved
#undef Hwi_excSvCall
#undef Hwi_excUsageFault
#undef Hwi_init
#undef Hwi_initStack
#undef Hwi_Module_state
#undef Hwi_Params_default
#undef Hwi_postInit
#undef Hwi_ramVectors
#undef Hwi_swiTaskKeyAddres
#undef Hwi_resetVectors
#undef Hwi_checkStack
#undef Hwi_swiDisableNull
#undef Hwi_swiRestoreNull
#undef Hwi_taskDisableNull
#undef Hwi_taskRestoreHwiNull
#undef Hwi_enable
#undef Hwi_disable
#undef Hwi_restore
#endif /* do_hwi_undef_short_names */

#ifdef do_hwi_short_to_long_name_conversion
#undef do_hwi_short_to_long_name_conversion
#define Hwi_Instance ti_sysbios_family_arm_m3_Hwi_Instance
#define Hwi_Handle ti_sysbios_family_arm_m3_Hwi_Handle
#define Hwi_Module ti_sysbios_family_arm_m3_Hwi_Module
#define Hwi_Object ti_sysbios_family_arm_m3_Hwi_Object
#define Hwi_Struct ti_sysbios_family_arm_m3_Hwi_Struct
#define Hwi_FuncPtr ti_sysbios_family_arm_m3_Hwi_FuncPtr
#define Hwi_Irp ti_sysbios_family_arm_m3_Hwi_Irp
#define Hwi_HookSet ti_sysbios_family_arm_m3_Hwi_HookSet
#define Hwi_MaskingOption ti_sysbios_family_arm_m3_Hwi_MaskingOption
#define Hwi_StackInfo ti_sysbios_family_arm_m3_Hwi_StackInfo
#define Hwi_VectorFuncPtr ti_sysbios_family_arm_m3_Hwi_VectorFuncPtr
#define Hwi_ExceptionHookFuncPtr ti_sysbios_family_arm_m3_Hwi_ExceptionHookFuncPtr
#define Hwi_CCR ti_sysbios_family_arm_m3_Hwi_CCR
#define Hwi_NVIC ti_sysbios_family_arm_m3_Hwi_NVIC
#define Hwi_ExcContext ti_sysbios_family_arm_m3_Hwi_ExcContext
#define Hwi_ExcHandlerFuncPtr ti_sysbios_family_arm_m3_Hwi_ExcHandlerFuncPtr
#define Hwi_HandlerFuncPtr ti_sysbios_family_arm_m3_Hwi_HandlerFuncPtr
#define Hwi_Instance_State ti_sysbios_family_arm_m3_Hwi_Instance_State
#define Hwi_Module_State ti_sysbios_family_arm_m3_Hwi_Module_State
#define Hwi_MaskingOption_NONE ti_sysbios_family_arm_m3_Hwi_MaskingOption_NONE
#define Hwi_MaskingOption_ALL ti_sysbios_family_arm_m3_Hwi_MaskingOption_ALL
#define Hwi_MaskingOption_SELF ti_sysbios_family_arm_m3_Hwi_MaskingOption_SELF
#define Hwi_MaskingOption_BITMASK ti_sysbios_family_arm_m3_Hwi_MaskingOption_BITMASK
#define Hwi_MaskingOption_LOWER ti_sysbios_family_arm_m3_Hwi_MaskingOption_LOWER
#define Hwi_dispatcherAutoNestingSupport ti_sysbios_family_arm_m3_Hwi_dispatcherAutoNestingSupport
#define Hwi_dispatcherSwiSupport ti_sysbios_family_arm_m3_Hwi_dispatcherSwiSupport
#define Hwi_dispatcherTaskSupport ti_sysbios_family_arm_m3_Hwi_dispatcherTaskSupport
#define Hwi_dispatcherIrpTrackingSupport ti_sysbios_family_arm_m3_Hwi_dispatcherIrpTrackingSupport
#define Hwi_NUM_INTERRUPTS ti_sysbios_family_arm_m3_Hwi_NUM_INTERRUPTS
#define Hwi_NUM_PRIORITIES ti_sysbios_family_arm_m3_Hwi_NUM_PRIORITIES
#define Hwi_nullIsrFunc ti_sysbios_family_arm_m3_Hwi_nullIsrFunc
#define Hwi_excHandlerFunc ti_sysbios_family_arm_m3_Hwi_excHandlerFunc
#define Hwi_excHookFunc ti_sysbios_family_arm_m3_Hwi_excHookFunc
#define Hwi_excHookFuncs ti_sysbios_family_arm_m3_Hwi_excHookFuncs
#define Hwi_disablePriority ti_sysbios_family_arm_m3_Hwi_disablePriority
#define Hwi_priGroup ti_sysbios_family_arm_m3_Hwi_priGroup
#define Hwi_numSparseInterrupts ti_sysbios_family_arm_m3_Hwi_numSparseInterrupts
#define Hwi_swiDisable ti_sysbios_family_arm_m3_Hwi_swiDisable
#define Hwi_swiRestore ti_sysbios_family_arm_m3_Hwi_swiRestore
#define Hwi_swiRestoreHwi ti_sysbios_family_arm_m3_Hwi_swiRestoreHwi
#define Hwi_taskDisable ti_sysbios_family_arm_m3_Hwi_taskDisable
#define Hwi_taskRestoreHwi ti_sysbios_family_arm_m3_Hwi_taskRestoreHwi
#define Hwi_ccr ti_sysbios_family_arm_m3_Hwi_ccr
#define Hwi_hooks ti_sysbios_family_arm_m3_Hwi_hooks
#define Hwi_Params ti_sysbios_family_arm_m3_Hwi_Params
#define Hwi_getStackInfo ti_sysbios_family_arm_m3_Hwi_getStackInfo
#define Hwi_getCoreStackInfo ti_sysbios_family_arm_m3_Hwi_getCoreStackInfo
#define Hwi_startup ti_sysbios_family_arm_m3_Hwi_startup
#define Hwi_switchFromBootStack ti_sysbios_family_arm_m3_Hwi_switchFromBootStack
#define Hwi_post ti_sysbios_family_arm_m3_Hwi_post
#define Hwi_getTaskSP ti_sysbios_family_arm_m3_Hwi_getTaskSP
#define Hwi_disableInterrupt ti_sysbios_family_arm_m3_Hwi_disableInterrupt
#define Hwi_enableInterrupt ti_sysbios_family_arm_m3_Hwi_enableInterrupt
#define Hwi_restoreInterrupt ti_sysbios_family_arm_m3_Hwi_restoreInterrupt
#define Hwi_clearInterrupt ti_sysbios_family_arm_m3_Hwi_clearInterrupt
#define Hwi_getFunc ti_sysbios_family_arm_m3_Hwi_getFunc
#define Hwi_setFunc ti_sysbios_family_arm_m3_Hwi_setFunc
#define Hwi_getHookContext ti_sysbios_family_arm_m3_Hwi_getHookContext
#define Hwi_setHookContext ti_sysbios_family_arm_m3_Hwi_setHookContext
#define Hwi_getIrp ti_sysbios_family_arm_m3_Hwi_getIrp
#define Hwi_construct2 ti_sysbios_family_arm_m3_Hwi_construct2
#define Hwi_disableFxn ti_sysbios_family_arm_m3_Hwi_disableFxn
#define Hwi_enableFxn ti_sysbios_family_arm_m3_Hwi_enableFxn
#define Hwi_restoreFxn ti_sysbios_family_arm_m3_Hwi_restoreFxn
#define Hwi_plug ti_sysbios_family_arm_m3_Hwi_plug
#define Hwi_getHandle ti_sysbios_family_arm_m3_Hwi_getHandle
#define Hwi_setPriority ti_sysbios_family_arm_m3_Hwi_setPriority
#define Hwi_excSetBuffers ti_sysbios_family_arm_m3_Hwi_excSetBuffers
#define Hwi_initNVIC ti_sysbios_family_arm_m3_Hwi_initNVIC
#define Hwi_initStacks ti_sysbios_family_arm_m3_Hwi_initStacks
#define Hwi_flushVnvic ti_sysbios_family_arm_m3_Hwi_flushVnvic
#define Hwi_testStaticInlines ti_sysbios_family_arm_m3_Hwi_testStaticInlines
#define Hwi_reconfig ti_sysbios_family_arm_m3_Hwi_reconfig
#define Hwi_Module_name ti_sysbios_family_arm_m3_Hwi_Module_name
#define Hwi_Module_id ti_sysbios_family_arm_m3_Hwi_Module_id
#define Hwi_Module_startup ti_sysbios_family_arm_m3_Hwi_Module_startup
#define Hwi_Module_startupDone ti_sysbios_family_arm_m3_Hwi_Module_startupDone
#define Hwi_construct ti_sysbios_family_arm_m3_Hwi_construct
#define Hwi_create ti_sysbios_family_arm_m3_Hwi_create
#define Hwi_handle ti_sysbios_family_arm_m3_Hwi_handle
#define Hwi_struct ti_sysbios_family_arm_m3_Hwi_struct
#define Hwi_Handle_label ti_sysbios_family_arm_m3_Hwi_Handle_label
#define Hwi_Handle_name ti_sysbios_family_arm_m3_Hwi_Handle_name
#define Hwi_Instance_init ti_sysbios_family_arm_m3_Hwi_Instance_init
#define Hwi_Object_first ti_sysbios_family_arm_m3_Hwi_Object_first
#define Hwi_Object_next ti_sysbios_family_arm_m3_Hwi_Object_next
#define Hwi_Object_sizeof ti_sysbios_family_arm_m3_Hwi_Object_sizeof
#define Hwi_Params_copy ti_sysbios_family_arm_m3_Hwi_Params_copy
#define Hwi_Params_init ti_sysbios_family_arm_m3_Hwi_Params_init
#define Hwi_Instance_finalize ti_sysbios_family_arm_m3_Hwi_Instance_finalize
#define Hwi_delete ti_sysbios_family_arm_m3_Hwi_delete
#define Hwi_destruct ti_sysbios_family_arm_m3_Hwi_destruct
#define Hwi_dispatch ti_sysbios_family_arm_m3_Hwi_dispatch
#define Hwi_dispatchC ti_sysbios_family_arm_m3_Hwi_dispatchC
#define Hwi_pendSV ti_sysbios_family_arm_m3_Hwi_pendSV
#define Hwi_excHandlerAsm ti_sysbios_family_arm_m3_Hwi_excHandlerAsm
#define Hwi_excHandler ti_sysbios_family_arm_m3_Hwi_excHandler
#define Hwi_switchAndRunFunc ti_sysbios_family_arm_m3_Hwi_switchAndRunFunc
#define Hwi_doSwiRestore ti_sysbios_family_arm_m3_Hwi_doSwiRestore
#define Hwi_doTaskRestore ti_sysbios_family_arm_m3_Hwi_doTaskRestore
#define Hwi_dispatchTable ti_sysbios_family_arm_m3_Hwi_dispatchTable
#define Hwi_excBusFault ti_sysbios_family_arm_m3_Hwi_excBusFault
#define Hwi_excDebugMon ti_sysbios_family_arm_m3_Hwi_excDebugMon
#define Hwi_excDumpRegs ti_sysbios_family_arm_m3_Hwi_excDumpRegs
#define Hwi_excFillContext ti_sysbios_family_arm_m3_Hwi_excFillContext
#define Hwi_excHandlerMax ti_sysbios_family_arm_m3_Hwi_excHandlerMax
#define Hwi_excHandlerMin ti_sysbios_family_arm_m3_Hwi_excHandlerMin
#define Hwi_excHardFault ti_sysbios_family_arm_m3_Hwi_excHardFault
#define Hwi_excMemFault ti_sysbios_family_arm_m3_Hwi_excMemFault
#define Hwi_excNmi ti_sysbios_family_arm_m3_Hwi_excNmi
#define Hwi_excNoIsr ti_sysbios_family_arm_m3_Hwi_excNoIsr
#define Hwi_excReserved ti_sysbios_family_arm_m3_Hwi_excReserved
#define Hwi_excSvCall ti_sysbios_family_arm_m3_Hwi_excSvCall
#define Hwi_excUsageFault ti_sysbios_family_arm_m3_Hwi_excUsageFault
#define Hwi_init ti_sysbios_family_arm_m3_Hwi_init
#define Hwi_initStack ti_sysbios_family_arm_m3_Hwi_initStack
#define Hwi_Module_state ti_sysbios_family_arm_m3_Hwi_Module_state
#define Hwi_Params_default ti_sysbios_family_arm_m3_Hwi_Params_default
#define Hwi_postInit ti_sysbios_family_arm_m3_Hwi_postInit
#define Hwi_ramVectors ti_sysbios_family_arm_m3_Hwi_ramVectors
#define Hwi_swiTaskKeyAddress ti_sysbios_family_arm_m3_Hwi_swiTaskKeyAddress
#define Hwi_resetVectors ti_sysbios_family_arm_m3_Hwi_resetVectors
#define Hwi_checkStack ti_sysbios_family_arm_m3_Hwi_checkStack
#define Hwi_swiDisableNull ti_sysbios_family_arm_m3_Hwi_swiDisableNull
#define Hwi_swiRestoreNull ti_sysbios_family_arm_m3_Hwi_swiRestoreNull
#define Hwi_taskDisableNull ti_sysbios_family_arm_m3_Hwi_taskDisableNull
#define Hwi_taskRestoreHwiNull ti_sysbios_family_arm_m3_Hwi_taskRestoreHwiNull
#define Hwi_enable ti_sysbios_family_arm_m3_Hwi_enable
#define Hwi_disable ti_sysbios_family_arm_m3_Hwi_disable
#define Hwi_restore ti_sysbios_family_arm_m3_Hwi_restore
#endif /* do_hwi_short_to_long_name_conversion */

/* @endcond */
