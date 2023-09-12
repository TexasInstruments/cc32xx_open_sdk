/*
 * Copyright (c) 2020, Texas Instruments Incorporated - http://www.ti.com
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
 *
 */

/* @cond LONG_NAMES */

/*
 * ======== HeapCallback_defs.h ========
 */

#ifdef ti_sysbios_heaps_HeapCallback_long_names

#define HeapCallback_Instance ti_sysbios_heaps_HeapCallback_Instance
#define HeapCallback_Handle ti_sysbios_heaps_HeapCallback_Handle
#define HeapCallback_Module ti_sysbios_heaps_HeapCallback_Module
#define HeapCallback_Object ti_sysbios_heaps_HeapCallback_Object
#define HeapCallback_Struct ti_sysbios_heaps_HeapCallback_Struct
#define HeapCallback_AllocInstFxn ti_sysbios_heaps_HeapCallback_AllocInstFxn
#define HeapCallback_CreateInstFxn ti_sysbios_heaps_HeapCallback_CreateInstFxn
#define HeapCallback_DeleteInstFxn ti_sysbios_heaps_HeapCallback_DeleteInstFxn
#define HeapCallback_FreeInstFxn ti_sysbios_heaps_HeapCallback_FreeInstFxn
#define HeapCallback_GetStatsInstFxn ti_sysbios_heaps_HeapCallback_GetStatsInstFxn
#define HeapCallback_InitInstFxn ti_sysbios_heaps_HeapCallback_InitInstFxn
#define HeapCallback_IsBlockingInstFxn ti_sysbios_heaps_HeapCallback_IsBlockingInstFxn
#define HeapCallback_Instance_State ti_sysbios_heaps_HeapCallback_Instance_State
#define HeapCallback_allocInstFxn ti_sysbios_heaps_HeapCallback_allocInstFxn
#define HeapCallback_createInstFxn ti_sysbios_heaps_HeapCallback_createInstFxn
#define HeapCallback_deleteInstFxn ti_sysbios_heaps_HeapCallback_deleteInstFxn
#define HeapCallback_freeInstFxn ti_sysbios_heaps_HeapCallback_freeInstFxn
#define HeapCallback_getStatsInstFxn ti_sysbios_heaps_HeapCallback_getStatsInstFxn
#define HeapCallback_initInstFxn ti_sysbios_heaps_HeapCallback_initInstFxn
#define HeapCallback_isBlockingInstFxn ti_sysbios_heaps_HeapCallback_isBlockingInstFxn
#define HeapCallback_Params ti_sysbios_heaps_HeapCallback_Params
#define HeapCallback_alloc ti_sysbios_heaps_HeapCallback_alloc
#define HeapCallback_free ti_sysbios_heaps_HeapCallback_free
#define HeapCallback_isBlocking ti_sysbios_heaps_HeapCallback_isBlocking
#define HeapCallback_getStats ti_sysbios_heaps_HeapCallback_getStats
#define HeapCallback_getContext ti_sysbios_heaps_HeapCallback_getContext
#define HeapCallback_Module_name ti_sysbios_heaps_HeapCallback_Module_name
#define HeapCallback_Module_id ti_sysbios_heaps_HeapCallback_Module_id
#define HeapCallback_Module_startup ti_sysbios_heaps_HeapCallback_Module_startup
#define HeapCallback_Module_startupDone ti_sysbios_heaps_HeapCallback_Module_startupDone
#define HeapCallback_Module_hasMask ti_sysbios_heaps_HeapCallback_Module_hasMask
#define HeapCallback_Module_getMask ti_sysbios_heaps_HeapCallback_Module_getMask
#define HeapCallback_Module_setMask ti_sysbios_heaps_HeapCallback_Module_setMask
#define HeapCallback_Object_heap ti_sysbios_heaps_HeapCallback_Object_heap
#define HeapCallback_Module_heap ti_sysbios_heaps_HeapCallback_Module_heap
#define HeapCallback_construct ti_sysbios_heaps_HeapCallback_construct
#define HeapCallback_create ti_sysbios_heaps_HeapCallback_create
#define HeapCallback_handle ti_sysbios_heaps_HeapCallback_handle
#define HeapCallback_struct ti_sysbios_heaps_HeapCallback_struct
#define HeapCallback_Handle_label ti_sysbios_heaps_HeapCallback_Handle_label
#define HeapCallback_Handle_name ti_sysbios_heaps_HeapCallback_Handle_name
#define HeapCallback_Instance_init ti_sysbios_heaps_HeapCallback_Instance_init
#define HeapCallback_Object_first ti_sysbios_heaps_HeapCallback_Object_first
#define HeapCallback_Object_next ti_sysbios_heaps_HeapCallback_Object_next
#define HeapCallback_Object_sizeof ti_sysbios_heaps_HeapCallback_Object_sizeof
#define HeapCallback_Params_copy ti_sysbios_heaps_HeapCallback_Params_copy
#define HeapCallback_Params_init ti_sysbios_heaps_HeapCallback_Params_init
#define HeapCallback_Instance_finalize ti_sysbios_heaps_HeapCallback_Instance_finalize
#define HeapCallback_delete ti_sysbios_heaps_HeapCallback_delete
#define HeapCallback_destruct ti_sysbios_heaps_HeapCallback_destruct
#define HeapCallback_Module_upCast ti_sysbios_heaps_HeapCallback_Module_upCast
#define HeapCallback_Module_to_xdc_runtime_IHeap ti_sysbios_heaps_HeapCallback_Module_to_xdc_runtime_IHeap
#define HeapCallback_Handle_upCast ti_sysbios_heaps_HeapCallback_Handle_upCast
#define HeapCallback_Handle_to_xdc_runtime_IHeap ti_sysbios_heaps_HeapCallback_Handle_to_xdc_runtime_IHeap
#define HeapCallback_Handle_downCast ti_sysbios_heaps_HeapCallback_Handle_downCast
#define HeapCallback_Handle_from_xdc_runtime_IHeap ti_sysbios_heaps_HeapCallback_Handle_from_xdc_runtime_IHeap
#define HeapCallback_defaultAlloc ti_sysbios_heaps_HeapCallback_defaultAlloc
#define HeapCallback_defaultCreate ti_sysbios_heaps_HeapCallback_defaultCreate
#define HeapCallback_defaultDelete ti_sysbios_heaps_HeapCallback_defaultDelete
#define HeapCallback_defaultFree ti_sysbios_heaps_HeapCallback_defaultFree
#define HeapCallback_defaultGetStats ti_sysbios_heaps_HeapCallback_defaultGetStats
#define HeapCallback_defaultInit ti_sysbios_heaps_HeapCallback_defaultInit
#define HeapCallback_defaultIsBlocking ti_sysbios_heaps_HeapCallback_defaultIsBlocking
#define HeapCallback_getHeapCallbackFromObjElem ti_sysbios_heaps_HeapCallback_getHeapCallbackFromObjElem
#define HeapCallback_init ti_sysbios_heaps_HeapCallback_init
#define HeapCallback_Module_state ti_sysbios_heaps_HeapCallback_Module_state
#define HeapCallback_Params_default ti_sysbios_heaps_HeapCallback_Params_default

#endif /* ti_sysbios_heaps_HeapCallback_long_names */

#if defined(ti_sysbios_heaps_HeapCallback__nolocalnames) && !defined(ti_sysbios_heaps_HeapCallback_long_names)

#undef HeapCallback_Instance
#undef HeapCallback_Handle
#undef HeapCallback_Module
#undef HeapCallback_Object
#undef HeapCallback_Struct
#undef HeapCallback_AllocInstFxn
#undef HeapCallback_CreateInstFxn
#undef HeapCallback_DeleteInstFxn
#undef HeapCallback_FreeInstFxn
#undef HeapCallback_GetStatsInstFxn
#undef HeapCallback_InitInstFxn
#undef HeapCallback_IsBlockingInstFxn
#undef HeapCallback_Instance_State
#undef HeapCallback_allocInstFxn
#undef HeapCallback_createInstFxn
#undef HeapCallback_deleteInstFxn
#undef HeapCallback_freeInstFxn
#undef HeapCallback_getStatsInstFxn
#undef HeapCallback_initInstFxn
#undef HeapCallback_isBlockingInstFxn
#undef HeapCallback_Params
#undef HeapCallback_alloc
#undef HeapCallback_free
#undef HeapCallback_isBlocking
#undef HeapCallback_getStats
#undef HeapCallback_getContext
#undef HeapCallback_Module_name
#undef HeapCallback_Module_id
#undef HeapCallback_Module_startup
#undef HeapCallback_Module_startupDone
#undef HeapCallback_Module_hasMask
#undef HeapCallback_Module_getMask
#undef HeapCallback_Module_setMask
#undef HeapCallback_Object_heap
#undef HeapCallback_Module_heap
#undef HeapCallback_construct
#undef HeapCallback_create
#undef HeapCallback_handle
#undef HeapCallback_struct
#undef HeapCallback_Handle_label
#undef HeapCallback_Handle_name
#undef HeapCallback_Instance_init
#undef HeapCallback_Object_first
#undef HeapCallback_Object_next
#undef HeapCallback_Object_sizeof
#undef HeapCallback_Params_copy
#undef HeapCallback_Params_init
#undef HeapCallback_Instance_finalize
#undef HeapCallback_delete
#undef HeapCallback_destruct
#undef HeapCallback_Module_upCast
#undef HeapCallback_Module_to_xdc_runtime_IHeap
#undef HeapCallback_Handle_upCast
#undef HeapCallback_Handle_to_xdc_runtime_IHeap
#undef HeapCallback_Handle_downCast
#undef HeapCallback_Handle_from_xdc_runtime_IHeap
#undef HeapCallback_defaultAlloc
#undef HeapCallback_defaultCreate
#undef HeapCallback_defaultDelete
#undef HeapCallback_defaultFree
#undef HeapCallback_defaultGetStats
#undef HeapCallback_defaultInit
#undef HeapCallback_defaultIsBlocking
#undef HeapCallback_getHeapCallbackFromObjElem
#undef HeapCallback_init
#undef HeapCallback_Module_state
#undef HeapCallback_Params_default

#endif

/* @endcond */
