# Copyright (c) 2022, Texas Instruments Incorporated
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# *  Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# *  Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# *  Neither the name of Texas Instruments Incorporated nor the names of
#    its contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
# OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
cmake_minimum_required(VERSION ${TI_MIN_CMAKE_VERSION})
project(driverlib_cc32xx LANGUAGES)
get_install_dir(DRIVERLIB_CC32XX_INSTALL_DIR)

set(ARCH_cc32xx m4)
list(APPEND SUPPORTED_PLATFORMS cc32xx)
list(REMOVE_DUPLICATES SUPPORTED_PLATFORMS)
list(APPEND SUPPORTED_ARCHITECTURES m4)
list(REMOVE_DUPLICATES SUPPORTED_ARCHITECTURES)
list(APPEND SUPPORTED_COMPONENTS driverlib_cc32xx)
list(REMOVE_DUPLICATES SUPPORTED_COMPONENTS)

# Assume nobody is actively misbehaving and defining partial target sets
if (NOT TARGET device_cc32xx)
    add_library(device_cc32xx INTERFACE IMPORTED)
    add_library(Device::cc32xx ALIAS device_cc32xx)
    target_compile_definitions(device_cc32xx INTERFACE DeviceFamily_CC3220)

    add_library(driverlib_cc32xx INTERFACE IMPORTED)
    add_library(Driverlib::cc32xx ALIAS driverlib_cc32xx)
    target_link_libraries(
        driverlib_cc32xx
        INTERFACE
            Device::cc32xx
            ${DRIVERLIB_CC32XX_INSTALL_DIR}/source/ti/devices/cc32xx/driverlib/${TI_TOOLCHAIN_NAME}/Release/driverlib.a
    )
    target_include_directories(
        driverlib_cc32xx
            INTERFACE ${DRIVERLIB_CC32XX_INSTALL_DIR}/source
                      ${DRIVERLIB_CC32XX_INSTALL_DIR}/source/ti/devices/cc32xx
    )
endif ()