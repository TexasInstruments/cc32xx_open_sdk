#
# Set location of various cgtools
#
# These variables can be set here or on the command line.
#
# The various *_ARMCOMPILER variables, in addition to pointing to
# their respective locations, also serve as "switches" for disabling a build
# using those cgtools. To disable a build using a specific cgtool, either set
# the cgtool's variable to empty or delete/comment-out its definition:
#     TICLANG_ARMCOMPILER ?=
# or
#     #TICLANG_ARMCOMPILER ?= ...
#
# If a cgtool's *_ARMCOMPILER variable is set (non-empty), various sub-makes
# in the installation will attempt to build with that cgtool.  This means
# that if multiple *_ARMCOMPILER cgtool variables are set, the sub-makes
# will build using each non-empty *_ARMCOMPILER cgtool.
#

XDC_INSTALL_DIR        ?= C:/ti/xdctools_3_62_01_15_core
SYSCONFIG_TOOL         ?= C:/ti/ccs1220/ccs/utils/sysconfig_1.15.0/sysconfig_cli.bat

FREERTOS_INSTALL_DIR   ?= C:/FreeRTOSv202104.00
CMAKE                  ?= C:/cmake-3.21.3/bin/cmake
PYTHON                 ?= python

TICLANG_ARMCOMPILER    ?= C:/ti/ccs1220/ccs/tools/compiler/ti-cgt-armllvm_2.1.2.LTS
GCC_ARMCOMPILER        ?= C:/ti/ccs1220/ccs/tools/compiler/9.2019.q4.major-win32
IAR_ARMCOMPILER        ?= C:/Program Files/IAR Systems/Embedded Workbench 9.1/arm
##TFM_BUILD_VAR##
ifeq ("$(SHELL)","sh.exe")
# for Windows/DOS shell
    RM      = del
    RMDIR   = -rmdir /S /Q
    DEVNULL = NUL
    ECHOBLANKLINE = @cmd /c echo.
else
# for Linux-like shells
    RM      = rm -f
    RMDIR   = rm -rf
    DEVNULL = /dev/null
    ECHOBLANKLINE = echo
endif
