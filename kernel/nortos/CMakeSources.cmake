cmake_minimum_required(VERSION 3.21.3)

set(SOURCES_COMMON
    NoRTOS.c
    dpl/posix_sleep.c
    dpl/DebugP_nortos.c
    dpl/MutexP_nortos.c
    dpl/SemaphoreP_nortos.c
    dpl/SwiP_nortos.c
    dpl/SystemP_nortos.c
)

set(SOURCES_CC32XX ${SOURCES_COMMON} dpl/ClockPSysTick_nortos.c dpl/HwiPCC32XX_nortos.c dpl/PowerCC32XX_nortos.c
                   startup/startup_cc32xx_${TI_TOOLCHAIN_NAME}.c dpl/TimestampPCC32XX_nortos.c
)

set(SOURCES_CC13XX_CC26XX
    ${SOURCES_COMMON}
    dpl/ClockPTimer_nortos.c
    dpl/QueueP_nortos.c
    dpl/HwiPCC26XX_nortos.c
    dpl/PowerCC26X2_nortos.c
    dpl/TimerPCC26XX_nortos.c
    dpl/TimestampPCC26XX_nortos.c
)

set(SOURCES_CC13X2_CC26X2 ${SOURCES_CC13XX_CC26XX} startup/startup_cc13x2_cc26x2_${TI_TOOLCHAIN_NAME}.c)

set(SOURCES_CC13X1_CC26X1 ${SOURCES_CC13XX_CC26XX} startup/startup_cc13x1_cc26x1_${TI_TOOLCHAIN_NAME}.c)

set(SOURCES_CC13X4_CC26X4 ${SOURCES_CC13XX_CC26XX} startup/startup_cc13x4_cc26x4_${TI_TOOLCHAIN_NAME}.c)

set(SOURCES_CC13X4_CC26X4_NS ${SOURCES_CC13XX_CC26XX} startup/startup_cc13x4_cc26x4_${TI_TOOLCHAIN_NAME}_ns.c)

set(SOURCES_CC23XX ${SOURCES_COMMON} dpl/ClockPCC23XX_nortos.c dpl/HwiPCC23XX_nortos.c dpl/TimestampPCC23XX_nortos.c)

set(SOURCES_CC23X0 ${SOURCES_CC23XX} dpl/PowerCC23X0_nortos.c startup/startup_cc23x0_${TI_TOOLCHAIN_NAME}.c
                   dpl/QueueP_nortos.c
)

set(SOURCES_CC23X0R2 ${SOURCES_CC23XX} dpl/PowerCC23X0R2_nortos.c startup/startup_cc23x0r2_${TI_TOOLCHAIN_NAME}.c
                     dpl/QueueP_nortos.c
)
