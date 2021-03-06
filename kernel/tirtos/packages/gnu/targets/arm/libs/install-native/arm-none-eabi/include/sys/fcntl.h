#ifndef _SYS_FCNTL_H_
#define _SYS_FCNTL_H_

#include <sys/_default_fcntl.h>

#if defined (__ARM_ARCH_7A__) || defined(__ARM_ARCH_8A) || defined(__ARM_ARCH_8A__)
/* We want to support O_BINARY for the open syscall.
   For example, the Demon debug monitor has a separate
   flag value for "rb" vs "r". */
#define _FBINARY        0x10000
#define O_BINARY        _FBINARY
#endif

#endif /* _SYS_FCNTL_H_ */
