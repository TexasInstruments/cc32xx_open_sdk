# Bring in common target-independent definitions
include ../../../defs.mak

# Enable local overrides via a top-directory 'products.mak'
-include ../../../products.mak

CC = "$(GCC_ARMCOMPILER)/bin/arm-none-eabi-gcc"
AR = "$(GCC_ARMCOMPILER)/bin/arm-none-eabi-gcc-ar"

STDINC = "$(GCC_ARMCOMPILER)/include"
#
# Include path STDINC
#

IPATH += $(POSIX_INCS_GCC)
IPATH += -I$(STDINC)

#TIKERNELLIBINC = "$(SDK_INSTALL_DIR)/kernel/tirtos/packages/gnu/targets/arm/libs/install-native/arm-none-eabi/include/newlib-nano"
#TIKERNELINC = "$(SDK_INSTALL_DIR)/kernel/tirtos/packages/gnu/targets/arm/libs/install-native/arm-none-eabi/include"
#IPATH += -I$(KERNELINC)
#IPATH += -I$(TIKERNELLIBINC)

#STDINC = "$(GCC_ARMCOMPILER)/arm-none-eabi/include"

#PATH += -I$(STDINC)





CFLAGS = -O2 -std=c99 -c -Wunused -Wunknown-pragmas -ffunction-sections -fdata-sections -mcpu=cortex-m4 -mthumb -mabi=aapcs -mfloat-abi=soft $(IPATH) 
#	-Wformat-overflow=2 -Wformat-truncation 

#	-march=armv7e-m  -std=c99 -mfloat-abi=soft \
#	 -g  -gstrict-dwarf -Wall --specs=nano.specs
#CFLAGS = -O2 -std=c99 -c -Wunused -Wunknown-pragmas -ffunction-sections -fdata-sections -mcpu=cortex-m4 -mthumb -mabi=aapcs $(IPATH)

ARFLAGS = -cr

LIB_OBJ_DIR = lib
L_OBJS = $(addprefix $(LIB_OBJ_DIR)/,$(LIB_OBJS))

PORT_OBJ_DIR = port
P_OBJS = $(addprefix $(PORT_OBJ_DIR)/,$(PORT_OBJS))

vpath %.c ../../../../library ../../../port

$(LIB_OBJ_DIR)/%.o: %.c | $(LIB_OBJ_DIR)
	@ echo "#"
	@ echo "# Building $@"
	$(CC) $(CFLAGS) $< -o $@

$(PORT_OBJ_DIR)/%.o: %.c | $(PORT_OBJ_DIR)
	@ echo "#"
	@ echo "# Building $@"
	$(CC)  $(CFLAGS) $< -o $@

#$(POSIX_INCS_GCC)
# To enable a wide set of users, only the hardware-independent library
# is built from the default "all" build goal.  
all: mbedtls.a

$(LIB_OBJ_DIR):
	mkdir $@

$(PORT_OBJ_DIR):
	mkdir $@

clean:
	@ $(RMDIR) $(LIB_OBJ_DIR) $(PORT_OBJ_DIR) 
	@ $(RM) *.a

mbedtls.a: $(L_OBJS) $(P_OBJS)
	@ echo "#"
	@ echo "# Building $@"
	$(AR) $(ARFLAGS) $@ $^


