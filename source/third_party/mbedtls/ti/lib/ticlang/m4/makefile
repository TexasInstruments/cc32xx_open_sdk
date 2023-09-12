# Bring in common target-independent definitions
include ../../../defs.mak

# Enable local overrides via a top-directory 'products.mak'
-include ../../../products.mak

CC = "$(TICLANG_ARMCOMPILER)/bin/tiarmclang"
AR = "$(TICLANG_ARMCOMPILER)/bin/tiarmar"

#CFLAGS = -c $(INCS) "-I$(TICLANG_ARMCOMPILER)/include/armv7em-ti-none-eabi" \
#        -mcpu=cortex-m4 -mfloat-abi=soft -mfpu=none -mlittle-endian -mthumb -Oz \
#         -DDeviceFamily_CC3220 -gdwarf-3 -march=armv7e-m 


#
# Include path
#
IPATH += $(POSIX_INCS_TICLANG)

CFLAGS = -c -Wunused -Wunknown-pragmas -ffunction-sections -fdata-sections -mcpu=cortex-m4 -mthumb -mabi=aapcs  -std=c99 -mfloat-abi=soft $(IPATH)

ARFLAGS = -cr

OBJ_DIR = default
OBJS = $(addprefix $(OBJ_DIR)/,$(LIB_OBJS)) $(addprefix $(OBJ_DIR)/,$(PORT_OBJS))

vpath %.c ../../../../library ../../../port

$(OBJ_DIR)/%.o: %.c | $(OBJ_DIR)
	@ echo "#"
	@ echo "# Building $@"
	$(CC) $(POSIX_INCS_TICLANG) $(CFLAGS) $< -o $@


all: mbedtls.a

$(OBJ_DIR):
	mkdir $@

clean:
	@ $(RMDIR) $(OBJ_DIR)
	@ $(RM) *.a

mbedtls.a: $(OBJS)
	@ echo "#"
	@ echo "# Building $@"
	$(AR) $(ARFLAGS) $@ $^

debug:
	@ echo "### make sure all of these are correct: ### CC path =  $(CC), ### POSIX_INCS_TICLANG path = $(POSIX_INCS_TICLANG) ###" 
