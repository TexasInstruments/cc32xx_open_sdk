# Bring in common target-independent definitions
include ../../../defs.mak

# Enable local overrides via a top-directory 'products.mak'
-include ../../../products.mak

CC = "$(IAR_ARMCOMPILER)/bin/iccarm"
ASM = "$(IAR_ARMCOMPILER)/bin/iasmarm"
AR = "$(IAR_ARMCOMPILER)/bin/iarchive"

STDINC = "$(IAR_ARMCOMPILER)/include"

#
# Include path
#
IPATH += $(POSIX_INCS_IAR)
IPATH += -I$(STDINC)

CFLAGS = -Dewarm  --debug --silent --aeabi --cpu=Cortex-M4 --diag_suppress=Pa050,Go005 --endian=little -e --thumb $(IPATH)

#
# Creating lib and port oject directories
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
	$(CC) $(CFLAGS) $< -o $@

all: mbedtls.a

$(LIB_OBJ_DIR):
	mkdir $@

$(PORT_OBJ_DIR):
	mkdir $@

mbedtls.a: $(L_OBJS) $(P_OBJS)
	@ echo "#"
	@ echo "# Building $@"
	$(AR) --silent -o $@ $^

clean:
	@ $(RMDIR) $(LIB_OBJ_DIR) $(PORT_OBJ_DIR) 
	@ $(RM) *.a


