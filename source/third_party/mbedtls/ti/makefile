# This makefile builds all TI libraries it finds.  To build a specific
# library, run the appropriate lib/<toolchain>/<isa>/makefile


.PHONY: all $(SUBDIRS) clean $(SUBCLEAN) debug

include $ ../../../../imports.mak

# Find subdirectories
RELEASE-TICLANG-SUBDIRS = $(wildcard ./lib/ticlang/m4/.)
RELEASE-GCC-SUBDIRS = $(wildcard ./lib/gcc/m4/.)
RELEASE-IAR-SUBDIRS = $(wildcard ./lib/iar/m4/.)

# Locate all subdirectories to build
SUBDIRS =
ifneq ($(TICLANG_ARMCOMPILER),)
	SUBDIRS += $(RELEASE-TICLANG-SUBDIRS)
endif
ifneq ($(GCC_ARMCOMPILER),)
	SUBDIRS += $(RELEASE-GCC-SUBDIRS)
endif
ifneq ($(IAR_ARMCOMPILER),)
	SUBDIRS += $(RELEASE-IAR-SUBDIRS)
endif

#@echo SUBDIRS

# phony '<subdir>.clean' build goals (used during clean)
SUBCLEAN := $(addsuffix .clean,$(SUBDIRS))

all: $(addsuffix /mbedtls.a,$(SUBDIRS))

%/mbedtls.a: %/makefile
	$(MAKE) -C $*

clean: $(SUBCLEAN)

$(SUBCLEAN): %.clean:
	$(MAKE) -C $* clean
