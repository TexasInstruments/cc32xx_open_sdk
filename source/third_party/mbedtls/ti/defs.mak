#
# By default, look for an SDK-provided imports.mak file to provide
# dependency locations (e.g. toolchains).
#
# Note, this SDK's imports.mak inclusion is optional (the leading '-'
# when including).  If exists, we'll use variables in it, else the
# user will have to set these variables themselves.  The variables
# this build system may use, depending on what you're trying to build, include:
#    CCS_ARMCOMPILER     - CCS ARM toolchain
#    IAR_ARMCOMPILER     - IAR ARM toolchain
#    GCC_ARMCOMPILER     - GCC ARM toolchain
#    GCC_ARM64COMPILER   - GCC 64-bit ARM toolchain
#    CCS_C6XCOMPILER     - CCS C6x toolchain
#    TICLANG_ARMCOMPILER - TI ARM CLANG toolchain
#    RM                  - delete a file in an OS-independent way
#    RMDIR               - delete a directory in an OS-independent way
#
# Note that this SDK_INSTALL_DIR path is relative to the
# makefiles including this defs.mak, which are 3 directories deeper
# (lib/<toolchain>/<isa>) than this file.
SDK_INSTALL_DIR ?= $(abspath ../../../../../../..)

include $(SDK_INSTALL_DIR)/imports.mak

# Default POSIX is in the SDK's source/ directory, but users can
# override this
POSIX_ROOT = $(SDK_INSTALL_DIR)/source

ROOT = $(SDK_INSTALL_DIR)/source/third_party/mbedtls


# To build MSP432E4-specific hardware-accelerated libraries, uncomment
# this variable assignment and ensure it is set to your MSP432E4
# SimpleLink SDK
#MSP432E4_SDK_INSTALL_DIR = $(SDK_INSTALL_DIR)

OBJS_CRYPTO= \
   threading.o \
   platform.o \
   platform_util.o \
   aes.o \
   aesni.o \
   aria.o \
   asn1parse.o \
   asn1write.o \
   base64.o \
   bignum.o \
   bignum_core.o \
   camellia.o \
   ccm.o \
   chacha20.o \
   chachapoly.o \
   cipher.o \
   cipher_wrap.o \
   constant_time.o \
   cmac.o \
   ctr_drbg.o \
   des.o \
   dhm.o \
   ecdh.o \
   ecdsa.o \
   ecjpake.o \
   ecp.o \
   ecp_curves.o \
   entropy.o \
   entropy_poll.o \
   error.o \
   gcm.o \
   hash_info.o \
   hkdf.o \
   hmac_drbg.o \
   md.o \
   md5.o \
   memory_buffer_alloc.o \
   mps_reader.o \
   mps_trace.o \
   nist_kw.o \
   oid.o \
   padlock.o \
   pem.o \
   pk.o \
   pk_wrap.o \
   pkcs12.o \
   pkcs5.o \
   pkparse.o \
   pkwrite.o \
   poly1305.o \
   psa_crypto.o \
   psa_crypto_aead.o \
   psa_crypto_cipher.o \
   psa_crypto_client.o \
   psa_crypto_driver_wrappers.o \
   psa_crypto_ecp.o \
   psa_crypto_hash.o \
   psa_crypto_mac.o \
   psa_crypto_rsa.o \
   psa_crypto_se.o \
   psa_crypto_slot_management.o \
   psa_crypto_storage.o \
   psa_its_file.o \
   ripemd160.o \
   rsa.o \
   rsa_alt_helpers.o \
   sha1.o \
   sha256.o \
   sha512.o \
   ssl_debug_helpers_generated.o \
   timing.o \
   version.o \
   version_features.o \

   # This line is intentionally left blank

OBJS_X509= \
   x509.o \
   x509_create.o \
   x509_crl.o \
   x509_crt.o \
   x509_csr.o \
   x509write_crt.o \
   x509write_csr.o \
   # This line is intentionally left blank

OBJS_TLS= \
  debug.o \
  net_sockets.o \
  ssl_cache.o \
  ssl_ciphersuites.o \
  ssl_client.o \
  ssl_cookie.o \
  ssl_msg.o \
  ssl_ticket.o \
  ssl_tls.o \
  ssl_tls12_client.o\
  ssl_tls12_server.o\
  ssl_tls13_keys.o \
  ssl_tls13_client.o \
  ssl_tls13_server.o \
  ssl_tls13_generic.o \
  # This line is intentionally left blank

LIB_OBJS= $(OBJS_CRYPTO) $(OBJS_X509) $(OBJS_TLS) 
PORT_OBJS = entropy_alt.o threading_alt.o

#
# Include path
#
IPATH  = -I$(ROOT)
IPATH += -I$(ROOT)/include
IPATH += -I$(ROOT)/library
IPATH += -I$(ROOT)/ti/port
IPATH += -I$(SDK_INSTALL_DIR)/source

#INCS = -I../../../../include -I../../../../library  -I../../../../ti/port \
        -I../../../../ti/configs -I$(POSIX_ROOT)


POSIX_INCS_CCS = -I$(POSIX_ROOT)/ti/posix/ccs
POSIX_INCS_IAR = -I$(POSIX_ROOT)/ti/posix/iar
POSIX_INCS_GCC = -I$(POSIX_ROOT)/ti/posix/gcc
POSIX_INCS_TICLANG = -I$(POSIX_ROOT)/ti/posix/ticlang
