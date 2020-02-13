BOARD ?= samr21-xpro
include ../Makefile.tests_common

FEATURES_REQUIRED = periph_i2c
FEATURES_REQUIRED = periph_spi
FEATURES_REQUIRED = soft_spi

USEMODULE += periph_i2c
USEMODULE += periph_spi
USEMODULE += soft_spi
USEMODULE += shell

CFLAGS += -DSOFT_SPI_NUMOF=1

include $(RIOTBASE)/Makefile.include
