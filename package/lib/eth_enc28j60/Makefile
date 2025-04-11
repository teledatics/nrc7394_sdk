include $(CONFIG)

PKG_NAME=enc28j60

include eth_enj28j60.make

ETH_ENC28J60_BASE	= $(CURDIR)

INCLUDE	+= -I$(ETH_ENC28J60_BASE)

INCLUDE += -I$(TOPDIR)/lib/ethernet
INCLUDE += -I$(TOPDIR)/sdk/include

VPATH	+= $(ETH_ENC28J60_BASE)

include $(TOPDIR)/make/build-lib.mk
