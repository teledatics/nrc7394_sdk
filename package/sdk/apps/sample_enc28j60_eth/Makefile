# NOTE: max firmware binary size is ~934K, compress HTML to save space
APP_DIR := ./sdk/apps/sample_enc28j60_eth
COMPRESS_HTML := $(shell `lzop -9 -c  $(APP_DIR)/setting.html | base64 - | sed 's/$$/\"/' | sed 's/^/\"/'   >  $(APP_DIR)/setting_html_lzo.h`)

CSRCS += \
	enc28j60_example_main.c \
	web_service.c \
	wifi_service.c \
	lzo_decompress.c \

DEFINE += -DSUPPORT_ETHERNET_ACCESSPOINT
DEFINE += -DENABLE_ETHERNET_INTERRUPT
DEFINE += -DNRC_DHCP_SET_BROADCAST_FLAG
DEFINE += -DSUPPORT_USE_4ADDRESS
DEFINE += -DETHERNET_SPI_DMA
DEFINE += -DNRC7394_DMA_MEMPOOL
include $(SDK_WIFI_COMMON)/module.mk

CONFIG_FLASH_SIZE = 2M
