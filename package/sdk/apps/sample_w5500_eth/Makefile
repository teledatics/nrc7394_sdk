# NOTE: max firmware binary size is ~934K, compress HTML to save space
APP_DIR := ./sdk/apps/sample_w5500_eth
COMPRESS_HTML := $(shell `lzop -9 -c  $(APP_DIR)/setting.html | base64 - | sed 's/$$/\"/' | sed 's/^/\"/'   >  $(APP_DIR)/setting_html_lzo.h`)

CSRCS += \
	w5500_example_main.c \
	web_service.c \
	wifi_service.c \
	lzo_decompress.c \
	button_action.c

DEFINE += -DSUPPORT_ETHERNET_ACCESSPOINT
DEFINE += -DENABLE_ETHERNET_INTERRUPT
DEFINE += -DNRC_DHCP_SET_BROADCAST_FLAG=1
DEFINE += -DENABLE_RESET_BUTTON=0

include $(SDK_WIFI_COMMON)/module.mk
