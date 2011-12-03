EXTRA_CFLAGS += -Dlinux -DLINUX -DBDC -DBCMDRIVER -DBCMDONGLEHOST -DDHDTHREAD -DBCMWPA2 \
    -DDHD_GPL -DDHD_SCHED -DBCMSDIO -DBCMLXSDMMC -DBCMPLATFORM_BUS -DDHD_BCMEVENTS      \
    -DSHOW_EVENTS -DANDROID_SPECIFIC -DENABLE_DEEP_SLEEP

################ OPTIONAL FEATURES ############################################################
#TO ENALBLE OPTIONAL FEATURES UNCOMMENT THE CORRESPONDING FLAGS

# For Samsung Aries only.
EXTRA_CFLAGS += -DCUSTOMER_HW_SAMSUNG

# For MACID management
EXTRA_CFLAGS += -DREAD_MACADDR

# For 5G scan home channel time. (default=45 -> 60)
#EXTRA_CFLAGS += -DSCAN_5G_HOMECHANNEL_TIME

# For AUTH time patch
EXTRA_CFLAGS += -DAUTH_TIME_PATCH

# Check if this is required. SDIO_ISR_THREAD is mutually exclusive with OOB_INTR_ONLY
#EXTRA_CFLAGS += -DSDIO_ISR_THREAD

# For OOB only
EXTRA_CFLAGS += -DOOB_INTR_ONLY

# For HW_OOB
#EXTRA_CFLAGS += -DHW_OOB

# FOR SOFTAP
EXTRA_CFLAGS += -DSOFTAP

# For Debug
#EXTRA_CFLAGS += -DDHD_DEBUG -DSRCBASE=\"$(src)/src\"

# HOST WAKEUP
EXTRA_CFLAGS += -DBCM_HOSTWAKE

#STATIC MEMORY ALLOCATION FEATURE
EXTRA_CFLAGS += -DDHD_USE_STATIC_BUF

EXTRA_CFLAGS += -DEMBEDDED_PLATFORM

EXTRA_CFLAGS += -DARP_OFFLOAD_SUPPORT
EXTRA_CFLAGS += -DPKT_FILTER_SUPPORT
#Disable TPC feature for FCC 
#EXTRA_CFLAGS += -DFCC_CERT

# Scan Method - ESCAN
#EXTRA_CFLAGS += -DWL_IW_USE_ISCAN
EXTRA_CFLAGS += -DWL_IW_USE_ESCAN
#Disable PowerSave mode for OTA or certification test
#EXTRA_CFLAGS += -DBCMDISABLE_PM
EXTRA_CFLAGS += -DCONFIG_CONTROL_PM

###############################################################################################

ifeq ($(CONFIG_BCM_EMBED_FW),y)
	EXTRA_CFLAGS		+= -I$(src)/src/dongle/rte/wl/builds/4325b0/sdio-g-cdc-reclaim-wme/
	EXTRA_CFLAGS		+= -DBCMEMBEDIMAGE
	EXTRA_CFLAGS		+= -DIMAGE_NAME="4325b0/sdio-g-cdc-reclaim-wme"
endif

EXTRA_CFLAGS += -I$(src)/include/
EXTRA_CFLAGS += -I$(src)
EXTRA_LDFLAGS += --strip-debug

KBUILD_CFLAGS += -I$(LINUXDIR)/include -I$(shell pwd)


obj-$(CONFIG_BCM4329) += bcm4329.o hotspot_event_monitoring.o
bcm4329-objs := dhd_linux.o \
         dhd_common.o \
         dhd_cdc.o \
         dhd_linux_sched.o\
         dhd_sdio.o \
         dhd_custom_gpio.o \
         aiutils.o \
         bcmutils.o \
         bcmwifi.o \
         hndpmu.o \
         linux_osl.o \
         sbutils.o \
         siutils.o \
         bcmsdh_sdmmc.o \
         bcmsdh.o \
         bcmsdh_linux.o \
         bcmsdh_sdmmc_linux.o \
         wl_iw.o


ifeq ($(CONFIG_TARGET_LOCALE_KOR),y)
bcm4329-objs := $(bcm4329-objs:dhd_common.o=dhd_common_skt.o)
bcm4329-objs := $(bcm4329-objs:wl_iw.o=wl_iw_skt.o)
endif

all:
	@echo "$(MAKE) --no-print-directory -C $(KDIR) SUBDIRS=$(CURDIR) modules"
	@$(MAKE) --no-print-directory -C $(KDIR) \
		SUBDIRS=$(CURDIR) modules

clean: 
	rm -rf *.o *.ko *.mod.c *~ .*.cmd Module.symvers modules.order .tmp_versions

install:
	@$(MAKE) --no-print-directory -C $(KDIR) \
		SUBDIRS=$(CURDIR) modules_install
