# drivers/media/video/samsung/g2d/Makefile
#
# Copyright (c) 2010 Samsung Electronics Co., Ltd.
# 		http://www.samsung.com/
#
# Licensed under GPLv2

obj-y				:=
obj-m				:=
obj-n				:=
obj-				:=

obj-$(CONFIG_VIDEO_G2D) += fimg2d_3x.o

ifeq ($(CONFIG_VIDEO_G2D_DEBUG),y)
EXTRA_CFLAGS += -DDEBUG
endif
