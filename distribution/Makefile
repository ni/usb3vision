################################################################################
#
#   Makefile: Linux driver for USB3 Vision(TM) Devices.
#
#   (C) Copyright 2014 National Instruments Corp.
#   Authors: Katie Ensign <katie.ensign@ni.com>,
#            Jared Jenson <jared.jenson@ni.com>
#
#   The "USB3 Vision" name and logo are trademarks of the AIA and may not
#   be used without the authorization of the AIA <www.visiononline.org>
#
#   Anyone wishing to develop, manufacture, or sell private labeled
#   compliant product for commercial purposes or to develop compliant
#   software for distribution must obtain a license to use the USB3
#   Vision standard and the USB3 Vision name and logo from the AIA.
#   All products (including software) must be registered with the AIA and
#   must be tested for compliancy with the standard.
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.
#
#   You should have received a copy of the GNU General Public License
#   along with this program; if not, write to the Free Software
#   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
#
################################################################################

# If KERNELRELEASE is defined, we've been invoked from the
# kernel build system and can use its language. (LDD3, ch 2, pg 24)
ifneq ($(KERNELRELEASE),)

# if version_compatibility is defined, add a cflag to include the right
# header files
ifneq ($(version_compatibility),)
	EXTRA_CFLAGS += -DVERSION_COMPATIBILITY
endif

obj-m := u3v.o
u3v-objs := u3v_core.o u3v_control.o u3v_event.o u3v_stream.o

else

PWD := $(shell pwd)

MOD_DIR := kernel/natinst/u3v
MOD_PATH := /lib/modules/$(shell uname -r)/$(MOD_DIR)

all:
	@$(MAKE) --no-print-directory -C $(KERNELHEADERS) M=$(PWD) modules

debug: all
	EXTRA_CFLAGS += -DDEBUG -g

install: all
	@$(MAKE) --no-print-directory -C $(KERNELHEADERS) M=$(PWD) INSTALL_MOD_DIR=$(MOD_DIR) modules_install

uninstall:
	@$(RM) -rf $(MOD_PATH)
	@/sbin/depmod -a

clean:
	@$(MAKE) --no-print-directory -s -C $(KERNELHEADERS) M=$(PWD) clean
	@$(RM) -rf Module.*

.PHONY: all install uninstall clean

endif
