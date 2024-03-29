/*
 * u3v_control.h: Driver for USB3 Vision(TM) class devices
 *
 * (C) Copyright 2014 National Instruments Corp.
 * Authors: Katie Ensign <katie.ensign@ni.com>,
 *          Jared Jenson <jared.jenson@ni.com>
 *
 * The "USB3 Vision" name and logo are trademarks of the AIA and may not
 * be used without the authorization of the AIA <www.visiononline.org>
 *
 * Anyone wishing to develop, manufacture, or sell private labeled
 * compliant product for commercial purposes or to develop compliant
 * software for distribution must obtain a license to use the USB3
 * Vision standard and the USB3 Vision name and logo from the AIA.
 * All products (including software) must be registered with the AIA and
 * must be tested for compliancy with the standard.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA  02110-1301, USA.
 */

#ifndef _U3V_CONTROL_H_
#define _U3V_CONTROL_H_

#include <linux/mutex.h>

/* Structs */

struct u3v_control {
	struct u3v_device *u3v_dev;
	struct mutex read_write_lock;
	u8 *ack_buffer;
	u32 max_ack_transfer_size;
	u8 *cmd_buffer;
	u32 max_cmd_transfer_size;
	u16 request_id;
	u16 max_request_id; /* Maximum id value we can have before we loop back around */
	u32 u3v_timeout; /* Maximum device response time in ms */
};

/* Functions */

int u3v_create_control(struct u3v_device *u3v);

int u3v_read_memory(struct u3v_control *ctrl, u32 transfer_size,
	u32 *bytes_read, u64 address, void *buffer);

int u3v_write_memory(struct u3v_control *ctrl, u32 transfer_size,
	u32 *bytes_written, u64 address, const void *buffer);

int u3v_control_msg(struct usb_device *udev, u8 request, u8 requesttype,
	u16 value, u16 index, void __user *u_data, u16 size);

void u3v_destroy_control(struct u3v_device *u3v);

#endif
