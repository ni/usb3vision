/*
 * u3v_event.h: Driver for USB3 Vision(TM) class devices
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

#ifndef _U3V_EVENT_H_
#define _U3V_EVENT_H_

#include "u3v.h"
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/usb.h>

#define U3V_EVENT_PREFIX 0x45563355

struct u3v_event {
	struct u3v_device *u3v_dev;
	struct mutex event_lock;
	bool started;
	bool wait_in_progress;
	struct list_head event_queue;
	int queue_depth;
	struct completion event_complete;
	u64 overwritten_event_count;
	int order; /* number of pages needed for buffers */
	struct usb_anchor event_anchor;
};

int u3v_create_events(struct u3v_device *u3v, struct usb_interface *intf,
	u32 event_max_transfer, u32 event_queue_depth);

int u3v_start_events(struct u3v_event *event);

int u3v_wait_for_event(struct u3v_event *event, void __user *u_buffer,
	__u32 __user *u_buffer_size, void __user *u_event_complete_buffer);

void u3v_stop_events(struct u3v_event *event);

void u3v_destroy_events(struct u3v_device *u3v);

#endif

