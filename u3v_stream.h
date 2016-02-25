/*
 * u3v_stream.h: Driver for USB3 Vision(TM) class devices
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

#ifndef _U3V_STREAM_H_
#define _U3V_STREAM_H_

#include "u3v.h"
#include "u3v_shared.h"
#include <linux/completion.h>
#include <linux/mutex.h>
#include <linux/rbtree.h>


struct u3v_stream_buffer_config {
	u64 host_image_buffer_size;
	u64 device_image_buffer_size;
	u64 chunk_data_buffer_size;
	u32 max_leader_size;
	u32 max_trailer_size;
	u32 payload_size;
	u32 payload_count;
	u32 transfer1_size;
	u32 transfer2_size;
	u32 transfer2_data_size;
	u32 max_pglist_count;
	u32 alignment_padding;
	u32 segment_padding;
	u64 segment_size;
	bool sg_constraint;
};

struct u3v_stream {
	struct u3v_device *u3v_dev;
	struct mutex stream_lock;
	bool wait_in_progress;
	struct usb_anchor stream_anchor;
	u64 next_buffer_id;
	struct rb_root root;
	struct u3v_stream_buffer_config config;
};

int u3v_create_stream(struct u3v_device *u3v, struct usb_interface *intf,
	struct u3v_configure_stream2 *config_stream, u32 req_leader_size,
	u32 req_trailer_size);

int u3v_destroy_stream(struct u3v_device *u3v);

int u3v_configure_buffer(struct u3v_stream *stream,
	void __user *u_image_buffer, void __user *u_chunk_data_buffer,
	__u64 __user *u_buffer_id);

int u3v_unconfigure_buffer(struct u3v_stream *stream, u64 buffer_id);

int u3v_queue_buffer(struct u3v_stream *stream, u64 buffer_id);

int u3v_wait_for_buffer(struct u3v_stream *stream, u64 buffer_id,
	void __user *u_leader_buffer, __u32 __user *u_leader_size,
	void __user *u_trailer_buffer, __u32 __user *u_trailer_size,
	void __user *u_buffer_complete_data);

int u3v_cancel_all_buffers(struct u3v_stream *stream);
#endif

