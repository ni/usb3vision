/*
 * u3v_stream.c: Driver for USB3 Vision(TM) class devices
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

#include "u3v.h"
#include "u3v_shared.h"
#include "u3v_stream.h"
#include "u3v_control.h"
#include <linux/atomic.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/highmem.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/pagemap.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/usb.h>
#include <linux/vmalloc.h>
#include <asm/unaligned.h>

#ifdef VERSION_COMPATIBILITY
	#include <linux/version.h>
#else
	#include <generated/uapi/linux/version.h>
#endif


/* Internal stream structs and enums */

struct urb_info {
	u32 urb_index;
	struct u3v_stream *stream;
	struct buffer_entry *entry;
	struct urb *purb;
	bool kernel_allocated;
	struct pg_list *pglist;
	u8 *buffer;
	size_t buffer_size;
	size_t expected_size;
};

enum buffer_state {
	u3v_idle, u3v_queued, u3v_complete, u3v_cancelled
};

struct buffer_entry {
	atomic_t callbacks_received;
	enum buffer_state state;
	struct urb_info *urb_info_array;
	int urb_info_count;
	u32 incomplete_callbacks_received;
	u32 leader_received_size;
	u32 payload_received_size;
	u32 trailer_received_size;
	void *transfer2_addr;
	u32 transfer2_received_size;
	struct completion buffer_complete;
	u64 buffer_id;
	struct rb_node node;
	s32 status;
};

struct pg_list {
	size_t num_bytes;
	size_t num_pages;
	size_t offset;
	struct page **pages;
	struct sg_table *sgt;
};

/*
 * Inline helper function that returns the number of pages in a num_bytes
 * sized chunk of memory starting at addr.
 */
static inline int get_num_pages(uintptr_t addr, size_t num_bytes)
{
	unsigned long end = (addr + num_bytes + PAGE_SIZE - 1) >> PAGE_SHIFT;
	unsigned long start = addr >> PAGE_SHIFT;
	return end - start;
}

/* Forward declaration for internal functions */
/* Helper function for u3v_create_stream */
static int set_buffer_sizes(struct u3v_stream *stream, u64 image_buffer_size,
	u64 chunk_data_buffer_size, u32 max_leader_size,
	u32 max_trailer_size, u32 payload_size, u32 payload_count,
	u32 transfer1_size, u32 transfer2_size);
/* Helper functions for u3v_configure/unconfigure_buffer */
static int create_buffer_entry(struct u3v_stream *stream,
	void *user_image_buffer,
	void *user_chunk_data_buffer,
	__u64 *buffer_id);
static int allocate_urb_buffer(struct u3v_stream *stream,
	struct urb_info *urb_info, size_t size,
	size_t expected_size);
static int map_urb_buffer(struct u3v_stream *stream,
	struct urb_info *urb_info, void *user_buffer,
	size_t buffer_size, size_t offset,
	size_t urb_buffer_size, size_t expected_size);
static int destroy_buffer(struct u3v_stream *stream, __u64 buffer_id);
static void destroy_urb(struct urb_info *urb_info);
static struct pg_list *create_pg_list(struct u3v_stream *stream,
	void *user_buffer, size_t num_bytes);
static void destroy_pg_list(struct pg_list *pglist, bool dirty);
static struct page **lock_user_pages(struct u3v_stream *stream,
	uintptr_t uaddr, size_t count, int num_pages);
static void unlock_user_pages(struct pg_list *pglist, int dirty);
static void create_sg_list(struct u3v_stream *stream, struct pg_list *pglist);
static void destroy_sg_list(struct pg_list *pglist);
/* Helper functions for u3v_queue_buffer */
static void reset_counters(struct buffer_entry *entry);
static int submit_stream_urb(struct u3v_stream *stream,
			     struct urb_info *urb_info);
/* Stream URB callback function */
static void stream_urb_completion(struct urb *purb);
/* Cancels buffers and resets the stream bulk in endpoint */
static int reset_stream(struct u3v_stream *stream);
/* Helper functions for accessing the rb tree of buffer entries */
static struct buffer_entry *search_buffer_entries(struct u3v_stream *stream,
	__u64 value);
static void buffer_entry_insert(struct u3v_stream *stream,
	struct buffer_entry *new);


/*
 * u3v_create_stream - Initializes the stream interface.
 * @u3v: pointer to the u3v_device struct
 * @intf: pointer to the stream interface struct
 * @image_buffer_size: the size of the buffers that will hold the image data.
 *	Could include size of both the image data and chunk data if they
 *	are to be acquired into the same buffer. Otherwise, it is just
 *	the size of the image data.
 * @chunk_data_buffer_size: the size of the buffers that will hold only
 *	chunk data. If the chunk data is acquired into the same buffer
 *	as the image data, this size should be 0.
 * @max_leader_size: the max required size + alignment for the leader buffer
 * @max_trailer_size: the max required size + alignment for the trailer buffer
 * @payload_size: size of each payload buffer
 * @payload_count: number of payload buffers
 * @transfer1_size: size of transfer1 payload buffer
 * @transfer2_size: size of transfer2 payload buffer
 */
int u3v_create_stream(struct u3v_device *u3v, struct usb_interface *intf,
	u64 image_buffer_size, u64 chunk_data_buffer_size,
	u32 max_leader_size, u32 max_trailer_size, u32 payload_size,
	u32 payload_count, u32 transfer1_size, u32 transfer2_size)
{
	struct u3v_stream *stream = NULL;
	int ret = 0;

	if (u3v == NULL)
		return -EINVAL;

	if (u3v->stream_info.bulk_in == NULL) {
		dev_err(u3v->device,
			"%s: Did not detect bulk in endpoint in the stream interface.\n",
			__func__);
		return U3V_ERR_NO_STREAM_INTERFACE;
	}

	if (u3v->u3v_info->sirm_addr == 0) {
		dev_err(u3v->device,
			"%s: the SIRM address is 0 - streaming is not supported\n",
			__func__);
		return U3V_ERR_NO_STREAM_INTERFACE;
	}

	if (u3v->stream_info.interface_ptr != NULL)
		return U3V_ERR_STREAM_ALREADY_CONFIGURED;

	stream = kzalloc(sizeof(*stream), GFP_KERNEL);
	if (stream == NULL)
		return -ENOMEM;


	mutex_init(&stream->stream_lock);
	stream->u3v_dev = u3v;
	stream->wait_in_progress = false;
	init_usb_anchor(&stream->stream_anchor);
	stream->next_buffer_id = 0;
	stream->root = RB_ROOT;
	reset_stream(stream);
	/* set buffer configurations */
	ret = set_buffer_sizes(stream, image_buffer_size,
		chunk_data_buffer_size, max_leader_size,
		max_trailer_size, payload_size, payload_count,
		transfer1_size, transfer2_size);
	if (ret != 0)
		kfree(stream);
	else
		u3v->stream_info.interface_ptr = stream;

	return ret;
}


/*
 * u3v_destroy_stream - destroys the stream interface
 * @u3v: pointer to the u3v_device struct
 */
int u3v_destroy_stream(struct u3v_device *u3v)
{
	struct u3v_stream *stream;
	struct rb_node *node;
	struct buffer_entry *entry;
	int ret;

	if (u3v == NULL)
		return -EINVAL;

	stream = (struct u3v_stream *)(u3v->stream_info.interface_ptr);

	if (stream == NULL)
		return U3V_ERR_STREAM_NOT_CONFIGURED;

	mutex_lock(&stream->stream_lock);

	ret = reset_stream(stream);
	if (ret != 0)
		goto error;

	mutex_unlock(&stream->stream_lock);

	wait_for_completion(&u3v->stream_info.ioctl_complete);

	mutex_lock(&stream->stream_lock);

	for (node = rb_first(&stream->root); node; node = rb_next(node)) {
		entry = rb_entry(node, struct buffer_entry, node);
		destroy_buffer(stream, entry->buffer_id);
	}

	mutex_unlock(&stream->stream_lock);
	kfree(stream);
	u3v->stream_info.interface_ptr = NULL;
	return 0;
error:
	mutex_unlock(&stream->stream_lock);
	return ret;
}


/*
 * set_buffer_sizes - helper function for configure_stream that initializes
 *	buffer size information and sets the device up for streaming
 * precondition: must be called from u3v_create_stream
 * @stream: pointer to the u3v_stream interface struct
 * @image_buffer_size: the size of the buffers that will hold the image data.
 *	Could include size of both the image data and chunk data if they
 *	are to be acquired into the same buffer. Otherwise, it is just
 *	the size of the image data.
 * @chunk_data_buffer_size: the size of the buffers that will hold only
 *	chunk data. If the chunk data is acquired into the same buffer
 *	as the image data, this size should be 0.
 * @max_leader_size: the max required size + alignment for the leader buffer
 * @max_trailer_size: the max required size + alignment for the trailer buffer
 * @payload_size: size of each payload buffer
 * @payload_count: number of payload buffers
 * @transfer1_size: size of transfer1 payload buffer
 * @transfer2_size: size of transfer2 payload buffer
 */
static int set_buffer_sizes(struct u3v_stream *stream,
	u64 image_buffer_size,
	u64 chunk_data_buffer_size, u32 max_leader_size,
	u32 max_trailer_size, u32 payload_size, u32 payload_count,
	u32 transfer1_size, u32 transfer2_size)
{
	struct u3v_device *u3v;
	struct device *dev;
	u32 transfer2_data_size;

	if (stream == NULL)
		return -EINVAL;

	u3v = stream->u3v_dev;
	dev = u3v->device;

	dev_dbg(dev, "%s: image buffer size = %llu\n",
		__func__, image_buffer_size);
	dev_dbg(dev, "%s: chunk data buffer size = %llu\n",
		__func__, chunk_data_buffer_size);
	dev_dbg(dev, "%s: payload size = %u\n",
		__func__, payload_size);
	dev_dbg(dev, "%s: payload count = %u\n",
		__func__, payload_count);
	dev_dbg(dev, "%s: transfer1 = %u\n",
		__func__, transfer1_size);
	dev_dbg(dev, "%s: transfer2 = %u\n",
		__func__, transfer2_size);

	if (image_buffer_size == 0 || max_leader_size == 0 ||
	    max_trailer_size == 0 || payload_size == 0) {
		dev_err(dev,
			"%s: leader, trailer, and image buffer sizes cannot be 0\n",
			__func__);
		dev_err(dev, "\timage buffer size is %llu\n",
			image_buffer_size);
		dev_err(dev, "\tmax leader buffer size is %u\n",
			max_leader_size);
		dev_err(dev, "\tmax trailer buffer size is %u\n",
			max_trailer_size);
		dev_err(dev, "\tpayload transfer buffer size is %u\n",
			payload_size);
		return -EINVAL;
	}
	if ((image_buffer_size + chunk_data_buffer_size) <
	   ((payload_size * payload_count) + transfer1_size)) {

		dev_err(dev,
			"%s: buffer sizes are too small to hold all of the requested DMA payload data. Total buffer size = %llu, calculated DMA size = %u\n",
			__func__, image_buffer_size + chunk_data_buffer_size,
			((payload_size * payload_count) + transfer1_size));
		return -EINVAL;
	}
	/*
	 * Calculate how much data we actually expect to be valid in the
	 * final transfer 2 buffer
	 */
	transfer2_data_size = (image_buffer_size + chunk_data_buffer_size -
		(payload_size * payload_count) - transfer1_size);

	dev_dbg(dev, "%s: transfer2_data = %u\n",
		__func__, transfer2_data_size);

	if (transfer2_data_size > transfer2_size) {
		dev_err(dev,
			"%s: final transfer 2 data size (%u) exceeds the size of the buffer (%u)\n",
			__func__, transfer2_data_size, transfer2_size);
		return -EINVAL;
	}

	/* The buffer sizes are valid, so now we store them */
	stream->config.image_buffer_size = image_buffer_size;
	stream->config.chunk_data_buffer_size = chunk_data_buffer_size;
	stream->config.max_leader_size = max_leader_size;
	stream->config.max_trailer_size = max_trailer_size;
	stream->config.payload_size = payload_size;
	stream->config.payload_count = payload_count;
	stream->config.transfer1_size = transfer1_size;
	stream->config.transfer2_size = transfer2_size;
	stream->config.transfer2_data_size = transfer2_data_size;

	return 0;
}


/*
 * u3v_configure_buffer - creates the buffer context data, page locks the
 *	user mode image and chunk data buffers, and adds the buffer to the
 *	rbtree of available buffers.
 * @stream: pointer to the stream interface struct
 * @user_image_buffer: user mode memory address of the image buffer to lock
 * @user_chunk_data_buffer: user mode memory address of the chunk data buffer
 *	to lock
 */
int u3v_configure_buffer(struct u3v_stream *stream, void *user_image_buffer,
	void *user_chunk_data_buffer, __u64 *buffer_id)
{
	struct device *dev;
	int ret;

	if (stream == NULL || user_image_buffer == NULL ||
		buffer_id == NULL)
		return -EINVAL;
	dev = stream->u3v_dev->device;

	if ((user_chunk_data_buffer == NULL &&
	     stream->config.chunk_data_buffer_size != 0) ||
	     (user_chunk_data_buffer != NULL &&
	     stream->config.chunk_data_buffer_size == 0)) {

		dev_err(dev,
			"%s: Invalid chunk data buffer address and size combination\n",
			__func__);
		return -EINVAL;
	}
	ret = create_buffer_entry(stream, user_image_buffer,
		user_chunk_data_buffer, buffer_id);
	if (ret != 0)
		return ret;

	return 0;
}


/*
 * create_buffer_entry - this function allocates and initializes the buffer
 *	context data, page locks the image buffer and chunk data buffer,
 *	allocates buffers for the leader and trailer URBs, and allocates
 *	a scratch buffer for the final transfer 2 URB if necessary
 *
 * @stream: pointer to the stream interface struct
 * @user_image_buffer: memory address of the user image buffer to be locked
 * @user_chunk_data_buffer: memory address of the user chunk data buffer to
 *	be locked
 * @buffer_id: on return this user pointer points to valid buffer id
 */
static int create_buffer_entry(struct u3v_stream *stream,
			       void *user_image_buffer,
			       void *user_chunk_data_buffer,
			       __u64 *buffer_id)
{
	int i;
	int ret;
	size_t image_offset;
	size_t chunk_data_offset;
	struct buffer_entry *entry;
	u8 *user_transfer2_addr;

	entry = kzalloc(sizeof(*entry), GFP_KERNEL);
	if (entry == NULL)
		return -ENOMEM;

	/*
	 * We will need URBs for each of the payloads as well as the
	 * leader, trailer, final transfer 1 (if applicable) and final
	 * transfer 2 (if applicable)
	 */
	entry->urb_info_count = stream->config.payload_count + 2;
	if (stream->config.transfer1_size != 0)
		entry->urb_info_count++;
	if (stream->config.transfer2_size != 0)
		entry->urb_info_count++;

	entry->urb_info_array = kzalloc(sizeof(struct urb_info) *
		entry->urb_info_count, GFP_KERNEL);

	if (entry->urb_info_array == NULL) {
		kfree(entry);
		return -ENOMEM;
	}

	/* Initialize the common fields of all of the urb_info structs */
	for (i = 0; i < entry->urb_info_count; i++) {
		entry->urb_info_array[i].urb_index = i;
		entry->urb_info_array[i].stream = stream;
		entry->urb_info_array[i].entry = entry;
		entry->urb_info_array[i].purb = usb_alloc_urb(0, GFP_KERNEL);
		if (entry->urb_info_array[i].purb == NULL) {
			ret = -ENOMEM;
			goto error;
		}
	}

	/* Initialize the leader */
	i = 0;
	ret = allocate_urb_buffer(stream, &entry->urb_info_array[i++],
		stream->config.max_leader_size, sizeof(struct leader_header));
	if (ret != 0)
		goto error;

	/* Initialize the buffer offsets for configuring the URBs */
	image_offset = 0;
	chunk_data_offset = 0;

	while ((i < stream->config.payload_count + 1) &&
	       (image_offset < stream->config.image_buffer_size)) {

		ret = map_urb_buffer(stream, &entry->urb_info_array[i++],
			user_image_buffer, stream->config.image_buffer_size,
			image_offset, stream->config.payload_size,
			stream->config.payload_size);
		if (ret != 0)
			goto error;

		image_offset += stream->config.payload_size;
	}
	/*
	 * If we account for the total image size before getting to
	 * the last payload transfer then switch to the chunk data
	 * buffer for the remaining payload transfer requests
	 */

	while (i < stream->config.payload_count + 1) {
		ret = map_urb_buffer(stream, &entry->urb_info_array[i++],
			user_chunk_data_buffer,
			stream->config.chunk_data_buffer_size,
			chunk_data_offset, stream->config.payload_size,
			stream->config.payload_size);
		if (ret != 0)
			goto error;
		chunk_data_offset += stream->config.payload_size;
	}

	/*
	 * Configure the final transfer 1 buffer if applicable. If we
	 * haven't already finished requesting all of the image data
	 * then use the image buffer. Otherwise, use the chunk data buffer.
	 */
	if (stream->config.transfer1_size != 0) {
		if (image_offset < stream->config.image_buffer_size) {
			ret = map_urb_buffer(stream,
				&entry->urb_info_array[i++],
				user_image_buffer,
				stream->config.image_buffer_size,
				image_offset, stream->config.transfer1_size,
				stream->config.transfer1_size);
			if (ret != 0)
				goto error;
			image_offset += stream->config.transfer1_size;
		} else {
			ret = map_urb_buffer(stream,
				&entry->urb_info_array[i++],
				user_chunk_data_buffer,
				stream->config.chunk_data_buffer_size,
				chunk_data_offset,
				stream->config.transfer1_size,
				stream->config.transfer1_size);
			if (ret != 0)
				goto error;
			chunk_data_offset += stream->config.transfer1_size;
		}
	}

	entry->transfer2_addr = NULL;
	user_transfer2_addr = NULL;
	/*
	 * Configure the final transfer 2 buffer if applicable. If the
	 * transfer 2 size is not equal to the transfer 2 data size, then
	 * use a scratch buffer. We will copy from the scratch buffer
	 * to the specified address or just transfer the data straight
	 * to the image or chunk data buffer
	 */
	if (stream->config.transfer2_size != 0) {

		if (stream->config.transfer2_size !=
		    stream->config.transfer2_data_size) {

			ret = allocate_urb_buffer(stream,
				&entry->urb_info_array[i++],
				stream->config.transfer2_size,
				stream->config.transfer2_data_size);
			if (ret != 0)
				goto error;

			if (image_offset < stream->config.image_buffer_size) {
				user_transfer2_addr =
					(u8 *)(user_image_buffer) +
					image_offset;
				image_offset +=
					stream->config.transfer2_data_size;
			} else {
				user_transfer2_addr =
					(u8 *)(user_chunk_data_buffer) +
					chunk_data_offset;
				chunk_data_offset +=
					stream->config.transfer2_data_size;
			}
			/* lock user memory */
			entry->urb_info_array[i - 1].pglist =
				create_pg_list(stream,
				user_transfer2_addr,
				stream->config.transfer2_size);
			if (entry->urb_info_array[i - 1].pglist == NULL) {
				ret = U3V_ERR_INTERNAL;
				goto error;
			}

			entry->transfer2_addr = user_transfer2_addr;
		} else {
			if (image_offset < stream->config.image_buffer_size) {
				ret = map_urb_buffer(stream,
					&entry->urb_info_array[i++],
					user_image_buffer,
					stream->config.image_buffer_size,
					image_offset,
					stream->config.transfer2_size,
					stream->config.transfer2_data_size);
				if (ret != 0)
					goto error;
				image_offset +=
					stream->config.transfer2_data_size;
			} else {
				ret = map_urb_buffer(stream,
					&entry->urb_info_array[i++],
					user_chunk_data_buffer,
					stream->config.chunk_data_buffer_size,
					chunk_data_offset,
					stream->config.transfer2_size,
					stream->config.transfer2_data_size);
				if (ret != 0)
					goto error;
				chunk_data_offset +=
					stream->config.transfer2_data_size;
			}
		}
	}

	/* Initialize the trailer */
	ret = allocate_urb_buffer(stream, &entry->urb_info_array[i++],
		stream->config.max_trailer_size,
		sizeof(struct trailer_header));
	if (ret != 0)
		goto error;

	if ((i != entry->urb_info_count) ||
	     (image_offset != stream->config.image_buffer_size) ||
	     (chunk_data_offset != stream->config.chunk_data_buffer_size)) {

		dev_err(stream->u3v_dev->device,
			"%s: error configuring buffer sizes\n", __func__);
		ret = U3V_ERR_INTERNAL;
		goto error;
	}

	init_completion(&entry->buffer_complete);
	complete_all(&entry->buffer_complete);
	atomic_set(&entry->callbacks_received, 0);
	entry->state = u3v_idle;
	entry->buffer_id = stream->next_buffer_id++;
	put_user(entry->buffer_id, buffer_id);
	entry->leader_received_size = 0;
	entry->payload_received_size = 0;
	entry->trailer_received_size = 0;
	entry->status = 0;

	mutex_lock(&stream->stream_lock);
	buffer_entry_insert(stream, entry);
	mutex_unlock(&stream->stream_lock);
	return 0;

error:
	for (i = 0; i < entry->urb_info_count; i++)
		destroy_urb(&entry->urb_info_array[i]);
	kfree(entry->urb_info_array);
	kfree(entry);
	return ret;
}


/*
 * allocate_urb_buffer - this function populates a urb_info struct by
 *	allocating a kernel buffer. If a user buffer exists and needs to
 *	be mapped to the kernel, map_urb_buffer is used instead of this
 *	function
 * @stream: pointer to the stream interface struct
 * @urb_info: on return this struct will be initialized
 * @size: size of the buffer to allocate
 * @expected_size: amount of data we expect to receive
 */
static int allocate_urb_buffer(struct u3v_stream *stream,
			       struct urb_info *urb_info, size_t size,
			       size_t expected_size)
{
	if (urb_info == NULL)
		return -EINVAL;

	urb_info->buffer = kzalloc(size, GFP_KERNEL);
	if (urb_info->buffer == NULL)
		return -ENOMEM;
	urb_info->kernel_allocated = true;
	urb_info->pglist = NULL;
	urb_info->buffer_size = size;
	urb_info->expected_size = expected_size;
	return 0;
}


/*
 * map_urb_buffer - this function locks a segment of user memory, creates
 *	a corresponding scatter-gather list, and populates the urb_info
 *	struct accordingly. This function is an alternative to
 *	allocate_urb_buffer and is used to DMA directly into the user
 *	buffer without making a copy.
 * @stream: pointer to the stream interface struct
 * @urb_info: on return this struct will be initialized
 * @user_buffer: user mode memory address of the buffer to lock
 * @buffer_size: size of the user buffer
 * @offset: offset into the user buffer
 * @urb_buffer_size: size of the URB buffer
 * @expected_size: amount of data we expect to receive
 */
static int map_urb_buffer(struct u3v_stream *stream,
	struct urb_info *urb_info, void *user_buffer, size_t buffer_size,
	size_t offset, size_t urb_buffer_size, size_t expected_size)
{
	if (urb_info == NULL || user_buffer == NULL || buffer_size == 0 ||
		(offset + expected_size > buffer_size))
		return -EINVAL;

	urb_info->kernel_allocated = false;
	urb_info->pglist = create_pg_list(stream, user_buffer + offset,
		expected_size);
	if (urb_info->pglist == NULL)
		return -ENOMEM;
	urb_info->buffer = NULL;
	urb_info->buffer_size = urb_buffer_size;
	urb_info->expected_size = expected_size;
	return 0;
}


/*
 * u3v_unconfigure_buffer - finds the buffer with buffer_id,
 * unlocks corresponding pagelists, and frees all resources associated
 * with the buffer entry.
 * @stream: pointer to the u3v_stream interface struct
 * @buffer_id: id of the buffer to be destroyed
 */
int u3v_unconfigure_buffer(struct u3v_stream *stream, __u64 buffer_id)
{
	int ret;

	if (stream == NULL)
		return -EINVAL;

	mutex_lock(&stream->stream_lock);
	ret = destroy_buffer(stream, buffer_id);
	mutex_unlock(&stream->stream_lock);

	return ret;
}


/*
 * destroy_buffer - helper function that finds a buffer entry by id,
 *	removes it from the list of buffers, and frees its memory
 * precondition: caller must hold stream_lock
 * @stream: pointer to the u3v_stream interface struct
 * @buffer_id: id of the buffer to be destroyed
 */
static int destroy_buffer(struct u3v_stream *stream, __u64 buffer_id)
{
	struct buffer_entry *entry;
	struct device *dev;
	int i;

	if (stream == NULL)
		return -EINVAL;

	dev = stream->u3v_dev->device;

	entry = search_buffer_entries(stream, buffer_id);

	if (entry == NULL) {
		dev_err(dev, "%s: Failed to find entry with buffer id %llu\n",
			__func__, buffer_id);
		return -EINVAL;
	}

	if (entry->state == u3v_queued) {
		dev_err(dev, "%s: Buffer is in use, cannot be unconfigured\n",
			__func__);
		return U3V_ERR_BUFFER_STILL_IN_USE;
	}

	rb_erase(&entry->node, &stream->root);

	for (i = 0; i < entry->urb_info_count; i++)
		destroy_urb(&entry->urb_info_array[i]);

	kfree(entry->urb_info_array);
	kfree(entry);

	return 0;
}


/*
 * destroy_urb - helper function that handles the destruction of a urb.
 *	If the urb was mapped from user memory, we destroy the page list
 *	and unlock the memory. If it was allocated from kernel memory,
 *	we just free that buffer
 * precondition: caller must hold stream_lock
 * @urb_info: pointer to struct containing a urb pointer and its metadata
 */
static void destroy_urb(struct urb_info *urb_info)
{
	if (urb_info == NULL)
		return;

	usb_free_urb(urb_info->purb);

	if (urb_info->kernel_allocated)
		kfree(urb_info->buffer);

	if (urb_info->pglist != NULL)
		destroy_pg_list(urb_info->pglist, true);
}


/*
 * u3v_queue_buffer - queues the buffer to the camera for it to be filled
 *	with image data
 * @stream: pointer to the stream interface struct
 * @buffer_id: on return, buffer with this id is queued
 */
int u3v_queue_buffer(struct u3v_stream *stream, __u64 buffer_id)
{
	struct buffer_entry *entry;
	struct device *dev = stream->u3v_dev->device;
	int ret = 0;
	int i = 0;
	if (stream == NULL)
		return -EINVAL;

	dev_vdbg(dev, "%s: buffer id = %llu\n", __func__, buffer_id);

	mutex_lock(&stream->stream_lock);
	entry = search_buffer_entries(stream, buffer_id);

	if (entry == NULL) {
		dev_err(dev, "%s: Failed to find entry with id %llu\n",
			__func__, buffer_id);
		ret = U3V_ERR_INTERNAL;
		goto exit;
	}

	if (entry->state == u3v_queued) {
		dev_err(dev, "%s: Error, buffer %llu is already queued\n",
			__func__, buffer_id);
		ret = U3V_ERR_INTERNAL;
		goto exit;
	}

	reset_counters(entry);
	entry->state = u3v_queued;
	u3v_reinit_completion(&entry->buffer_complete);

	for (i = 0; i < entry->urb_info_count; i++) {
		ret = submit_stream_urb(stream, &entry->urb_info_array[i]);
		if (ret != 0)
			goto reset_stream_exit;
	}

exit:
	mutex_unlock(&stream->stream_lock);
	return ret;

reset_stream_exit:
	entry->state = u3v_cancelled;
	complete_all(&entry->buffer_complete);
	reset_stream(stream);
	mutex_unlock(&stream->stream_lock);
	return ret;
}


/*
 * reset_counters - helper function that resets values in a buffer
 *	entry so that it can be resubmitted
 * precondition: caller must hold stream_lock
 * @entry: buffer to be reset
 */
static void reset_counters(struct buffer_entry *entry)
{
	entry->state = u3v_idle;
	atomic_set(&entry->callbacks_received, 0);
	entry->incomplete_callbacks_received = 0;
	entry->leader_received_size = 0;
	entry->payload_received_size = 0;
	entry->trailer_received_size = 0;
	entry->status = 0;
}


/*
 * submit_stream_urb - queues a buffer to the stream endpoint and
 *	increments the number of outstanding URBS
 * precondition: caller must hold stream_lock
 * @stream: pointer to the stream interface struct
 * @urb_info: pointer to the urb_info struct that contains the
 *	urb to be submitted
 */
static int submit_stream_urb(struct u3v_stream *stream,
			     struct urb_info *urb_info)
{
	struct device *dev;
	struct usb_device *udev;
	int ret;

	if (stream == NULL || urb_info == NULL || urb_info->purb == NULL)
		return -EINVAL;

	dev = stream->u3v_dev->device;
	udev = stream->u3v_dev->udev;

	if (urb_info->kernel_allocated && (urb_info->buffer == NULL)) {
		dev_err(dev, "%s: NULL buffer for kernel URB\n", __func__);
		return -EINVAL;
	}

	if (!urb_info->kernel_allocated && (urb_info->pglist == NULL)) {
		dev_err(dev, "%s: NULL page list for user buffer URB\n",
			__func__);
		return -EINVAL;
	}
	usb_fill_bulk_urb(urb_info->purb, udev,
		usb_rcvbulkpipe(udev,
		usb_endpoint_num(stream->u3v_dev->stream_info.bulk_in)),
		urb_info->buffer, urb_info->buffer_size,
		stream_urb_completion, urb_info);

	/*
	 * If we are setting up zero-copy DMA for user memory,
	 * we want to use the sg list we created
	 */
	if (!urb_info->kernel_allocated) {

		urb_info->purb->sg = urb_info->pglist->sgt->sgl;
		urb_info->purb->num_sgs = urb_info->pglist->sgt->nents;

	}


	usb_anchor_urb(urb_info->purb, &stream->stream_anchor);
	ret = usb_submit_urb(urb_info->purb, GFP_KERNEL);
	if (ret != 0) {
		usb_unanchor_urb(urb_info->purb);
		dev_err(dev, "%s: Error %d submitting urb\n", __func__, ret);
	}

	return ret;
}


/*
 * stream_urb_completion - This function executes when the transfer
 *	finishes, fails, or is cancelled. It runs in interrupt context,
 *	and keeps track of when a buffer entry has received all of its
 *	expected callbacks. Once the callbacks have all been received, it
 *	signals that it is complete so that wait_for_buffer can finish.
 * @purb: pointer to the urb that has been completed
 */
static void stream_urb_completion(struct urb *purb)
{
	struct urb_info *urb_info;
	struct buffer_entry *entry;
	struct device *dev;
	int idx;
	size_t len;
	int urb_count;

	/* check pointers */
	if (purb == NULL || purb->context == NULL) {
		pr_err("%s: Received an invalid urb pointer\n", __func__);
		return;
	}
	dev = &purb->dev->dev;

	/* check status */
	if (purb->status &&
		!(purb->status == -ENOENT ||
		  purb->status == -ECONNRESET ||
		  purb->status == -ESHUTDOWN ||
		  purb->status == -EPROTO ||
		  purb->status == -EPIPE)) {
		dev_err(dev, "%s: Received nonzero urb completion status: %d",
			__func__, purb->status);
	}
	urb_info = (struct urb_info *)(purb->context);
	if (urb_info == NULL) {
		dev_err(dev, "%s: Received a NULL context pointer\n",
			__func__);
		return;
	}

	entry = urb_info->entry;
	if (entry == NULL) {
		dev_err(dev, "%s: Received a NULL buffer entry pointer\n",
			__func__);
		return;
	}

	if (entry->state == u3v_cancelled)
		return;

	/* If status is already bad, don't overwrite */
	if (purb->status != 0)
		entry->status = purb->status;
	len = purb->actual_length;

	dev_vdbg(dev, "%s: urb %d: length = %zu, expected >=%zu\n",
		__func__, urb_info->urb_index, len,
		urb_info->expected_size);

	WARN_ON(entry->state != u3v_queued);

	if (len < urb_info->expected_size)
		entry->incomplete_callbacks_received++;

	/* Handle the callback based on the urb index */
	idx = urb_info->urb_index;
	urb_count = entry->urb_info_count;

	if (idx == 0) {
		/* leader */
		entry->leader_received_size = len;
	} else if (idx == urb_count - 1) {
		/* trailer */
		entry->trailer_received_size = len;
	} else {
		/* payload */
		if ((idx == urb_count - 2) &&
		    (entry->transfer2_addr != NULL)) {
			entry->transfer2_received_size = len;
		}
		entry->payload_received_size += len;
	}
	/*
	 * If this is the last callback for a buffer entry,
	 * signal waiting thread
	 */
	if (atomic_inc_return(&entry->callbacks_received) ==
		urb_info->entry->urb_info_count) {
		entry->state = u3v_complete;
		complete_all(&entry->buffer_complete);
	}
}


/*
 * u3v_wait_for_buffer - This function waits for the buffer entry with
 *	the specified id to receive all of its callbacks, then copies
 *	the leader, trailer and buffer complete data into the
 *	provided buffers.
 * @stream: pointer to the stream interface
 * @buffer_id: the buffer to be waited on
 * @user_leader_buffer: if not NULL, the leader information will be copied
 *	to this buffer on return
 * @leader_size: size of user_leader_buffer on input, and number of bytes
 *	copied on return. Can be NULL if user_leader_buffer is NULL
 * @user_trailer_buffer: if not NULL, the trailer information will be
 *	copied to this buffer on return
 * @trailer_size: size of the user_trailer_buffer on input, and number
 *	of bytes copied on return. Can be NULL if user_trailer_buffer is NULL
 * @buffer_complete_data: additional data describing the completed buffer,
 *	can be NULL.
 */
int u3v_wait_for_buffer(struct u3v_stream *stream, __u64 buffer_id,
	void *user_leader_buffer, __u32 *leader_size,
	void *user_trailer_buffer, __u32 *trailer_size,
	void *buffer_complete_data)
{
	struct buffer_entry *entry;
	struct device *dev;
	struct buffer_complete_data bcd;
	int ret = 0;
	int leader_index;
	int transfer2_index;
	int trailer_index;
	struct trailer *trailer;

	if (stream == NULL)
		return ret;

	dev = stream->u3v_dev->device;

	dev_vdbg(dev, "%s: buffer id = %llu\n", __func__, buffer_id);

	mutex_lock(&stream->stream_lock);

	entry = search_buffer_entries(stream, buffer_id);

	if (entry == NULL) {
		dev_err(dev, "%s: Failed to find buffer with id %llu\n",
			__func__, buffer_id);
		ret = -EINVAL;
		goto exit;
	}

	if (entry->state == u3v_idle) {
		dev_err(dev, "%s: Error, cannot wait for idle buffer\n",
			__func__);
		ret = -EINVAL;
		goto exit;
	}

	if (entry->state == u3v_cancelled) {
		dev_err(dev, "%s: Error, cannot wait for cancelled buffer\n",
			__func__);
		ret = -EINVAL;
		goto exit;
	}

	if (stream->wait_in_progress) {
		dev_err(dev, "%s: Error, wait is already in progress\n",
			__func__);
		ret = U3V_ERR_ALREADY_WAITING;
		goto exit;
	}

	if (atomic_read(&entry->callbacks_received) !=
		entry->urb_info_count) {

		stream->wait_in_progress = true;
		mutex_unlock(&stream->stream_lock);
		ret = wait_for_completion_interruptible(
			&entry->buffer_complete);
		mutex_lock(&stream->stream_lock);
		stream->wait_in_progress = false;
	}
	trailer = (struct trailer *)
		entry->urb_info_array[entry->urb_info_count - 1].buffer;
	dev_vdbg(dev, "%s: device reports valid payload size is %llu\n",
		__func__, trailer->header.valid_payload_size);
	dev_vdbg(dev, "%s: driver reports payload received size is %u\n",
		__func__, entry->payload_received_size);
	dev_vdbg(dev, "%s: device reports status %04X\n",
		__func__, trailer->header.status);

	if (ret != 0) {
		dev_err(dev, "%s: Error, interrupted by signal\n",
			__func__);
		ret = U3V_ERR_BUFFER_WAIT_CANCELED;
		goto exit;
	}

	if (entry->state != u3v_complete) {
		dev_err(dev,
			"%s: Cannot copy from the requested buffer because it has been requeued or cancelled\n",
			__func__);
		ret = U3V_ERR_BUFFER_CANNOT_BE_COPIED;
		goto exit;
	}

	/* Fill in the requested information */
	leader_index = 0;
	transfer2_index = entry->urb_info_count - 2;
	trailer_index = entry->urb_info_count - 1;

	if (user_leader_buffer != NULL && leader_size != NULL) {

		ret = copy_to_user(user_leader_buffer,
			entry->urb_info_array[leader_index].buffer,
			min(entry->leader_received_size, *leader_size));
		if (ret != 0) {
			dev_err(dev, "%s: Error copying leader buffer\n",
				__func__);
			goto exit;
		}
	}
	if (leader_size != NULL)
		put_user(entry->leader_received_size, leader_size);

	if (entry->transfer2_addr != NULL) {
			ret = copy_to_user(entry->transfer2_addr,
				entry->urb_info_array[transfer2_index].buffer,
				min((size_t)(entry->transfer2_received_size),
				entry->urb_info_array[transfer2_index].
				expected_size));
		if (ret != 0) {
			dev_err(dev,  "%s: Error copying transfer2 buffer\n",
				__func__);
			goto exit;
		}
	}

	if (user_trailer_buffer != NULL && trailer_size != NULL) {
		ret = copy_to_user(user_trailer_buffer,
			entry->urb_info_array[trailer_index].buffer,
			min(entry->trailer_received_size, *trailer_size));
		if (ret != 0) {
			dev_err(dev,  "%s: Error copying trailer buffer\n",
				__func__);
			goto exit;
		}
	}

	if (trailer_size != NULL)
		put_user(entry->trailer_received_size, trailer_size);

	if (buffer_complete_data != NULL) {
		bcd.structure_size = sizeof(struct buffer_complete_data);
		bcd.status = entry->status;
		bcd.expected_urb_count = entry->urb_info_count;
		bcd.incomplete_urb_count =
			entry->incomplete_callbacks_received;
		bcd.payload_bytes_received = entry->payload_received_size;

		ret = copy_to_user(buffer_complete_data, &bcd,
			bcd.structure_size);
		if (ret != 0) {
			dev_err(dev, "%s: Error copying buffer metadata\n",
				__func__);
			goto exit;
		}
	}

exit:
	mutex_unlock(&stream->stream_lock);
	return ret;
}


/*
 * u3v_cancel_all_buffers - cancels any URB transactions that are in
 *	progress, which causes queued buffers to be completed.
 * @stream: pointer to the stream interface struct
 */
int u3v_cancel_all_buffers(struct u3v_stream *stream)
{
	int ret;

	if (stream == NULL)
		return -EINVAL;

	mutex_lock(&stream->stream_lock);
	/*
	 * Cancel all urbs that are currently in progress.
	 * on return, all callbacks have been completed
	 */
	ret = reset_stream(stream);
	mutex_unlock(&stream->stream_lock);

	return ret;
}


/*
 * create_pg_list - helper function that creates a struct pg_list
 *	out of a chunk of user memory of size num_bytes. It generates
 *	a list of page-locked memory and then creates a scatter-gather
 *	list to be submitted for DMA transfer.
 * @stream: pointer to the stream interface struct
 * @user_buffer: user buffer to be locked in memory
 * @num_bytes: buffer size
 * @return: pointer to the page_list if successful, NULL otherwise.
 */
static struct pg_list *create_pg_list(struct u3v_stream *stream,
	void *user_buffer, size_t num_bytes)
{
	struct device *dev = stream->u3v_dev->device;
	struct pg_list *pglist = kzalloc(sizeof(struct pg_list),
		GFP_KERNEL);
	if (pglist == NULL)
		return NULL;

	pglist->num_bytes = num_bytes;
	pglist->num_pages = get_num_pages((uintptr_t)(user_buffer),
		num_bytes);
	pglist->offset = ((uintptr_t)(user_buffer) & ~PAGE_MASK);
	pglist->pages = lock_user_pages(stream, (uintptr_t)(user_buffer),
		num_bytes, pglist->num_pages);

	if (pglist->pages == NULL) {
		/* clear fields */
		pglist->num_bytes = 0;
		pglist->num_pages = 0;
		pglist->offset = 0;
		goto error;
	}

	create_sg_list(stream, pglist);
	if (pglist->sgt == NULL)
		goto error;

	return pglist;
error:
	dev_err(dev, "%s: Error creating page list with user buffer\n",
		__func__);
	/* 0 because pages are not dirty */
	destroy_pg_list(pglist, 0);
	return NULL;
}


/*
 * destroy_pg_list - helper function that unlocks the pages pinned from
 *	user memory and frees corresponding sglist resources
 * precondition: caller must hold stream_lock or buffer has not yet been
 *	added to the list
 * @pglist: pointer to page_list to be destroyed
 * @dirty: true if data needs to be written out to disk
 */
static void destroy_pg_list(struct pg_list *pglist, bool dirty)
{
	if (pglist == NULL)
		return;

	unlock_user_pages(pglist, dirty);
	destroy_sg_list(pglist);
	kfree(pglist);
}


/*
 * create_sg_list - helper function that creates a scatter-gather list from
 *	a page_list struct
 * @stream: pointer to the stream interface struct
 * @pglist: pointer to page_list to create the sg list from
 */
static void create_sg_list(struct u3v_stream *stream, struct pg_list *pglist)
{
	int ret, i, last_entry;
	struct device *dev = stream->u3v_dev->device;
	pglist->sgt = kzalloc(sizeof(struct sg_table), GFP_KERNEL);
	if (pglist->sgt == NULL)
		return;

#if LINUX_VERSION_CODE <= KERNEL_VERSION(3, 6, 0)
	ret = 0;
	pglist->sgt->sgl =
		kzalloc(pglist->num_pages * sizeof(struct scatterlist),
		GFP_KERNEL);

	if (!pglist->sgt->sgl) {
		kfree(pglist->sgt);
		pglist->sgt = NULL;
		return;
	}

	sg_init_table(pglist->sgt->sgl, pglist->num_pages);
	pglist->sgt->nents = pglist->num_pages;
	pglist->sgt->orig_nents = pglist->num_pages;
	/* first */
	sg_set_page(&pglist->sgt->sgl[0], pglist->pages[0],
		PAGE_SIZE - pglist->offset, pglist->offset);

	last_entry = 0;
	/* middle */
	for (i = 1; i < (pglist->num_pages - 1); i++) {
		if (page_to_pfn(pglist->pages[i]) ==
			page_to_pfn(pglist->pages[i - 1]) + 1) {

			pglist->sgt->sgl[last_entry].length += PAGE_SIZE;
			pglist->sgt->nents--;
		} else {
			sg_set_page(&pglist->sgt->sgl[++last_entry],
			pglist->pages[i], PAGE_SIZE, 0);
		}
	}
	/* last */
	if (pglist->num_pages > 1) {
		sg_set_page(&pglist->sgt->sgl[++last_entry],
			pglist->pages[pglist->num_pages - 1],
			pglist->num_bytes - (PAGE_SIZE - pglist->offset) -
			((pglist->num_pages - 2) * PAGE_SIZE), 0);
	}
#else
	i = 0;
	last_entry = 0;
	ret = sg_alloc_table_from_pages(pglist->sgt, pglist->pages,
		pglist->num_pages, pglist->offset, pglist->num_bytes,
		GFP_KERNEL);
	if (ret != 0) {
		dev_err(dev, "%s: failed to allocate sgtable with error %d\n",
			__func__, ret);
		kfree(pglist->sgt);
		pglist->sgt = NULL;
		return;
	}
#endif
}


/*
 * destroy_sg_list - helper function that destroys a scatter-gather list in
 *	a page_list struct
 * @pglist: pointer to page_list with the sglist to be destroyed
 */
static void destroy_sg_list(struct pg_list *pglist)
{
	if (pglist == NULL || pglist->sgt == NULL)
		return;
#if LINUX_VERSION_CODE <= KERNEL_VERSION(3, 6, 0)
	kfree(pglist->sgt->sgl);
#else
	sg_free_table(pglist->sgt);
#endif
	kfree(pglist->sgt);
	pglist->sgt = NULL;
}


/*
 * lock_user_pages - helper function that takes a pointer to num_bytes
 *	of contiguous virtual user memory, faults in any pages, then
 *	locks them in memory.
 * @stream: pointer to the stream interface struct
 * @uaddr: user virtual address
 * @num_bytes: buffer size in bytes
 * @num_pages: number of physical pages this buffer consists of
 * @return: a pointer to an array of locked pages if successful,
 *	NULL otherwise.
 */
static struct page **lock_user_pages(struct u3v_stream *stream,
	uintptr_t uaddr, size_t num_bytes, int num_pages)
{
	int ret, i;
	struct page **pages;
	struct device *dev = stream->u3v_dev->device;

	/* Check for overflow */
	if ((uaddr + num_bytes) < uaddr)
		return NULL;

	/* No point continuing in this case */
	if (num_bytes == 0)
		return NULL;

	pages = kmalloc((num_pages * sizeof(struct page)), GFP_KERNEL);
	if (pages == NULL)
		return NULL;

	/* Fault in all of the necessary pages */
	down_read(&current->mm->mmap_sem);
	/* will store a page locked array of physical pages in pages var */
	ret = get_user_pages(
		current,
		current->mm,
		uaddr,
		num_pages,
		WRITE,
		0, /* Don't force */
		pages,
		NULL);
	up_read(&current->mm->mmap_sem);

	if (ret < num_pages) {
		dev_err(dev, "%s: get_user_pages returned %d, expected %d\n",
			__func__, ret, num_pages);
		goto err_unlock;
	}

	for (i = 0; i < num_pages; i++)
		flush_dcache_page(pages[i]);

	return pages;

err_unlock:
	if (ret > 0) {
		for (i = 0; i < ret; i++)
			put_page(pages[i]);
	}
	kfree(pages);
	return NULL;
}


/*
 * reset_stream - cancels any outstanding urbs and resets the stream
 *	bulk in endpoint
 * precondition: caller holds stream_lock or has not yet set the stream
 *	interface pointer
 * @stream - pointer to the stream interface struct
 */
static int reset_stream(struct u3v_stream *stream)
{
	struct u3v_device *u3v;
	struct usb_device *udev;
	u8 ep_addr;
	int dummy_size = 32;
	u8 dummy_buffer[dummy_size];
	int actual = 0;
	int ret = 0;

	if (stream == NULL)
		return -EINVAL;

	u3v = stream->u3v_dev;
	if (!u3v->device_connected)
		return 0;

	udev = u3v->udev;
	ep_addr = u3v->stream_info.bulk_in->bEndpointAddress;

	/*
	 * We have had issues with xhci when trying to stall/halt
	 * endpoints, so this is a workaround.
	 */
	if (u3v->stalling_disabled) {
		if (usb_anchor_empty(&stream->stream_anchor) == 0) {
			/* stall pipe */
			ret = usb_control_msg(udev,
			      usb_sndctrlpipe(udev, U3V_CTRL_ENDPOINT),
			      USB_REQ_SET_FEATURE,
			      USB_RECIP_ENDPOINT,
			      USB_ENDPOINT_HALT,
			      ep_addr,
			      NULL,	/* buffer */
			      0,	/* size */
			      U3V_TIMEOUT);
			if (ret != 0) {
				dev_err(u3v->device,
					"%s: Error %d stalling ep %02X\n",
					__func__, ret, ep_addr);
				return ret;
			}
			/* submit dummy read */
			usb_bulk_msg(udev, usb_sndbulkpipe(udev,
				usb_endpoint_num(u3v->stream_info.bulk_in)),
				dummy_buffer, dummy_size,
				&actual, U3V_TIMEOUT);
			/* clear stall */
			ret = usb_clear_halt(udev,
				usb_rcvbulkpipe(udev, ep_addr));
			if (ret != 0) {
				dev_err(u3v->device,
					"%s: Error %d clearing halt on ep %02X\n",
					__func__, ret, ep_addr);
			}
			/* submit another dummy read */
			usb_bulk_msg(udev, usb_sndbulkpipe(udev,
				usb_endpoint_num(u3v->stream_info.bulk_in)),
				dummy_buffer, dummy_size,
				&actual, U3V_TIMEOUT);
		}
	}
	/* aborts outstanding urbs, waits for callbacks to complete */
	usb_kill_anchored_urbs(&stream->stream_anchor);

	if (!u3v->stalling_disabled)
		ret = reset_pipe(stream->u3v_dev, &u3v->stream_info);

	return ret;
}


/*
 * unlock_user_pages - helper function that unlocks user memory that was
 *	previously pagelocked, marking the page as dirty if necessary.
 * @pglist: pointer to the page_list to be unpinned
 * @dirty: true if data needs to be written out to disk
 */
static void unlock_user_pages(struct pg_list *pglist, int dirtied)
{
	int i;

	if (pglist->pages == NULL)
		return;

	for (i = 0; i < pglist->num_pages; i++) {
		struct page *page = pglist->pages[i];
		if (dirtied)
			set_page_dirty_lock(page);
		put_page(page);
	}
	kfree(pglist->pages);
}


/*
 * search_buffer_entries - helper function that traverses the rb tree of
 *	buffer entries to find the one whose id matches the provided value.
 * precondition: caller must hold stream_lock
 * @stream: pointer to the stream interface struct
 * @value: id of the buffer to be found
 * @return: pointer to the entry if found, NULL otherwise
 */
static struct buffer_entry *search_buffer_entries(struct u3v_stream *stream,
						  __u64 value)
{
	struct rb_node *node;

	/* start at top of the tree */
	node = stream->root.rb_node;

	while (node) {
		struct buffer_entry *entry =
			rb_entry(node, struct buffer_entry, node);
		if (entry->buffer_id > value)
			node = node->rb_left;
		else if (entry->buffer_id < value)
			node = node->rb_right;
		else
			return entry;
	}
	return NULL;
}


/*
 * buffer_entry_insert - helper function that adds a new buffer entry
 *	in the correct spot to balance the tree
 * precondition: caller must hold stream_lock
 * @stream: pointer to the stream interface struct
 * @new: buffer entry to be added to the rb tree
 */
static void buffer_entry_insert(struct u3v_stream *stream,
				struct buffer_entry *new)
{
	struct buffer_entry *entry;
	struct rb_node **link = &stream->root.rb_node;
	struct rb_node *parent = NULL;
	__u64 value = new->buffer_id;

	/* Go to the bottom of the tree */
	while (*link) {
		parent = *link;
		entry = rb_entry(parent, struct buffer_entry, node);
		if (entry->buffer_id > value)
			link = &(*link)->rb_left;
		else
			link = &(*link)->rb_right;
	}
	/* Now insert the new node, balance if necessary*/
	rb_link_node(&new->node, parent, link);
	rb_insert_color(&new->node, &stream->root);
}
