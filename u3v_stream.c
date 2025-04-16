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
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/pagemap.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/usb.h>
#include <linux/vmalloc.h>
#ifdef VERSION_COMPATIBILITY
	#include <linux/version.h>
#else
	#include <generated/uapi/linux/version.h>
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0)
	#include <linux/unaligned.h>
#else
	#include <asm/unaligned.h>
#endif

/* Internal stream structs and enums */

enum u3v_buffer_state {
	u3v_idle, u3v_queued, u3v_complete, u3v_cancelled
};

struct u3v_buffer {
	u32 urb_info_count;
	u32 incomplete_callbacks_received;
	u32 leader_received_size;
	u32 payload_received_size;
	u32 trailer_received_size;
	u32 transfer2_received_size;
	void __user *u_transfer2_addr;
	u64 buffer_id;
	atomic_t callbacks_received;
	int status;
	enum u3v_buffer_state state;
	struct u3v_urb_info *urb_info_array;
	struct completion buffer_complete;
	struct rb_node node;
};

struct u3v_urb_info {
	u32 urb_index;
	u32 num_pglists;
	u32 buffer_size;
	u32 expected_size;
	struct u3v_stream *stream;
	struct u3v_buffer *entry;
	struct urb *purb;
	struct u3v_pglist **pglists;
	struct sg_table *sgt;
};

struct u3v_pglist {
	bool kernel_allocated;
	bool segmented_xfer;
	u32 num_host_bytes;
	u32 num_device_bytes;
	u32 num_pages;
	u32 page_offset;
	u64 host_offset;
	u64 device_offset;
	struct page **pages;
};


/*
 * Inline helper function that returns the number of pages in a num_bytes
 * sized chunk of memory starting at addr.
 */
static inline unsigned long get_num_pages(void *ptr, u32 num_bytes)
{
	uintptr_t addr = (uintptr_t)(ptr);
	unsigned long end = (addr + num_bytes + PAGE_SIZE - 1) >> PAGE_SHIFT;
	unsigned long start = addr >> PAGE_SHIFT;
	return end - start;
}

/* Forward declaration for internal functions */

/* Functions related to u3v_create_stream */
static int calculate_urb_sizes(struct u3v_stream *stream,
	struct u3v_configure_stream2 *config_stream,
	u32 req_leader_size, u32 req_trailer_size);

/* Functions related to u3v_configure_buffer */
static int create_buffer(struct u3v_stream *stream,
	void *u_image_buffer,
	void *u_chunk_data_buffer,
	u64 *buffer_id);
static int configure_urbs(struct u3v_stream *stream,
	struct u3v_buffer *entry, void __user *u_image_buffer,
	void __user *u_chunk_data_buffer);
static int configure_urbs_sg_constraint(struct u3v_stream *stream,
	struct u3v_buffer *entry, void __user *u_image_buffer,
	void __user *u_chunk_data_buffer);
static u32 num_host_buffer_bytes(u32 device_bytes, u64 device_image_offset,
	u32 segment_padding, u64 segment_size);
static int add_buffer_to_urb(struct u3v_stream *stream,
	struct u3v_urb_info *urb_info, void __user *u_buffer, u64 host_offset,
	u64 device_offset, u32 buffer_size, u32 expected_size,
	bool use_padding);
static struct u3v_pglist *create_pglist(struct u3v_stream *stream,
	void __user *u_buffer, u64 host_offset, u64 device_offset,
	u32 num_bytes, u32 segment_padding, u64 segment_size);
static struct page **lock_user_pages(struct u3v_stream *stream,
	void __user *u_buffer, u32 num_bytes, u32 num_pages);
static int create_sglist(struct u3v_stream *stream,
	struct u3v_urb_info *urb_info);
static int calculate_sglist_entries(struct u3v_stream *stream,
	struct u3v_urb_info *urb_info, u32 *num_sgs, bool configure);
static int num_contiguous_bytes(struct u3v_pglist *pglist, u32 pg_idx,
	u32 offset, u32 limit);

/* Functions related to u3v_unconfigure_buffer */
static int destroy_buffer(struct u3v_stream *stream, u64 buffer_id);
static void destroy_urb(struct u3v_urb_info *urb_info);
static void destroy_pglist(struct u3v_pglist *pglist, bool dirty);
static void unlock_user_pages(struct u3v_pglist *pglist, int dirty);
static void destroy_sglist(struct u3v_urb_info *urb_info);

/* Functions related to u3v_queue_buffer */
static void reset_counters(struct u3v_buffer *entry);
static int submit_stream_urb(struct u3v_stream *stream,
	struct u3v_urb_info *urb_info);

/* Callback function for when urb is transferred */
static void stream_urb_completion(struct urb *purb);

/* Function related to u3v_cancel_all_buffers */
static int reset_stream(struct u3v_stream *stream);

/*
 * Functions for translating to and from an array of struct page *
 * and a kernel address
 */
static struct page **kernel_addr_to_pages(void *addr, u32 num_pages);
static void *pglist_to_kernel_addr(struct u3v_pglist *pglist);

/* Functions for accessing the rb tree of buffer entries */
static struct u3v_buffer *search_buffer_entries(struct u3v_stream *stream,
	u64 value);
static void buffer_entry_insert(struct u3v_stream *stream,
	struct u3v_buffer *new);

/*
 * Math functions to help with 64 bit operations to avoid __aeabi_uldivmod
 * unresolved symbol for 32 bit OSes
 */
static u32 u3v_u64_mod(u64 dividend, u32 divisor);
static u64 u3v_u64_roundup(u64 value, u32 alignment);
static u64 u3v_u64_rounddown(u64 value, u32 alignment);


/*
 * u3v_create_stream - initializes the stream interface
 *
 * @u3v: pointer to the u3v_device struct
 * @intf: pointer to the stream interface struct
 * @config_stream: pointer to the struct with stream configuration data
 *	from usermode
 * @req_leader_size: minimum required leader size, which was previously
 *	read from a camera register
 * @req_trailer_size: minimum required trailer size, which was previously
 *	read from a camera register
 */
int u3v_create_stream(struct u3v_device *u3v, struct usb_interface *intf,
	struct u3v_configure_stream2 *config_stream, u32 req_leader_size,
	u32 req_trailer_size)
{
	struct u3v_stream *stream = NULL;
	struct device *dev = NULL;
	int ret = 0;

	if (u3v == NULL)
		return -EINVAL;

	dev = u3v->device;

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

	stream = kzalloc(sizeof(struct u3v_stream), GFP_KERNEL);
	if (stream == NULL)
		return -ENOMEM;


	mutex_init(&stream->stream_lock);
	stream->u3v_dev = u3v;
	stream->wait_in_progress = false;
	init_usb_anchor(&stream->stream_anchor);
	stream->next_buffer_id = 0;
	stream->root = RB_ROOT;
	reset_stream(stream);

	ret = calculate_urb_sizes(stream, config_stream,
		req_leader_size, req_trailer_size);
	if (ret != 0)
		goto error;

	u3v->stream_info.interface_ptr = stream;

	return 0;
error:
	kfree(stream);
	return ret;
}


/*
 * calculate_urb_sizes - this function determines the stream interface urb
 *	configuration
 *
 * @stream: pointer to the stream interface
 * @config_stream: pointer to the struct with stream configuration data
 *	from usermode
 * @req_leader_size: minimum required leader size, which was previously
 *	read from a camera register
 * @req_trailer_size: minimum required trailer size, which was previously
 *	read from a camera register
 */
static int calculate_urb_sizes(struct u3v_stream *stream,
	struct u3v_configure_stream2 *config_stream,
	u32 req_leader_size, u32 req_trailer_size)
{
	struct u3v_device *u3v = stream->u3v_dev;
	struct device *dev = u3v->device;
	u32 byte_alignment = u3v->u3v_info->transfer_alignment;
	u32 payload_size = 0;
	u32 payload_count = 0;
	u32 chunk_payload_count = 0;
	u32 transfer1_size = 0;
	u32 max_pglist_count = 0;
	u32 transfer2_size = 0;
	u32 transfer2_data_size = 0;
	u32 max_transfer_size = 0;
	u32 aligned_max_transfer_size = 0;
	u32 aligned_cd_buffer_size = 0;
	u32 max_leader_size = 0;
	u32 max_trailer_size = 0;
	u32 num_segments = 0;
	u32 alignment_padding = 0;
	u32 unaligned_bytes = 0;
	u32 max_urb_size = config_stream->max_urb_size;
	u32 segment_padding = config_stream->segment_padding;
	u64 segment_size = config_stream->segment_size;
	u64 total_transfer_size = 0;
	u64 host_image_buffer_size = config_stream->image_buffer_size;
	u64 device_image_buffer_size = 0;
	u64 chunk_data_buffer_size = config_stream->chunk_data_buffer_size;
	u64 image_bytes_left = 0;
	u64 chunk_bytes_left = 0;
	u64 total_bytes_left = 0;
	bool sg_constraint = false;
	int ret = 0;

	if (host_image_buffer_size == 0) {
		dev_err(dev,
			"%s: Error, image buffer size must be greater than 0\n",
			__func__);
		return -EINVAL;
	}

	if (segment_size == 0 || segment_size > host_image_buffer_size) {
		dev_err(dev,
			"%s: Error, segment size is %llu, expected between 1 and %llu\n",
			__func__, segment_size, host_image_buffer_size);
		return -EINVAL;
	}
	/*
	 * Go down the sg constraint path unless sysfs attribute to enable
	 *	segmented transfers is set.
	 */
	sg_constraint = !(u3v->u3v_info->segmented_xfer_supported &&
		stream->u3v_dev->u3v_info->segmented_xfer_enabled);

	if (sg_constraint && (segment_padding != 0)) {
		dev_err(dev,
			"%s: segment padding is %u, but segmented transfers are not supported on this system\n",
			__func__, segment_padding);
		return U3V_ERR_SEGMENTED_XFER_NOT_AVAIL;
	}

	num_segments = div64_u64(host_image_buffer_size,
		(segment_size + segment_padding));
	device_image_buffer_size = num_segments * segment_size;

	image_bytes_left = device_image_buffer_size;

	max_transfer_size = min(u3v->u3v_info->os_max_transfer_size,
		max_urb_size);
	aligned_max_transfer_size = rounddown(max_transfer_size,
		byte_alignment);
	max_leader_size = min(roundup(req_leader_size, byte_alignment),
		aligned_max_transfer_size);
	max_trailer_size = min(roundup(req_trailer_size, byte_alignment),
		aligned_max_transfer_size);
	payload_size = aligned_max_transfer_size;

	if (sg_constraint) {
		payload_count =
			div_u64(device_image_buffer_size, payload_size);
		image_bytes_left -= payload_count * payload_size;
		transfer1_size =
			u3v_u64_rounddown(image_bytes_left, byte_alignment);
		image_bytes_left -= transfer1_size;
		transfer2_size =
			u3v_u64_roundup(image_bytes_left, byte_alignment);

		/* If there is a separate chunk data buffer, then the chunk
		 * data size will be > 0. In this case, image data must fit
		 * into the transfer buffers or the final transfer 1 payload.
		 * If final transfer 1 is being used for image data, then the
		 * chunk data must fit into a single URB. Chunk data does not
		 * have to be aligned because it's a small amount of data
		 * compared to the image. In that case we are okay using the
		 * final transfer 2 buffer for the transfer and then copying
		 * the data into the chunk data buffer.
		 */
		if (chunk_data_buffer_size != 0) {
			if (transfer2_size != 0) {
				dev_err(dev,
					"%s: Cannot split the image and chunk data into separate buffers because the image size is not a multiple of the required transfer alignment size\n",
					__func__);
				return U3V_ERR_IMAGE_SIZE_NOT_ALIGNED;
			}

			aligned_cd_buffer_size = u3v_u64_roundup(
				chunk_data_buffer_size, byte_alignment);
			if (transfer1_size != 0 &&
			   (aligned_cd_buffer_size >
			    aligned_max_transfer_size)) {
				dev_err(dev, "%s: Chunk data size too big\n",
				__func__);
				return U3V_ERR_CHUNK_DATA_SIZE_TOO_BIG;
			}
			/*
			 * If all of the image data is in the equally sized
			 * payload transfers, the chunk data can potentially
			 * use equally sized transfers as well as transfers
			 * 1 and 2
			 */
			if (transfer1_size == 0) {
				chunk_bytes_left = chunk_data_buffer_size;
				chunk_payload_count = div_u64(chunk_bytes_left,
					payload_size);
				payload_count += chunk_payload_count;
				chunk_bytes_left -=
					chunk_payload_count * payload_size;
				transfer1_size = u3v_u64_rounddown(
					chunk_bytes_left, byte_alignment);
				chunk_bytes_left -= transfer1_size;
				transfer2_size = u3v_u64_roundup(
					chunk_bytes_left, byte_alignment);
			} else {
				transfer2_size = aligned_cd_buffer_size;
			}

		}
		max_pglist_count = 1;
	} else {
		/*
		 * If we don't have restrictions on the size of scatterlist
		 * entries, we can have multiple buffers in the same urb
		 */
		total_transfer_size = device_image_buffer_size +
			chunk_data_buffer_size;
		total_bytes_left = total_transfer_size;
		payload_size = aligned_max_transfer_size;
		payload_count = div_u64(total_transfer_size,
			payload_size);
		total_bytes_left -= payload_size * payload_count;

		/*
		 * If we don't meet the device's alignment requirements,
		 * we will need to allocate some extra kernel memory
		 * to tack on to the end
		 */
		unaligned_bytes = u3v_u64_mod(total_bytes_left,
			byte_alignment);
		alignment_padding = unaligned_bytes ?
			byte_alignment - unaligned_bytes : 0;
		transfer1_size = total_bytes_left ?
			u3v_u64_roundup(total_bytes_left, byte_alignment) : 0;
		transfer2_size = 0;

		/* There will at least be an image buffer pglist */
		max_pglist_count = 1;
		if (chunk_data_buffer_size > 0)
			max_pglist_count++;
		if (alignment_padding != 0)
			max_pglist_count++;
	}
	/*
	 * If the image size is smaller than the device's alignment
	 * requirements, update the segment_size to at least be
	 * as large as that minimum
	 */
	if (segment_padding == 0 && segment_size < byte_alignment)
		segment_size = byte_alignment;

	/*
	 * Calculate how much data we actually expect to be valid in the
	 * final transfer 2 buffer
	 */
	if ((device_image_buffer_size + chunk_data_buffer_size +
		alignment_padding) < ((payload_size * payload_count) +
		transfer1_size)) {

		dev_err(dev,
			"%s: buffer sizes are too small to hold all of the requested DMA payload data. Total buffer size = %llu, calculated DMA size is at least %u\n",
			__func__,
			(device_image_buffer_size + chunk_data_buffer_size +
			alignment_padding), ((payload_size * payload_count) +
			transfer1_size));
		return -EINVAL;

	}
	transfer2_data_size =
		((device_image_buffer_size + chunk_data_buffer_size +
		alignment_padding) - ((payload_size * payload_count) +
		transfer1_size));

	/* Validate calculated sizes */

	dev_dbg(dev, "%s: host image buffer size = %llu\n",
		__func__, host_image_buffer_size);
	dev_dbg(dev, "%s: device image buffer size = %llu\n",
		__func__, device_image_buffer_size);
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
	dev_dbg(dev, "%s: transfer2_data = %u\n",
		__func__, transfer2_data_size);
	dev_dbg(dev, "%s: max_pglist_count = %u\n",
		__func__, max_pglist_count);
	dev_dbg(dev, "%s: alignment_padding = %u\n",
		__func__, alignment_padding);
	dev_dbg(dev, "%s: segment_padding = %u\n",
		__func__, segment_padding);
	dev_dbg(dev, "%s: segment_size = %llu\n",
		__func__, segment_size);
	dev_dbg(dev, "%s: sg_constraint = %s\n",
		__func__, (sg_constraint ? "true" : "false"));

	if (device_image_buffer_size == 0 || max_leader_size == 0 ||
	    max_trailer_size == 0) {
		dev_err(dev,
			"%s: leader, trailer, and image buffer sizes cannot be 0\n",
			__func__);
		dev_err(dev, "\tdevice image buffer size is %llu\n",
			device_image_buffer_size);
		dev_err(dev, "\tmax leader buffer size is %u\n",
			max_leader_size);
		dev_err(dev, "\tmax trailer buffer size is %u\n",
			max_trailer_size);
		return -EINVAL;
	}
	if (transfer2_data_size > transfer2_size) {
		dev_err(dev,
			"%s: final transfer 2 data size (%u) exceeds the size of the buffer (%u)\n",
			__func__, transfer2_data_size, transfer2_size);
		return -EINVAL;
	}

	if ((device_image_buffer_size + chunk_data_buffer_size +
		alignment_padding) < ((payload_size * payload_count) +
		transfer1_size + transfer2_data_size)) {

		dev_err(dev,
			"%s: buffer sizes are too small to hold all of the requested DMA payload data. Total buffer size = %llu, calculated DMA size = %u\n",
			__func__,
			(device_image_buffer_size + chunk_data_buffer_size +
			alignment_padding), ((payload_size * payload_count) +
			transfer1_size + transfer2_data_size));
		return -EINVAL;
	}

	/* The buffer sizes are valid, so now we store them */
	stream->config.host_image_buffer_size = host_image_buffer_size;
	stream->config.device_image_buffer_size = device_image_buffer_size;
	stream->config.chunk_data_buffer_size = chunk_data_buffer_size;
	stream->config.max_leader_size = max_leader_size;
	stream->config.max_trailer_size = max_trailer_size;
	stream->config.payload_size = payload_size;
	stream->config.payload_count = payload_count;
	stream->config.transfer1_size = transfer1_size;
	stream->config.max_pglist_count = max_pglist_count;
	stream->config.transfer2_size = transfer2_size;
	stream->config.transfer2_data_size = transfer2_data_size;
	stream->config.alignment_padding = alignment_padding;
	stream->config.segment_padding = segment_padding;
	stream->config.segment_size = segment_size;
	stream->config.sg_constraint = sg_constraint;

	if (config_stream->u_max_leader_size != NULL) {
		ret = put_user(stream->config.max_leader_size,
			config_stream->u_max_leader_size);
		if (ret != 0) {
			dev_err(dev,
				"%s: Error copying max leader size to user\n",
				__func__);
			goto error;
		}
	}

	if (config_stream->u_max_trailer_size != NULL) {
		ret = put_user(stream->config.max_trailer_size,
			config_stream->u_max_trailer_size);
		if (ret != 0) {
			dev_err(dev,
				"%s: Error copying max trailer size to user\n",
				__func__);
			goto error;
		}
	}
	return ret;
error:
	/* caller already holds stream_info->interface_lock */
	u3v_destroy_stream(u3v);
	return ret;
}


/*
 * u3v_destroy_stream - destroys the stream interface
 *
 * @u3v: pointer to the u3v_device struct
 */
int u3v_destroy_stream(struct u3v_device *u3v)
{
	struct u3v_stream *stream;
	struct rb_node *node;
	struct u3v_buffer *entry;
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
		entry = rb_entry(node, struct u3v_buffer, node);
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
 * u3v_configure_buffer - creates the buffer context data, page locks the
 *	user mode image and chunk data buffers, and adds the buffer to the
 *	rbtree of available buffers.
 *
 * @stream: pointer to the stream interface struct
 * @u_image_buffer: user mode memory address of the image buffer to lock
 * @u_chunk_data_buffer: user mode memory address of the chunk data buffer
 *	to lock
 * @u_buffer_id: on return, this user pointer will point to the buffer id
 *	value for the newly created buffer
 */
int u3v_configure_buffer(struct u3v_stream *stream,
	void __user *u_image_buffer, void __user *u_chunk_data_buffer,
	__u64 __user *u_buffer_id)
{
	struct device *dev = NULL;
	int ret = 0;

	if (stream == NULL || u_image_buffer == NULL ||
		u_buffer_id == NULL)
		return -EINVAL;
	dev = stream->u3v_dev->device;

	if ((u_chunk_data_buffer == NULL &&
	     stream->config.chunk_data_buffer_size != 0) ||
	     (u_chunk_data_buffer != NULL &&
	     stream->config.chunk_data_buffer_size == 0)) {

		dev_err(dev,
			"%s: Invalid chunk data buffer address and size combination\n",
			__func__);
		return -EINVAL;
	}
	ret = create_buffer(stream, u_image_buffer,
		u_chunk_data_buffer, u_buffer_id);

	return ret;
}


/*
 * create_buffer - this function allocates and initializes the buffer
 *	context data, page locks the image buffer and chunk data buffer,
 *	allocates buffers for the leader and trailer URBs, and allocates
 *	a scratch buffer for the final transfer 2 URB if necessary
 *
 * @stream: pointer to the stream interface struct
 * @u_image_buffer: memory address of the user image buffer to be locked
 * @u_chunk_data_buffer: memory address of the user chunk data buffer to
 *	be locked
 * @u_buffer_id: on return this user pointer points to valid buffer id
 */
static int create_buffer(struct u3v_stream *stream,
			       void __user *u_image_buffer,
			       void __user *u_chunk_data_buffer,
			       __u64 __user *u_buffer_id)
{
	int i = 0;
	int ret = 0;
	struct u3v_buffer *entry = NULL;
	struct device *dev = stream->u3v_dev->device;

	entry = kzalloc(sizeof(struct u3v_buffer), GFP_KERNEL);
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

	entry->urb_info_array = kzalloc(sizeof(struct u3v_urb_info) *
		entry->urb_info_count, GFP_KERNEL);

	if (entry->urb_info_array == NULL) {
		kfree(entry);
		return -ENOMEM;
	}

	/* Initialize the common fields of all of the u3v_urb structs */
	for (i = 0; i < entry->urb_info_count; i++) {
		entry->urb_info_array[i].urb_index = i;
		entry->urb_info_array[i].num_pglists = 0;
		entry->urb_info_array[i].stream = stream;
		entry->urb_info_array[i].entry = entry;
		entry->urb_info_array[i].purb = usb_alloc_urb(0, GFP_KERNEL);
		if (entry->urb_info_array[i].purb == NULL) {
			ret = -ENOMEM;
			goto error;
		}
		entry->urb_info_array[i].pglists =
			kzalloc((stream->config.max_pglist_count *
			sizeof(struct u3v_pglist *)), GFP_KERNEL);
		if (entry->urb_info_array[i].pglists == NULL) {
			ret = -ENOMEM;
			goto error;
		}
		entry->urb_info_array[i].sgt = NULL;
		entry->urb_info_array[i].buffer_size = 0;
		entry->urb_info_array[i].expected_size = 0;
	}

	if (stream->config.sg_constraint)
		ret = configure_urbs_sg_constraint(stream, entry,
			u_image_buffer, u_chunk_data_buffer);
	else
		ret = configure_urbs(stream, entry, u_image_buffer,
			u_chunk_data_buffer);
	if (ret != 0) {
		dev_err(dev, "%s: Configuring urbs failed with error %d\n",
			__func__, ret);
		goto error;
	}

	for (i = 0; i < entry->urb_info_count; i++) {
		ret = create_sglist(stream, &entry->urb_info_array[i]);
		if (ret != 0) {
			dev_err(dev,
				"%s: Creating sglist failed with error %d\n",
				__func__, ret);
			goto error;
		}
	}

	init_completion(&entry->buffer_complete);
	complete_all(&entry->buffer_complete);
	atomic_set(&entry->callbacks_received, 0);
	entry->state = u3v_idle;
	entry->buffer_id = stream->next_buffer_id++;
	put_user(entry->buffer_id, u_buffer_id);
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
 * configure_urbs - this function initializes the urb_info structs with
 *	corresponding pglists and sglists with the provided buffers.
 *
 * precondition: this function can only be called if sg_constraint is false
 * @stream: pointer to the stream interface struct
 * @entry: buffer entry in which to configure the urbs
 * @u_image_buffer: memory address of the user image buffer to be locked
 * @u_chunk_data_buffer: memory address of the user chunk data buffer to
 *	be locked
 */
static int configure_urbs(struct u3v_stream *stream,
	struct u3v_buffer *entry, void __user *u_image_buffer,
	void __user *u_chunk_data_buffer)
{
	int ret = 0;
	u32 urb_idx = 0;
	u32 payload_idx = 0;
	u32 payload_bytes_remaining = 0;
	u32 transfer1_bytes_remaining = 0;
	u32 alignment_padding = 0;
	u32 buffer_size = 0;
	u64 host_image_offset = 0;
	u64 chunk_data_offset = 0;
	u64 device_image_offset = 0;
	u64 device_bytes_remaining = 0;

	if (entry == NULL)
		return -EINVAL;

	alignment_padding = stream->config.alignment_padding;

	/* leader */
	ret = add_buffer_to_urb(stream, &entry->urb_info_array[urb_idx++],
		NULL, 0, 0, stream->config.max_leader_size,
		sizeof(struct leader_header), false);
	if (ret != 0)
		goto exit;

	/* payloads */
	while (payload_idx < stream->config.payload_count) {
		payload_bytes_remaining = stream->config.payload_size;
		if (host_image_offset <
		    stream->config.host_image_buffer_size) {
			device_bytes_remaining =
				stream->config.device_image_buffer_size -
				device_image_offset;

			buffer_size = min((device_bytes_remaining),
				(u64)(stream->config.payload_size));

			ret = add_buffer_to_urb(stream,
				&entry->urb_info_array[urb_idx],
				u_image_buffer, host_image_offset,
				device_image_offset, buffer_size, buffer_size,
				true);
			if (ret != 0)
				goto exit;

			device_image_offset += buffer_size;
			host_image_offset += num_host_buffer_bytes(
				buffer_size, device_image_offset,
				stream->config.segment_padding,
				stream->config.segment_size);
			payload_bytes_remaining -= buffer_size;
		}
		if (payload_bytes_remaining && chunk_data_offset <
		    stream->config.chunk_data_buffer_size) {
			device_bytes_remaining =
				stream->config.chunk_data_buffer_size -
				chunk_data_offset;

			buffer_size = min((device_bytes_remaining),
				(u64)(payload_bytes_remaining));

			ret = add_buffer_to_urb(stream,
				&entry->urb_info_array[urb_idx],
				u_chunk_data_buffer, chunk_data_offset,
				chunk_data_offset, buffer_size, buffer_size,
				false);
			if (ret != 0)
				goto exit;

			chunk_data_offset += buffer_size;
			payload_bytes_remaining -= buffer_size;
		}
		if (payload_bytes_remaining != 0) {
			dev_err(stream->u3v_dev->device,
				"%s: Error, unexpected result for calculated payload transfer bytes\n",
				__func__);
			ret = U3V_ERR_INTERNAL;
			goto exit;
		}
		urb_idx++;
		payload_idx++;
	}

	/* transfer1 */
	device_bytes_remaining =
		stream->config.device_image_buffer_size -
		device_image_offset;
	transfer1_bytes_remaining = stream->config.transfer1_size;

	if (device_bytes_remaining) {
		if (device_bytes_remaining > transfer1_bytes_remaining) {
			dev_err(stream->u3v_dev->device,
				"%s: Error, transfer1 bytes remaining is %u, but we still have %llu image bytes remaining\n",
				__func__, transfer1_bytes_remaining,
				device_bytes_remaining);
			ret = U3V_ERR_INTERNAL;
			goto exit;
		}
		ret = add_buffer_to_urb(stream,
			&entry->urb_info_array[urb_idx],
			u_image_buffer, host_image_offset,
			device_image_offset, device_bytes_remaining,
			device_bytes_remaining, true);
		if (ret != 0)
			goto exit;
		host_image_offset += num_host_buffer_bytes(
			device_bytes_remaining,
			device_image_offset,
			stream->config.segment_padding,
			stream->config.segment_size);
		device_image_offset += device_bytes_remaining;
		transfer1_bytes_remaining -= device_bytes_remaining;
	}
	device_bytes_remaining =
		stream->config.chunk_data_buffer_size - chunk_data_offset;

	if (device_bytes_remaining) {
		if (u_chunk_data_buffer == NULL) {
			ret = -EINVAL;
			goto exit;
		}
		if (device_bytes_remaining > transfer1_bytes_remaining) {
			dev_err(stream->u3v_dev->device,
				"%s: Error, transfer1 bytes remaining is %u, but we still have %llu chunk data bytes remaining\n",
				__func__, transfer1_bytes_remaining,
				device_bytes_remaining);
			ret = U3V_ERR_INTERNAL;
			goto exit;
		}

		ret = add_buffer_to_urb(stream,
			&entry->urb_info_array[urb_idx],
			u_chunk_data_buffer, chunk_data_offset,
			chunk_data_offset, device_bytes_remaining,
			device_bytes_remaining, false);
		if (ret != 0)
			goto exit;
		chunk_data_offset += device_bytes_remaining;
		transfer1_bytes_remaining -= device_bytes_remaining;
	}
	if (alignment_padding > 0) {
		ret = add_buffer_to_urb(stream,
			&entry->urb_info_array[urb_idx], NULL, 0, 0,
			alignment_padding, 0, false);
		if (ret != 0)
			goto exit;
		transfer1_bytes_remaining -= alignment_padding;
	}
	if (transfer1_bytes_remaining != 0) {
		dev_err(stream->u3v_dev->device,
			"%s: Error, unexpected result for calculated final transfer 1 bytes\n",
				__func__);
		ret = U3V_ERR_INTERNAL;
		goto exit;
	}
	if (stream->config.transfer1_size)
		urb_idx++;

	/* trailer */
	ret = add_buffer_to_urb(stream, &entry->urb_info_array[urb_idx++],
		NULL, 0, 0, stream->config.max_trailer_size,
		sizeof(struct trailer_header), false);

	if ((urb_idx != entry->urb_info_count) ||
	     (host_image_offset != stream->config.host_image_buffer_size) ||
	     (device_image_offset != stream->config.device_image_buffer_size) ||
	     (chunk_data_offset != stream->config.chunk_data_buffer_size)) {
		dev_err(stream->u3v_dev->device,
			"%s: error configuring buffer sizes\n", __func__);
		ret = U3V_ERR_INTERNAL;
	}
exit:
	return ret;
}


/*
 * configure_urbs_sg_constraint - this function initializes the urb_info
 *	structs with corresponding pglists and sglists with the provided
 *	buffers.
 *
 * precondition: this function can only be called if sg_constraint is true
 * @stream: pointer to the stream interface struct
 * @entry: buffer entry in which to configure the urbs
 * @u_image_buffer: memory address of the user image buffer to be locked
 * @u_chunk_data_buffer: memory address of the user chunk data buffer to
 *	be locked
 */
static int configure_urbs_sg_constraint(struct u3v_stream *stream,
	struct u3v_buffer *entry, void __user *u_image_buffer,
	void __user *u_chunk_data_buffer)
{
	int i = 0;
	int ret = 0;
	u64 image_offset = 0;
	u64 chunk_data_offset = 0;
	void __user *u_transfer2_addr = NULL;

	if (entry == NULL)
		return -EINVAL;

	/* Initialize the leader */
	i = 0;
	ret = add_buffer_to_urb(stream, &entry->urb_info_array[i++],
		NULL, 0, 0, stream->config.max_leader_size,
		sizeof(struct leader_header), false);
	if (ret != 0)
		goto exit;

	/* Configure image payload transfers */
	while ((i < stream->config.payload_count + 1) &&
	       (image_offset < stream->config.device_image_buffer_size)) {
		ret = add_buffer_to_urb(stream, &entry->urb_info_array[i++],
			u_image_buffer, image_offset,
			image_offset, stream->config.payload_size,
			stream->config.payload_size, false);
		if (ret != 0)
			goto exit;

		image_offset += stream->config.payload_size;
	}
	/*
	 * If we account for the total image size before getting to
	 * the last payload transfer then switch to the chunk data
	 * buffer for the remaining payload transfer requests
	 * We add 1 when checking if we're done with the payload
	 * because i includes the leader for the first urb.
	 */
	while (i < stream->config.payload_count + 1) {
		ret = add_buffer_to_urb(stream, &entry->urb_info_array[i++],
			u_chunk_data_buffer, chunk_data_offset,
			chunk_data_offset, stream->config.payload_size,
			stream->config.payload_size, false);
		if (ret != 0)
			goto exit;
		chunk_data_offset += stream->config.payload_size;
	}

	/*
	 * Configure the final transfer 1 buffer if applicable. If we
	 * haven't already finished requesting all of the image data
	 * then use the image buffer. Otherwise, use the chunk data buffer.
	 */
	if (stream->config.transfer1_size != 0) {
		if (image_offset < stream->config.device_image_buffer_size) {
			ret = add_buffer_to_urb(stream,
				&entry->urb_info_array[i++],
				u_image_buffer,
				image_offset, image_offset,
				stream->config.transfer1_size,
				stream->config.transfer1_size, false);
			if (ret != 0)
				goto exit;
			image_offset += stream->config.transfer1_size;
		} else {
			ret = add_buffer_to_urb(stream,
				&entry->urb_info_array[i++],
				u_chunk_data_buffer,
				chunk_data_offset, chunk_data_offset,
				stream->config.transfer1_size,
				stream->config.transfer1_size, false);
			if (ret != 0)
				goto exit;
			chunk_data_offset += stream->config.transfer1_size;
		}
	}

	entry->u_transfer2_addr = NULL;
	u_transfer2_addr = NULL;
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
			ret = add_buffer_to_urb(stream,
				&entry->urb_info_array[i++], NULL, 0, 0,
				stream->config.transfer2_size,
				stream->config.transfer2_data_size, false);
			if (ret != 0)
				goto exit;

			if (image_offset <
			    stream->config.device_image_buffer_size) {
				u_transfer2_addr =
					(u8 *)(u_image_buffer) +
					image_offset;
				image_offset +=
					stream->config.transfer2_data_size;
			} else {
				u_transfer2_addr =
					(u8 *)(u_chunk_data_buffer) +
					chunk_data_offset;
				chunk_data_offset +=
					stream->config.transfer2_data_size;
			}
			entry->u_transfer2_addr = u_transfer2_addr;
		} else {
			if (image_offset <
			    stream->config.device_image_buffer_size) {
				ret = add_buffer_to_urb(stream,
					&entry->urb_info_array[i++],
					u_image_buffer,
					image_offset, image_offset,
					stream->config.transfer2_size,
					stream->config.transfer2_data_size,
					false);
				if (ret != 0)
					goto exit;
				image_offset +=
					stream->config.transfer2_data_size;
			} else {
				ret = add_buffer_to_urb(stream,
					&entry->urb_info_array[i++],
					u_chunk_data_buffer,
					chunk_data_offset, chunk_data_offset,
					stream->config.transfer2_size,
					stream->config.transfer2_data_size,
					false);
				if (ret != 0)
					goto exit;
				chunk_data_offset +=
					stream->config.transfer2_data_size;
			}
		}
	}

	/* Initialize the trailer */
	ret = add_buffer_to_urb(stream, &entry->urb_info_array[i++],
		NULL, 0, 0, stream->config.max_trailer_size,
		sizeof(struct trailer_header), false);
	if (ret != 0)
		goto exit;

	if ((i != entry->urb_info_count) ||
	     (image_offset != stream->config.device_image_buffer_size) ||
	     (chunk_data_offset != stream->config.chunk_data_buffer_size)) {

		dev_err(stream->u3v_dev->device,
			"%s: error configuring buffer sizes\n", __func__);
		ret = U3V_ERR_INTERNAL;
	}

exit:
	return ret;
}


/*
 * num_host_buffer_bytes - helper function that uses segment information to
 *	compute the size of the user buffer from the host's perspective. Since
 *	this function is called in the context of calculating the host buffer
 *	bytes for a single urb, it is safe to use u32 values for device_bytes
 *	since the max urb size is a u32.
 *
 * @device_bytes: number of image bytes from the camera's perspective
 * @segment_padding: number of bytes to skip over between segments when
 *	configuring the scatterlist
 * @segment_size: number of bytes in each line of the image
 */
static u32 num_host_buffer_bytes(u32 device_bytes, u64 device_image_offset,
	u32 segment_padding, u64 segment_size)
{
	u32 device_byte_count = 0;
	u32 host_byte_count = 0;
	u32 current_segment_bytes = 0;
	u64 num_segments = div64_u64(device_image_offset, segment_size);
	u64 segment_bytes_remaining =
		device_image_offset - (num_segments * segment_size);
	while (device_byte_count < device_bytes) {
		current_segment_bytes = min(segment_bytes_remaining,
			(u64)(device_bytes - device_byte_count));
		device_byte_count += current_segment_bytes;
		host_byte_count += current_segment_bytes;
		if (device_byte_count != device_bytes) {
			host_byte_count += segment_padding;
			segment_bytes_remaining = segment_size;
		}
	}
	return host_byte_count;
}


/*
 * add_buffer_to_urb - this function takes a user or kernel-allocated
 *	buffer and makes a pglist for it, page-locking if necessary. It
 *	also adjusts the urb_info data to account for additional buffer.
 *	If sg_constraint is set, this function can only be called once per
 *	urb. Otherwise, the urb can support an arbitrary number of
 *	discontinuous buffers.
 *
 * @stream: pointer to the stream interface struct
 * @urb_info: on return this struct will be initialized
 * @u_buffer: user mode memory address of the buffer to lock
 * @buffer_size: size of the user buffer
 * @offset: offset into the user buffer
 * @urb_buffer_size: size of the URB buffer
 * @expected_size: amount of data we expect to receive
 */
static int add_buffer_to_urb(struct u3v_stream *stream,
	struct u3v_urb_info *urb_info, void __user *u_buffer,
	u64 host_offset, u64 device_offset, u32 buffer_size,
	u32 expected_size, bool use_padding)
{
	u32 padding = 0;

	if (urb_info == NULL || buffer_size == 0)
		return -EINVAL;

	if (urb_info->num_pglists + 1 > stream->config.max_pglist_count)
		return U3V_ERR_INTERNAL;

	padding = use_padding ? stream->config.segment_padding : 0;

	urb_info->pglists[urb_info->num_pglists] =
		create_pglist(stream, u_buffer, host_offset, device_offset,
		buffer_size, padding, min(stream->config.segment_size,
		(u64)(buffer_size)));

	if (urb_info->pglists[urb_info->num_pglists] == NULL)
		return U3V_ERR_INTERNAL;

	urb_info->num_pglists++;
	urb_info->buffer_size += buffer_size;
	urb_info->expected_size += expected_size;
	return 0;
}


/*
 * create_pglist - this function creates a struct u3v_pglist
 *	out of kernel or user memory of size num_bytes. It generates
 *	a list of page-locked memory and its associated metadata
 *
 * @stream: pointer to the stream interface struct
 * @u_buffer: user buffer to be locked in memory
 * @host_offset: offset in host image buffer
 * @device_offset: offset in device image buffer
 * @num_bytes: buffer size
 * @segment_padding: number of bytes to skip over between segments when
 *	configuring the scatterlist
 * @segment_size: number of bytes in each line of the image
 * @return: pointer to the page_list if successful, NULL otherwise.
 */
static struct u3v_pglist *create_pglist(struct u3v_stream *stream,
	void __user *u_buffer, u64 host_offset, u64 device_offset,
	u32 num_bytes, u32 segment_padding, u64 segment_size)
{
	struct device *dev = stream->u3v_dev->device;
	u8 *adjusted_u_buffer = NULL;
	void *kernel_buffer = NULL;
	struct u3v_pglist *pglist = kzalloc(sizeof(struct u3v_pglist),
		GFP_KERNEL);

	if (pglist == NULL)
		return NULL;

	/* Initialize common fields */
	pglist->segmented_xfer = segment_padding != 0;
	pglist->num_device_bytes = num_bytes;
	pglist->host_offset = host_offset;
	pglist->device_offset = device_offset;

	/* No user buffer provided, so allocate one here */
	if (u_buffer == NULL) {
		/*
		 * Verify that the offsets are 0 if we are allocating
		 * in the kernel.
		 */
		if (pglist->host_offset != 0 || pglist->device_offset != 0)
			goto error;
		pglist->kernel_allocated = true;
		pglist->num_host_bytes = 0;
		kernel_buffer = kzalloc(num_bytes, GFP_KERNEL);
		if (kernel_buffer == NULL)
			goto error;
		pglist->num_pages = get_num_pages(kernel_buffer,
			num_bytes);
		pglist->pages = kernel_addr_to_pages(kernel_buffer,
			pglist->num_pages);
		pglist->page_offset =
			((uintptr_t)(kernel_buffer) & ~PAGE_MASK);
	/* Page-lock user memory */
	} else {
		adjusted_u_buffer = (u8 *)(u_buffer) + host_offset;
		pglist->kernel_allocated = false;
		pglist->num_host_bytes = num_host_buffer_bytes(num_bytes,
			device_offset, segment_padding, segment_size);
		pglist->num_pages = get_num_pages(adjusted_u_buffer,
			pglist->num_host_bytes);
		pglist->pages = lock_user_pages(stream, adjusted_u_buffer,
			pglist->num_host_bytes, pglist->num_pages);
		pglist->page_offset =
			((uintptr_t)(adjusted_u_buffer) & ~PAGE_MASK);
	}

	if (pglist->pages == NULL)
		goto error;

	return pglist;
error:
	dev_err(dev, "%s: Error creating pglist with %s buffer\n",
		__func__, (pglist->kernel_allocated ? "kernel" : "user"));
	/* 0 because pages are not dirty */
	destroy_pglist(pglist, 0);
	return NULL;
}


/*
 * lock_user_pages - this function takes a pointer to num_bytes
 *	of contiguous virtual user memory, faults in any pages, then
 *	locks them in memory.
 *
 * @stream: pointer to the stream interface struct
 * @u_buffer: user virtual address
 * @num_bytes: buffer size in bytes
 * @num_pages: number of physical pages this buffer consists of
 * @return: a pointer to an array of locked pages if successful,
 *	NULL otherwise.
 */
static struct page **lock_user_pages(struct u3v_stream *stream,
	void __user *u_buffer, u32 num_bytes, u32 num_pages)
{
	int ret = 0;
	int i = 0;
	struct page **pages = NULL;
	struct device *dev = stream->u3v_dev->device;
	uintptr_t uaddr = (uintptr_t)(u_buffer);

	/* Check for overflow */
	if ((uaddr + num_bytes) < uaddr)
		return NULL;

	/* No point continuing in this case */
	if (num_bytes == 0)
		return NULL;

	pages = kmalloc((num_pages * sizeof(struct page *)), GFP_KERNEL);
	if (pages == NULL)
		return NULL;

	/* Fault in all of the necessary pages */
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0)
	down_read(&current->mm->mmap_sem);
#else
	down_read(&current->mm->mmap_lock);
#endif

/* will store a page locked array of physical pages in pages var */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 6, 0)
	ret = get_user_pages(current, current->mm, uaddr, num_pages, WRITE, 0, pages, NULL);
#elif LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0)
	ret = get_user_pages(uaddr, num_pages, WRITE, 0, pages, NULL);
#elif LINUX_VERSION_CODE < KERNEL_VERSION(6, 5, 0)
	// replace get_user_pages() write/force parameters with gup_flags in Commit 768ae30
	ret = get_user_pages(uaddr, num_pages, FOLL_WRITE, pages, NULL);
#else
	// vmas parameter removed from get_user_pages() in Commit 54d0206
	ret = get_user_pages(uaddr, num_pages, FOLL_WRITE, pages);
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0)
	up_read(&current->mm->mmap_sem);
#else
	up_read(&current->mm->mmap_lock);
#endif

	if (ret < num_pages) {
		dev_err(dev, "%s: get_user_pages returned %d, expected %d\n",
			__func__, ret, num_pages);
		goto err_unlock;
	}

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
 * create_sglist - this function creates a scatter-gather list from
 *	a urb's page list(s)
 *
 * @stream: pointer to the stream interface struct
 * @urb_info: pointer to urb info with necessary information to create the
 *	scatterlist from
 */
static int create_sglist(struct u3v_stream *stream,
	struct u3v_urb_info *urb_info)
{
	int ret = 0;
	u32 total_sgs = 0;

	if (urb_info == NULL)
		return -EINVAL;

	/* don't configure, just calculate number of entries for allocation */
	ret = calculate_sglist_entries(stream, urb_info, &total_sgs, false);
	if (ret != 0)
		return ret;

	urb_info->sgt = kmalloc(sizeof(struct sg_table), GFP_KERNEL);
	if (urb_info->sgt == NULL)
		return -ENOMEM;

	ret = sg_alloc_table(urb_info->sgt, total_sgs, GFP_KERNEL);
	if (ret != 0) {
		kfree(urb_info->sgt);
		urb_info->sgt = NULL;
		return ret;
	}

	/* now that the sgtable is allocated, initialize the sglist entries */
	ret = calculate_sglist_entries(stream, urb_info, NULL, true);
	if (ret != 0) {
		sg_free_table(urb_info->sgt);
		kfree(urb_info->sgt);
		urb_info->sgt = NULL;
	}

	return ret;

}


/*
 * calculate_sglist_entries - this function calculates the number of
 *	scatter-gather list elements for one or more pglists for a given
 *	urb. If the configure parameter is true, it also configures the list.
 *
 * @stream: pointer to the stream interface
 * @urb_info: pointer to struct containing a urb pointer and its metadata
 * @num_sgs: pointer to variable where this function will store the
 *	total number of scatterlist entries needed
 * @configure: if true, this function sets up the previously allocated
 *	scatter-gather list
 */
static int calculate_sglist_entries(struct u3v_stream *stream,
	struct u3v_urb_info *urb_info, u32 *num_sgs, bool configure)
{
	u32 i = 0;
	u32 pg_idx = 0;
	u32 sg_count = 0;
	u32 pglist_bytes_remaining = 0;
	u32 segment_offset = 0;
	u32 segment_bytes_remaining = 0;
	u32 limit = 0;
	u32 offset = 0;
	int bytes_to_transfer = 0;
	struct scatterlist *cur = NULL;
	struct u3v_pglist *pglist = NULL;
	struct usb_endpoint_descriptor *stream_endpoint =
		stream->u3v_dev->stream_info.bulk_in;
	u32 w_max_packet_size = usb_endpoint_maxp(stream_endpoint);

	/* the table and entries need to have already been allocated */
	if (urb_info->pglists == NULL)
		return U3V_ERR_INTERNAL;

	if (configure) {
		if (urb_info->sgt == NULL ||
		    urb_info->sgt->sgl == NULL)
			return U3V_ERR_INTERNAL;

		cur = urb_info->sgt->sgl;
	}

	sg_count = 0;

	for (i = 0; i < urb_info->num_pglists; i++) {
		pglist = urb_info->pglists[i];
		pg_idx = 0;
		offset = pglist->page_offset;

		pglist_bytes_remaining = pglist->num_device_bytes;
		if (pglist->segmented_xfer) {
			segment_offset = u3v_u64_mod(pglist->device_offset,
				stream->config.segment_size);
			segment_bytes_remaining =
				stream->config.segment_size - segment_offset;
		} else {
			segment_offset = 0;
			segment_bytes_remaining = pglist->num_device_bytes;
		}

		while (pglist_bytes_remaining != 0) {

			/* find the maximum bytes for this sglist entry */
			limit = min(segment_bytes_remaining,
				pglist_bytes_remaining);
			/*
			 * get the minimum of the amount left, the amount left
			 * in a page, and the amount left in a line
			 */
			bytes_to_transfer = num_contiguous_bytes(pglist,
				pg_idx, offset, limit);
			if (bytes_to_transfer < 0)
				return bytes_to_transfer;
			if (stream->config.sg_constraint) {
				if (bytes_to_transfer > w_max_packet_size) {
					bytes_to_transfer = rounddown(
						bytes_to_transfer,
						w_max_packet_size);
				} else if (bytes_to_transfer < w_max_packet_size
					&& !(sg_count == 0 ||
					bytes_to_transfer ==
					pglist_bytes_remaining)) {
					/*
					 * if we're in the constraint case,
					 * the scatterlist can be unaligned
					 * with w_max_packet_size only if it's
					 * the first or last entry in the list
					 */
					dev_err(stream->u3v_dev->device,
						"%s: invalid buffer: if sg_constraint is true, a buffer can only have a scatterlist entry smaller than w_max_packet_size if it's the first or last in the list\n",
						__func__);
					return U3V_ERR_INTERNAL;
				}
			}
			segment_bytes_remaining -= bytes_to_transfer;
			if (configure) {
				sg_set_page(cur, pglist->pages[pg_idx],
					bytes_to_transfer, offset);
				cur = sg_next(cur);
			}
			/* advance the offset past this segment and padding */
			offset += bytes_to_transfer;

			if (pglist->segmented_xfer &&
			    segment_bytes_remaining == 0) {
				/*
				 * skip over the padding only when we've
				 * completed a segment
				 */
				offset += stream->config.segment_padding;
				/*
				 * we can transfer the whole segment, so
				 * reset for the next one
				 */
				segment_bytes_remaining =
					stream->config.segment_size;
			}
			/*
			 * increment the page index if we crossed
			 * a page boundary
			 */
			pg_idx += offset / PAGE_SIZE;
			/* adjust the offset to be within a single page */
			offset %= PAGE_SIZE;
			sg_count++;
			pglist_bytes_remaining -= bytes_to_transfer;
		}
	}
	if (configure && sg_count != urb_info->sgt->nents) {
		dev_err(stream->u3v_dev->device,
			"%s: Error configuring sglist: counted %u elements, expected %u\n",
				__func__, sg_count, urb_info->sgt->nents);
		return U3V_ERR_INTERNAL;
	}
	if (num_sgs != NULL)
		*num_sgs = sg_count;
	return 0;
}


/*
 * num_contiguous_bytes - helper function that calculates the maximum
 *		number of contiguous bytes in physical memory up to the
 *		specified limit
 *
 * @pglist: pointer to the pglist struct with memory information
 * @pg_idx: index of the page to start with
 * @offset: starting offset within the initial page
 * @limit: maximum number of bytes to look ahead
 * @return: number of contiguous bytes if successful,
 *		negative error value otherwise
 */
static int num_contiguous_bytes(struct u3v_pglist *pglist, u32 pg_idx,
				u32 offset, u32 limit)
{
	u32 bytes_to_add = 0;
	u32 contiguous_bytes = 0;
	u32 max_bytes_remaining = 0;
	u64 cur_pfn = 0;
	u64 next_pfn = 0;
	bool adjacent = true;

	if (pg_idx >= pglist->num_pages ||
	    offset >= PAGE_SIZE)
		return U3V_ERR_INTERNAL;

	contiguous_bytes = min(limit, (u32)(PAGE_SIZE - offset));
	max_bytes_remaining = limit - contiguous_bytes;

	while (adjacent && (max_bytes_remaining > 0) &&
		((pg_idx + 1) < pglist->num_pages)) {

		cur_pfn = page_to_pfn(pglist->pages[pg_idx]);
		next_pfn = page_to_pfn(pglist->pages[++pg_idx]);
		adjacent = (next_pfn == (cur_pfn + 1));
		if (adjacent) {
			bytes_to_add =
				min(max_bytes_remaining, (u32)(PAGE_SIZE));
			contiguous_bytes += bytes_to_add;
			max_bytes_remaining = limit - contiguous_bytes;
		}
	}
	return contiguous_bytes;
}


/*
 * u3v_unconfigure_buffer - finds the buffer with buffer_id,
 * unlocks corresponding pagelists, and frees all resources associated
 * with the buffer entry.
 *
 * @stream: pointer to the u3v_stream interface struct
 * @buffer_id: id of the buffer to be destroyed
 */
int u3v_unconfigure_buffer(struct u3v_stream *stream, u64 buffer_id)
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
 *
 * precondition: caller must hold stream_lock
 * @stream: pointer to the u3v_stream interface struct
 * @buffer_id: id of the buffer to be destroyed
 */
static int destroy_buffer(struct u3v_stream *stream, u64 buffer_id)
{
	struct u3v_buffer *entry;
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
 *
 * precondition: caller must hold stream_lock
 * @urb_info: pointer to struct containing a urb pointer and its metadata
 */
static void destroy_urb(struct u3v_urb_info *urb_info)
{
	int i = 0;
	if (urb_info == NULL)
		return;

	usb_free_urb(urb_info->purb);
	for (i = 0; i < urb_info->num_pglists; i++)
		destroy_pglist(urb_info->pglists[i], true);
	destroy_sglist(urb_info);
}


/*
 * destroy_pglist - helper function that unlocks the pages pinned from
 *	user memory and frees corresponding sglist resources
 *
 * precondition: caller must hold stream_lock or buffer has not yet been
 *	added to the list
 * @pglist: pointer to page_list to be destroyed
 * @dirty: true if data needs to be written out to disk
 */
static void destroy_pglist(struct u3v_pglist *pglist, bool dirty)
{
	if (pglist == NULL)
		return;

	if (pglist->kernel_allocated) {
		kfree(pglist_to_kernel_addr(pglist));
		kfree(pglist->pages);
	} else {
		unlock_user_pages(pglist, dirty);
	}
	kfree(pglist);
}


/*
 * unlock_user_pages - helper function that unlocks user memory that was
 *	previously pagelocked, marking the page as dirty if necessary.
 *
 * @pglist: pointer to the page_list to be unpinned
 * @dirty: true if data needs to be written out to disk
 */
static void unlock_user_pages(struct u3v_pglist *pglist, int dirty)
{
	int i = 0;

	if (pglist == NULL || pglist->pages == NULL)
		return;

	for (i = 0; i < pglist->num_pages; i++) {
		struct page *page = pglist->pages[i];
		if (dirty)
			set_page_dirty_lock(page);
		put_page(page);
	}
	kfree(pglist->pages);
}


/*
 * destroy_sglist - helper function that destroys a scatter-gather list in
 *	a page_list struct
 *
 * @pglist: pointer to page_list with the sglist to be destroyed
 */
static void destroy_sglist(struct u3v_urb_info *urb_info)
{
	if (urb_info == NULL || urb_info->sgt == NULL)
		return;

	sg_free_table(urb_info->sgt);
	kfree(urb_info->sgt);
	urb_info->sgt = NULL;
}


/*
 * u3v_queue_buffer - queues the buffer to the camera for it to be filled
 *	with image data
 *
 * @stream: pointer to the stream interface struct
 * @buffer_id: on return, buffer with this id is queued
 */
int u3v_queue_buffer(struct u3v_stream *stream, u64 buffer_id)
{
	struct u3v_buffer *entry = NULL;
	struct device *dev = NULL;
	int ret = 0;
	int i = 0;

	if (stream == NULL)
		return -EINVAL;

	dev = stream->u3v_dev->device;
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
 *
 * precondition: caller must hold stream_lock
 * @entry: buffer to be reset
 */
static void reset_counters(struct u3v_buffer *entry)
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
 *
 * precondition: caller must hold stream_lock
 * @stream: pointer to the stream interface struct
 * @urb_info: pointer to the urb_info struct that contains the
 *	urb to be submitted
 */
static int submit_stream_urb(struct u3v_stream *stream,
			     struct u3v_urb_info *urb_info)
{
	struct device *dev = stream->u3v_dev->device;
	struct usb_device *udev = stream->u3v_dev->udev;
	int ret = 0;

	usb_fill_bulk_urb(urb_info->purb, udev,
		usb_rcvbulkpipe(udev,
		usb_endpoint_num(stream->u3v_dev->stream_info.bulk_in)),
		NULL, urb_info->buffer_size,
		stream_urb_completion, urb_info);

	/* Add scatterlist information */
	urb_info->purb->sg = urb_info->sgt->sgl;
	urb_info->purb->num_sgs = urb_info->sgt->nents;

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
 *
 * @purb: pointer to the urb that has been completed
 */
static void stream_urb_completion(struct urb *purb)
{
	u32 len = 0;
	u32 urb_count = 0;
	u32 idx = 0;
	struct u3v_urb_info *urb_info = NULL;
	struct u3v_buffer *entry = NULL;
	struct device *dev = NULL;

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
	urb_info = (struct u3v_urb_info *)(purb->context);
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
	if (entry->status != 0)
		entry->status = purb->status;
	len = purb->actual_length;

	WARN_ON(entry->state != u3v_queued);
	if (len < urb_info->expected_size) {
		dev_dbg(dev,
			"%s: entry %llu, urb %u: length = %u, expected >=%u\n",
			__func__, entry->buffer_id, urb_info->urb_index, len,
			urb_info->expected_size);
		entry->incomplete_callbacks_received++;
	}

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
		    (entry->u_transfer2_addr != NULL)) {
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
 *
 * @stream: pointer to the stream interface
 * @buffer_id: the buffer to be waited on
 * @u_leader_buffer: if not NULL, the leader information will be copied
 *	to this buffer on return
 * @u_leader_size: size of user_leader_buffer on input, and number of bytes
 *	copied on return. Can be NULL if user_leader_buffer is NULL
 * @u_trailer_buffer: if not NULL, the trailer information will be
 *	copied to this buffer on return
 * @u_trailer_size: size of the user_trailer_buffer on input, and number
 *	of bytes copied on return. Can be NULL if user_trailer_buffer is NULL
 * @u_buffer_complete_data: additional data describing the completed buffer,
 *	can be NULL.
 */
int u3v_wait_for_buffer(struct u3v_stream *stream, u64 buffer_id,
	void __user *u_leader_buffer, __u32 __user *u_leader_size,
	void __user *u_trailer_buffer, __u32 __user *u_trailer_size,
	void __user *u_buffer_complete_data)
{
	int ret = 0;
	int leader_index = 0;
	int transfer2_index = 0;
	int trailer_index = 0;
	u32 leader_size = 0;
	u32 trailer_size = 0;
	struct u3v_buffer *entry = NULL;
	struct device *dev = NULL;
	struct buffer_complete_data bcd;

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

	if (u_leader_buffer != NULL && u_leader_size != NULL) {
		get_user(leader_size, u_leader_size);
		ret = copy_to_user(u_leader_buffer,
			pglist_to_kernel_addr(
			entry->urb_info_array[leader_index].pglists[0]),
			min(entry->leader_received_size, leader_size));
		if (ret != 0) {
			dev_err(dev, "%s: Error copying leader buffer\n",
				__func__);
			goto exit;
		}
	}
	if (u_leader_size != NULL)
		put_user(entry->leader_received_size, u_leader_size);

	if (entry->u_transfer2_addr != NULL) {
			ret = copy_to_user(entry->u_transfer2_addr,
				pglist_to_kernel_addr(
				entry->urb_info_array[transfer2_index].
				pglists[0]),
				min(entry->transfer2_received_size,
				entry->urb_info_array[transfer2_index].
				expected_size));
		if (ret != 0) {
			dev_err(dev,  "%s: Error copying transfer2 buffer\n",
				__func__);
			goto exit;
		}
	}

	if (u_trailer_buffer != NULL && u_trailer_size != NULL) {
		get_user(trailer_size, u_trailer_size);
		ret = copy_to_user(u_trailer_buffer,
			pglist_to_kernel_addr(
			entry->urb_info_array[trailer_index].pglists[0]),
			min(entry->trailer_received_size, trailer_size));
		if (ret != 0) {
			dev_err(dev,  "%s: Error copying trailer buffer\n",
				__func__);
			goto exit;
		}
	}

	if (u_trailer_size != NULL)
		put_user(entry->trailer_received_size, u_trailer_size);

	if (u_buffer_complete_data != NULL) {
		bcd.structure_size = sizeof(struct buffer_complete_data);
		bcd.status = entry->status;
		bcd.expected_urb_count = entry->urb_info_count;
		bcd.incomplete_urb_count =
			entry->incomplete_callbacks_received;
		bcd.payload_bytes_received = entry->payload_received_size;
		ret = copy_to_user(u_buffer_complete_data, &bcd,
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
 *
 * @stream: pointer to the stream interface struct
 */
int u3v_cancel_all_buffers(struct u3v_stream *stream)
{
	int ret = 0;

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
 * reset_stream - cancels any outstanding urbs and resets the stream
 *	bulk in endpoint
 *
 * precondition: caller holds stream_lock or has not yet set the stream
 *	interface pointer
 * @stream - pointer to the stream interface struct
 */
static int reset_stream(struct u3v_stream *stream)
{
	struct u3v_device *u3v = stream->u3v_dev;
	struct usb_device *udev = u3v->udev;
	u8 ep_addr = u3v->stream_info.bulk_in->bEndpointAddress;
	int dummy_size = 32;
	u8 dummy_buffer[dummy_size];
	int actual = 0;
	int ret = 0;

	if (!u3v->device_connected)
		return 0;

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
 * kernel_addr_to_pages - this function takes a kernel address and
 *	number of pages and creates an array of struct page * to
 *	correspond with the memory
 *
 * @addr: kernel logical address previously allocated by kmalloc/kzalloc
 * @num_pages: number of total pages to put in the array
 * @return: an num_pages-sized array of struct page * corresponding
 *	with the kernel memory if successful, NULL otherwise
 */
static struct page **kernel_addr_to_pages(void *addr, u32 num_pages)
{
	unsigned int i = 0;
	struct page **pages = NULL;
	u8 *cur = (u8 *)(addr);

	if (addr == NULL || num_pages == 0)
		return NULL;
	pages = kzalloc((num_pages * sizeof(struct page *)), GFP_KERNEL);
	if (pages == NULL)
		return NULL;

	for (i = 0; i < num_pages; i++) {
		if (virt_addr_valid(cur))
			pages[i] = virt_to_page(cur);
		if (pages[i] == NULL)
			goto error;
		cur += PAGE_SIZE;
	}
	return pages;
error:
	kfree(pages);
	return NULL;
}


/*
 * pglist_to_kernel_addr - this function takes a pglist and calculates the
 *	original kernel logical address allocated by kmalloc/kzalloc
 *
 * @pglist: pglist with relevant kernel memory information
 * @return: pointer to original kernel address if successful, NULL otherwise
 */
static void *pglist_to_kernel_addr(struct u3v_pglist *pglist)
{
	u8 *addr = NULL;
	if (!pglist->kernel_allocated || pglist->pages == NULL)
		return NULL;

	addr = page_address(pglist->pages[0]);
	addr += pglist->page_offset;
	return addr;
}


/*
 * search_buffer_entries - helper function that traverses the rb tree of
 *	buffer entries to find the one whose id matches the provided value.
 *
 * precondition: caller must hold stream_lock
 * @stream: pointer to the stream interface struct
 * @value: id of the buffer to be found
 * @return: pointer to the entry if found, NULL otherwise
 */
static struct u3v_buffer *search_buffer_entries(struct u3v_stream *stream,
						  u64 value)
{
	struct rb_node *node;

	/* start at top of the tree */
	node = stream->root.rb_node;

	while (node) {
		struct u3v_buffer *entry =
			rb_entry(node, struct u3v_buffer, node);
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
 *
 * precondition: caller must hold stream_lock
 * @stream: pointer to the stream interface struct
 * @new: buffer entry to be added to the rb tree
 */
static void buffer_entry_insert(struct u3v_stream *stream,
				struct u3v_buffer *new)
{
	struct u3v_buffer *entry;
	struct rb_node **link = &stream->root.rb_node;
	struct rb_node *parent = NULL;
	u64 value = new->buffer_id;

	/* Go to the bottom of the tree */
	while (*link) {
		parent = *link;
		entry = rb_entry(parent, struct u3v_buffer, node);
		if (entry->buffer_id > value)
			link = &(*link)->rb_left;
		else
			link = &(*link)->rb_right;
	}
	/* Now insert the new node, balance if necessary*/
	rb_link_node(&new->node, parent, link);
	rb_insert_color(&new->node, &stream->root);
}


/*
 * u3v_u64_mod - helper function that returns remainder for 64 bit /
 *	32 bit division
 */
static u32 u3v_u64_mod(u64 dividend, u32 divisor)
{
	u32 remainder;
	div_u64_rem(dividend, divisor, &remainder);
	return remainder;
}


/*
 * u3v_u64_roundup - helper function that rounds a 64 bit integer
 *	up to the nearest multiple of alignment
 */
static u64 u3v_u64_roundup(u64 value, u32 alignment)
{
	return div_u64(value + (alignment - 1), alignment) *
		alignment;
}


/*
 * u3v_u64_rounddown - helper function that rounds a 64 bit integer
 *	down to the nearest multiple of alignment
 */
static u64 u3v_u64_rounddown(u64 value, u32 alignment)
{
	return value - u3v_u64_mod(value, alignment);
}
