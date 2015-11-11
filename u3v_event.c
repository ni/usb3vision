/*
 * u3v_event.c: Driver for USB3 Vision(TM) class devices
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
#include "u3v_event.h"
#include "u3v_shared.h"
#include <linux/mm.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/usb.h>

enum event_state {
	event_idle,
	event_queued,
	event_complete
};

struct event_entry {
	struct list_head list_elem;
	struct u3v_event *event;
	u32 buffer_size;
	u32 buffer_received_size;
	enum event_state state;
	int callback_status;
	struct urb *purb;
	u8 *buffer;
};

struct event_callback_context {
	struct event_entry *event_entry;
};

static int configure_events(struct u3v_event *event, __u32 event_max_transfer,
		     __u32 event_queue_depth);
static void unconfigure_events(struct u3v_event *event);
static int queue_urbs(struct u3v_event *event);
static void abort_urbs(struct u3v_event *event);
static int queue_next_event(struct u3v_event *event);
static int submit_event_urb(struct u3v_event *event, struct event_entry *entry);
static void event_urb_completion(struct urb *purb);
static int transfer_event_data(struct u3v_event *event, void *user_buffer,
			  __u32 *buffer_size, void *event_complete_buffer);


/*
 * u3v_create_events - creates, initializes, and configures the event
 * interface.
 * @u3v: pointer to the u3v_device struct
 * @intf: pointer to the event interface
 * @event_max_transfer: the maximum expected event payload size in bytes
 * @event_queue_depth: the number of event entries to create and place in
 *		       the queue
 */
int u3v_create_events(struct u3v_device *u3v, struct usb_interface *intf,
	__u32 event_max_transfer, __u32 event_queue_depth)
{
	struct u3v_event *event = NULL;
	int ret = 0;

	if (u3v == NULL || intf == NULL)
		return -EINVAL;

	if (u3v->event_info.interface_ptr != NULL)
		return U3V_ERR_EVENTS_ALREADY_CONFIGURED;

	if (u3v->event_info.bulk_in == NULL) {
		dev_err(u3v->device,
			"%s: Did not detect bulk in endpoint in the event interface.\n",
			__func__);
		return U3V_ERR_NO_EVENT_INTERFACE;
	}

	event = kzalloc(sizeof(struct u3v_event), GFP_KERNEL);
	if (event == NULL)
		return -ENOMEM;

	event->u3v_dev = u3v;
	mutex_init(&event->event_lock);
	INIT_LIST_HEAD(&event->event_queue);
	event->queue_depth = 0;
	init_completion(&event->event_complete);
	complete_all(&event->event_complete);
	init_usb_anchor(&event->event_anchor);

	if (!u3v->stalling_disabled)
		reset_pipe(u3v, &u3v->event_info);

	ret = configure_events(event, event_max_transfer, event_queue_depth);

	if (ret != 0)
		kfree(event);
	else
		u3v->event_info.interface_ptr = event;

	return ret;
}


/*
 * configure_events - allocates memory for event entries and adds them to the
 *	       event queue.
 * precondition: either called from u3v_create_event prior to assigning the
 *	event pointer, or caller must hold the event_lock
 * @event: pointer to event interface struct
 * @event_max_transfer: the maximum expected event payload size in bytes
 * @event_queue_depth: the number of event entries to create and place in
 *		       the queue
 */
static int configure_events(struct u3v_event *event, __u32 event_max_transfer,
	      __u32 event_queue_depth)
{
	int i;
	struct event_entry *entry;

	/*
	 * This data is needed to get contiguous physical pages in
	 * memory for efficient DMA transfer
	 */
	event->order = get_order(PAGE_ALIGN(event_max_transfer));
	event->queue_depth = event_queue_depth;

	for (i = 0; i < event_queue_depth; i++) {

		entry = kmalloc(sizeof(struct event_entry), GFP_KERNEL);

		if (!entry)
			goto error;

		/* Get contiguous memory */
		entry->buffer =
			(void *)(__get_free_pages(GFP_KERNEL, event->order));
		if (entry->buffer == NULL)
			goto error;

		/*
		 * First parameter is number of iso_packets, which is
		 * irrelevant for a bulk transfer
		 */
		entry->purb = usb_alloc_urb(0, GFP_KERNEL);
		if (entry->purb == NULL)
			goto error;

		entry->event = event;
		entry->buffer_size = event_max_transfer;
		entry->buffer_received_size = 0;
		entry->state = event_idle;
		entry->callback_status = -ENODATA;
		INIT_LIST_HEAD(&entry->list_elem);
		list_add_tail(&entry->list_elem, &event->event_queue);
	}
	return 0;

error:
	if (entry) {
		usb_free_urb(entry->purb);
		free_pages((unsigned long)(entry->buffer), event->order);
		kfree(entry);
	}
	unconfigure_events(event);
	dev_err(event->u3v_dev->device,
		"%s: Error allocating memory for event queue\n", __func__);
	return -ENOMEM;
}


/*
 * unconfigure_events - iterates through the event queue and removes each
 *	entry, freeing each event_entry struct.
 * Precondition: either called from u3v_create_event prior to assigning the
 *	event pointer, or caller must hold the event_lock
 * @event - pointer to the u3v_event struct
 */
static void unconfigure_events(struct u3v_event *event)
{
	struct list_head *cur, *next;
	struct event_entry *tmp;

	list_for_each_safe(cur, next, &event->event_queue) {
		tmp = list_entry(cur, struct event_entry, list_elem);
		list_del(cur);
		free_pages((unsigned long)(tmp->buffer), event->order);
		usb_free_urb(tmp->purb);
		kfree(tmp);
	}
}


/*
 * u3v_destroy_events - aborts queued events, waits for event ioctls
 *	to complete, unconfigures events, and frees the event interface.
 * @u3v: pointer to u3v_device struct
 */
void u3v_destroy_events(struct u3v_device *u3v)
{
	struct u3v_event *event;

	if (u3v == NULL)
		return;

	event = (struct u3v_event *)(u3v->event_info.interface_ptr);
	if (event == NULL)
		return;

	mutex_lock(&event->event_lock);

	abort_urbs(event);

	mutex_unlock(&event->event_lock);

	wait_for_completion(&u3v->event_info.ioctl_complete);

	mutex_lock(&event->event_lock);

	unconfigure_events(event);

	if (!u3v->stalling_disabled)
		reset_pipe(u3v, &u3v->event_info);

	mutex_unlock(&event->event_lock);
	kfree(event);
	u3v->event_info.interface_ptr = NULL;
}


/*
 * u3v_start_events - creates and initializes all of the event entries
 *		      and attaches them to the hardware
 * @event: pointer to u3v event struct
 */
int u3v_start_events(struct u3v_event *event)
{
	int ret;

	if (event == NULL || event->u3v_dev == NULL)
		return -EINVAL;

	mutex_lock(&event->event_lock);

	ret = queue_urbs(event);
	if (ret != 0) {
		dev_err(event->u3v_dev->device,
			"%s: Error %d starting event queue\n", __func__, ret);
	}

	mutex_unlock(&event->event_lock);
	return ret;
}


/*
 * u3v_stop_events - cancels URB requests and waits for them to complete
 * @event: pointer to event interface struct
 */
void u3v_stop_events(struct u3v_event *event)
{
	if (event == NULL)
		return;

	mutex_lock(&event->event_lock);

	abort_urbs(event);

	mutex_unlock(&event->event_lock);
}


/*
 * queue_urbs - queues up all of the event URBs and sets the events
 *	started flag
 * precondition: caller holds event_lock
 * @event: pointer to event interface struct
 */
static int queue_urbs(struct u3v_event *event)
{
	int i;
	int ret = 0;


	if (event->started) {
		dev_err(event->u3v_dev->device,
			"%s: event already started\n", __func__);
		return U3V_ERR_EVENTS_ALREADY_CONFIGURED;
	}


	for (i = 0; i < event->queue_depth; i++) {
		ret = queue_next_event(event);
		if (ret != 0)
			goto error;
	}
	event->started = true;
	return 0;
error:
	abort_urbs(event);
	return ret;
}


/*
 * abort_urbs - Deactivates events and synchronously aborts open URB
 *	transactions, which waits for all of the callbacks to complete.
 *	On return, no urbs should be in flight.
 * precondition: caller holds event_lock
 * @event: pointer to event interface struct
 */
static void abort_urbs(struct u3v_event *event)
{
	event->started = false;

	usb_kill_anchored_urbs(&event->event_anchor);
}


/*
 * queue_next_event - submits URB for next event and updates it to the
 *		      queued state. Then advances the queue.
 * precondition: caller holds event_lock
 * @event: pointer to event interface struct
 */
static int queue_next_event(struct u3v_event *event)
{
	struct event_entry *entry;
	int ret;

	if (event == NULL) {
		pr_err("%s: event interface cannot be NULL\n", __func__);
		return U3V_ERR_NO_EVENT_INTERFACE;
	}

	entry = list_first_entry(&event->event_queue,
		struct event_entry, list_elem);

	if (entry->state == event_queued) {
		dev_err(event->u3v_dev->device,
			"%s: Cannot queue an event that is already queued\n",
			__func__);
		ret =  U3V_ERR_EVENT_ALREADY_QUEUED;
		goto exit;
	}
	entry->buffer_received_size = 0;

	/* Advance the queue */
	list_rotate_left(&event->event_queue);
	entry->state = event_queued;
	u3v_reinit_completion(&event->event_complete);

	ret = submit_event_urb(event, entry);
	if (ret != 0) {
		entry->state = event_idle;
		complete_all(&event->event_complete);
	}

exit:
	return ret;
}


/*
 * submit_event_urb - fills and submits a URB for a given event entry
 *
 * Precondition: caller holds event lock and entry lock
 * @event: pointer to event interface struct
 * @entry: pointer to event that's being queued
 */
static int submit_event_urb(struct u3v_event *event, struct event_entry *entry)
{
	struct device *dev;
	struct usb_device *udev;
	int ret;

	if (event == NULL) {
		pr_err("%s: event interface cannot be NULL\n", __func__);
		return U3V_ERR_NO_EVENT_INTERFACE;
	}

	dev = event->u3v_dev->device;
	udev = event->u3v_dev->udev;

	if (entry == NULL || entry->buffer == NULL || entry->purb == NULL) {
		dev_err(dev, "%s: cannot have NULL entry, buffer, or URB\n",
			__func__);
		return -EINVAL;
	}

	if (entry->buffer_size == 0)
		return 0;

	usb_fill_bulk_urb(entry->purb, udev,
		usb_rcvbulkpipe(udev,
		usb_endpoint_num(event->u3v_dev->event_info.bulk_in)),
		entry->buffer, entry->buffer_size, event_urb_completion,
		entry);

	usb_anchor_urb(entry->purb, &event->event_anchor);
	ret = usb_submit_urb(entry->purb, GFP_KERNEL);

	if (ret != 0) {
		usb_unanchor_urb(entry->purb);
		dev_err(dev, "%s: Error %d submitting urb\n", __func__, ret);
	}

	return ret;
}


/*
 * event_urb_completion - callback method for URB requests for event data
 * @purb - pointer to completed URB
 */
static void event_urb_completion(struct urb *purb)
{
	struct event_entry *entry;
	struct device *dev;

	/* check pointers */
	if (purb == NULL) {
		pr_err("%s: Received a NULL urb pointer\n", __func__);
		return;
	}
	dev = &purb->dev->dev;

	/* check status */
	if (purb->status &&
		!(purb->status == -ENOENT ||
		  purb->status == -ECONNRESET ||
		  purb->status == -ESHUTDOWN ||
		  purb->status == -EPROTO)) {
		dev_err(dev, "%s: Received nonzero urb completion status: %d",
			__func__, purb->status);
	}

	entry = (struct event_entry *)(purb->context);
	if (entry == NULL) {
		dev_err(dev, "%s: Received a NULL context pointer\n",
			__func__);
		return;
	}
	/* Ensure buffer was queued */
	WARN_ON(entry->state != event_queued);
	entry->buffer_received_size = purb->actual_length;
	entry->callback_status = purb->status;
	entry->state = event_complete;
	complete_all(&entry->event->event_complete);
}


/*
 * u3v_wait_for_event - waits for an event to be sent from the device and
 * then copies the payload into the provided buffer
 *
 * @event: pointer to event interface struct
 * @user_buffer: the event payload will be copied into this buffer
 * @buffer_size: the size of the user_buffer on input, actual size
 *		 of the data copied on return
 * @event_complete_buffer: additional data describing the completed event,
 *		          can be NULL
 */
int u3v_wait_for_event(struct u3v_event *event, void *user_buffer,
		   __u32 *buffer_size, void *event_complete_buffer)
{
	struct device *dev;
	struct event_entry *entry;
	bool event_ready = false;
	int ret = 0;

	if (event == NULL || event->u3v_dev == NULL)
		return -EINVAL;

	dev = event->u3v_dev->device;

	if (user_buffer == NULL) {
		dev_err(dev, "%s: user_buffer cannot be NULL\n", __func__);
		return -EINVAL;
	}

	if (buffer_size == NULL) {
		dev_err(dev, "%s: buffer_size cannot be NULL\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&event->event_lock);

	/* Ensure that the event interface has been started */
	if (!event->started) {
		dev_err(dev,
			"%s: Cannot call wait_for_event before starting event interface\n",
			__func__);
		ret = U3V_ERR_EVENTS_NOT_STARTED;
		goto exit;
	}
	entry = list_first_entry(&event->event_queue,
				 struct event_entry, list_elem);

	/* Ensure that we are waiting on a queued event */
	if (entry->state == event_idle) {
		dev_err(dev, "%s: Requested to wait on an unqueued entry\n",
			__func__);
		ret = U3V_ERR_EVENTS_NOT_STARTED;
		goto exit;
	}

	/* Only one thread at a time can wait for an event */
	if (event->wait_in_progress) {
		dev_err(dev, "%s: Wait is already in progress\n", __func__);
		ret = U3V_ERR_ALREADY_WAITING;
		goto exit;
	}

	if (entry->state == event_complete)
		event_ready = true;

	event->wait_in_progress = true;

	while (!event_ready) {

		mutex_unlock(&event->event_lock);

		/* wait for the URB callback to complete */
		ret = wait_for_completion_interruptible(&event->event_complete);

		mutex_lock(&event->event_lock);

		/* check if we were interrupted by a signal */
		if (ret != 0) {
			dev_dbg(dev, "%s: wait interrupted by signal\n",
				__func__);
			ret = U3V_ERR_EVENT_WAIT_CANCELED;
			goto exit;
		}

		/* check if device was removed or wait cancelled somehow */
		if (!event->u3v_dev->device_connected ||
			!event->started ||
			entry->callback_status == -ENOENT ||
			entry->callback_status == -ECONNRESET ||
			entry->callback_status == -ESHUTDOWN ||
			entry->callback_status == -EPROTO) {

			dev_dbg(dev, "%s: Event interface was stopped\n",
				__func__);
			ret = U3V_ERR_EVENT_WAIT_CANCELED;
			goto exit;
		}

		/* Don't keep waiting if next event isn't queued */
		if (entry->state == event_idle) {
			dev_err(dev, "%s: Cannot wait on an unqueued entry\n",
			__func__);
			ret = U3V_ERR_EVENTS_NOT_STARTED;
			goto exit;
		}
		event_ready = (entry->state == event_complete);
	}
	ret = transfer_event_data(event, user_buffer, buffer_size,
			     event_complete_buffer);

exit:
	event->wait_in_progress = false;
	mutex_unlock(&event->event_lock);
	return ret;
}


/*
 * transfer_event_data - copies the payload into the provided buffer
 * precondition: caller holds event_lock, and user_buffer and buffer_size are
 *	not NULL
 *
 * @event: pointer to event interface struct
 * @user_buffer: the event payload will be copied into this buffer
 * @buffer_size: the size of the user_buffer on input, actual size
 *		 of the data copied on return
 * @event_complete_buffer: additional data describing the completed event,
 *		          can be NULL
 */
static int transfer_event_data(struct u3v_event *event, void *user_buffer,
			  __u32 *buffer_size, void *event_complete_buffer)
{
	struct command *cmd;
	struct event_entry *entry;
	struct device *dev;
	struct event_complete_data ecd;
	u32 k_buffer_size = 0;
	int ret;

	dev = event->u3v_dev->device;
	entry = list_first_entry(&event->event_queue,
			   struct event_entry, list_elem);

	/* Ensure we received a valid event command */
	cmd = (struct command *)(entry->buffer);

	if ((entry->buffer_received_size > entry->buffer_size) ||
	    (cmd->header.prefix != U3V_EVENT_PREFIX) ||
	    (cmd->header.cmd != EVENT_CMD) ||
	    (cmd->header.length > (entry->buffer_size - sizeof(*cmd)))) {

		dev_err(dev, "%s: Received an invalid event_cmd buffer\n",
			__func__);
		dev_err(dev, "\tReceived Bytes = 0x%X (Expected <= 0x%X)\n",
			entry->buffer_received_size, entry->buffer_size);
		dev_err(dev, "\tPrefix = 0x%X (Expected 0x%X)\n",
			cmd->header.prefix, U3V_EVENT_PREFIX);
		dev_err(dev, "\tCommand = 0x%X (Expected 0x%X)\n",
			cmd->header.cmd, EVENT_CMD);
		dev_err(dev, "\tPayload length = 0x%X (Expected 0x%X)\n",
			cmd->header.length,
			(u32)(entry->buffer_size - sizeof(*cmd)));
		ret = U3V_ERR_INVALID_DEVICE_RESPONSE;
		goto exit;
	}

	/* Fill in the event buffer with the payload data */
	k_buffer_size = min(entry->buffer_received_size, entry->buffer_size);
	ret = copy_to_user(user_buffer, cmd, k_buffer_size);

	/*
	 * If unsuccessful, ret will contain the number of bytes
	 * that failed to be copied
	 */
	if (ret != 0) {
		dev_err(dev, "%s: Error copying to user buffer\n", __func__);
		k_buffer_size = k_buffer_size - ret;
		ret = U3V_ERR_INTERNAL;
		goto exit;
	}

	/* Fill in event complete data buffer if necessary */
	if (event_complete_buffer != NULL) {
		ecd.structure_size = sizeof(ecd);
		ecd.event_callback_status = entry->callback_status;
		ecd.overwritten_event_count = event->overwritten_event_count;
		ret = copy_to_user(event_complete_buffer, &ecd,
			ecd.structure_size);
		if (ret != 0) {
			dev_err(dev, "%s: Error copying event data\n",
				__func__);
			ret = U3V_ERR_INTERNAL;
			goto exit;
		}
	}

	if (event->started)
		ret = queue_next_event(event);
exit:
	put_user(k_buffer_size, buffer_size);
	return ret;
}
