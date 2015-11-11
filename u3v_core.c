/*
 * u3v_core.c: Driver for USB3 Vision(TM) class devices
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
#include "u3v_control.h"
#include "u3v_stream.h"
#include "u3v_event.h"
#include "u3v_shared.h"
#include <linux/atomic.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/lcm.h>
#include <linux/module.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/version.h>
#include <asm/unaligned.h>

enum direction_t {
	dir_in,
	dir_out
};

/* probe/disconnect and file operations */
static int u3v_probe(struct usb_interface *interface,
		     const struct usb_device_id *id);
static void u3v_disconnect(struct usb_interface *interface);
static int u3v_open(struct inode *inode, struct file *filp);
static int u3v_release(struct inode *inode, struct file *filp);
static long u3v_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
static void u3v_delete(struct kref *kref);

/* helper functions for probe */
static int enumerate_u3v_interfaces(struct u3v_device *u3v);
static int populate_u3v_properties(struct u3v_device *u3v);
static struct usb_endpoint_descriptor *find_bulk_endpoint(
	struct usb_host_interface *iface_desc, enum direction_t dir);

/* device initialization */
static int initialize(struct u3v_device *u3v);
static int get_stream_capabilities(struct u3v_device *u3v);

/* device level ioctls and helper functions*/
static int u3v_get_os_max_transfer_size(struct u3v_device *u3v, __u32 *os_max);
static int u3v_get_stream_alignment(struct u3v_device *u3v, __u32 *alignment);
static int u3v_configure_stream(struct u3v_device *u3v,
	__u64 image_buffer_size, __u64 chunk_data_buffer_size,
	__u32 max_urb_size, __u32 *max_leader_size, __u32 *max_trailer_size);
static int read_stream_registers(struct u3v_device *u3v,
	u32 *req_leader_size, u32 *req_trailer_size);
static int write_stream_registers(struct u3v_device *u3v,
	u32 max_leader_size, u32 max_trailer_size, u32 payload_size,
	u32 payload_count, u32 transfer1_size, u32 transfer2_size);
static u64 compatibility_div(u64 dividend, u32 divisor);
static u32 compatibility_mod(u64 dividend, u32 divisor);

/* helper function for reset_pipe */
static int reset_endpoint(struct u3v_device *u3v,
	struct usb_endpoint_descriptor *ep);

#define GET_INTERFACE(ptr_type, ptr, interface_info)		\
do {								\
	mutex_lock(&interface_info.interface_lock);		\
	mutex_lock(&interface_info.ioctl_count_lock);		\
	ptr = (ptr_type)(interface_info.interface_ptr);		\
	if (interface_info.ioctl_count++ == 0)			\
		u3v_reinit_completion(&interface_info.ioctl_complete);	\
	mutex_unlock(&interface_info.ioctl_count_lock);		\
	mutex_unlock(&interface_info.interface_lock);		\
} while (0)

#define PUT_INTERFACE(interface_info)				\
do {								\
	mutex_lock(&interface_info.ioctl_count_lock);		\
	if (--interface_info.ioctl_count == 0)			\
		complete_all(&interface_info.ioctl_complete);	\
	mutex_unlock(&interface_info.ioctl_count_lock);		\
} while (0)

/* sysfs attribute configuration */

#define u3v_attribute_num(name)						\
static ssize_t name##_show(struct device *dev,				\
	struct device_attribute *attr, char *buf)			\
{									\
	struct u3v_device *u3v = u3v_get_driver_data(dev);		\
									\
	if (u3v == NULL || u3v->u3v_info == NULL)			\
		return -ENODEV;						\
									\
	return scnprintf(buf, PAGE_SIZE, "%u\n", u3v->u3v_info->name);	\
}									\
static DEVICE_ATTR(name, S_IRUGO, name##_show, NULL);

#define u3v_attribute_str(name)						\
static ssize_t name##_show(struct device *dev,				\
	struct device_attribute *attr, char *buf)			\
{									\
	struct u3v_device *u3v = u3v_get_driver_data(dev);		\
									\
	if (u3v == NULL || u3v->u3v_info == NULL)			\
		return -ENODEV;						\
									\
	return scnprintf(buf, PAGE_SIZE, "%s\n", u3v->u3v_info->name);	\
}									\
static DEVICE_ATTR(name, S_IRUGO, name##_show, NULL);

#define udev_attribute_str(name)					\
static ssize_t name##_show(struct device *dev,				\
	struct device_attribute *attr, char *buf)			\
{									\
	struct u3v_device *u3v = u3v_get_driver_data(dev);		\
									\
	if (u3v == NULL || u3v->udev == NULL)				\
		return -ENODEV;						\
									\
	return scnprintf(buf, PAGE_SIZE, "%s\n", u3v->udev->name);	\
}									\
static DEVICE_ATTR(name, S_IRUGO, name##_show, NULL);

#define udev_desc_attribute(name)					\
static ssize_t name##_show(struct device *dev,				\
	struct device_attribute *attr, char *buf)			\
{									\
	struct u3v_device *u3v = u3v_get_driver_data(dev);		\
									\
	if (u3v == NULL || u3v->udev == NULL)				\
		return -ENODEV;						\
									\
	return scnprintf(buf, PAGE_SIZE, "%u\n",			\
		le16_to_cpup(&u3v->udev->descriptor.name));		\
}									\
static DEVICE_ATTR(name, S_IRUGO, name##_show, NULL);

udev_desc_attribute(idVendor);
udev_desc_attribute(idProduct);
udev_attribute_str(manufacturer);
udev_attribute_str(product);
udev_attribute_str(serial);
u3v_attribute_num(gen_cp_version);
u3v_attribute_num(u3v_version);
u3v_attribute_str(device_guid);
u3v_attribute_str(vendor_name);
u3v_attribute_str(model_name);
u3v_attribute_str(family_name);
u3v_attribute_str(device_version);
u3v_attribute_str(manufacturer_info);
u3v_attribute_str(serial_number_u3v);
u3v_attribute_str(user_defined_name);
u3v_attribute_num(speed_support);
u3v_attribute_num(previously_initialized);
u3v_attribute_num(host_byte_alignment);
u3v_attribute_num(os_max_transfer_size);

static struct attribute *u3v_attrs[] = {
	&dev_attr_idVendor.attr,
	&dev_attr_idProduct.attr,
	&dev_attr_manufacturer.attr,
	&dev_attr_product.attr,
	&dev_attr_serial.attr,
	&dev_attr_gen_cp_version.attr,
	&dev_attr_u3v_version.attr,
	&dev_attr_device_guid.attr,
	&dev_attr_vendor_name.attr,
	&dev_attr_model_name.attr,
	&dev_attr_family_name.attr,
	&dev_attr_device_version.attr,
	&dev_attr_manufacturer_info.attr,
	&dev_attr_serial_number_u3v.attr,
	&dev_attr_user_defined_name.attr,
	&dev_attr_speed_support.attr,
	&dev_attr_previously_initialized.attr,
	&dev_attr_host_byte_alignment.attr,
	&dev_attr_os_max_transfer_size.attr,
	NULL,
};

static struct attribute_group u3v_attr_grp = {
	.attrs = u3v_attrs,
};

/* end of sysfs attributes */


/* Static structs */

static const struct file_operations fops = {
	.owner = THIS_MODULE,
/*	.read not implemented  */
/*	.write not implemented */
	.open = u3v_open,
	.release = u3v_release,
	.unlocked_ioctl = u3v_ioctl,
	.llseek = default_llseek,
};

static struct usb_class_driver u3v_class = {
	.name =		"u3v%d",
	.fops =		&fops,
	/*
	* This base is used when usb_register_dev is called in order to
	* determine which minor number should be given out by the function.
	* If CONFIG_USB_DYNAMIC_MINORS is set by the kernel (you can check
	* in /proc/config.gz), then you can dynamically allocate the minor
	* number out of the list of available ones. Ideally this would be
	* set so that we could have more than 16 of a single type of
	* device. However, if it is not set, we use this minor
	* base to provide this minor number and the 15 subsequent numbers
	* to be allocated.
	*/
	.minor_base =	U3V_MINOR_BASE,
};

static struct usb_device_id u3v_table[] = {
	{ USB_INTERFACE_INFO(U3V_INTERFACE_CLASS, U3V_INTERFACE_SUBCLASS,
	U3V_INTERFACE_PROTOCOL_CONTROL) },
	{} /* Terminating entry */
};

MODULE_DEVICE_TABLE(usb, u3v_table);

static struct usb_driver u3v_driver = {
	.name = "u3v",
	.id_table = u3v_table,
	.probe = u3v_probe,
	.disconnect = u3v_disconnect,
};

/* Functions */

/*
 * u3v_ioctl - ioctl commands for the u3v driver
 * @filp: pointer to the open device file
 * @cmd: value indicating which ioctl command to execute
 * @arg: pointer to a struct with additional data for ioctl, can be NULL
 */
static long u3v_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct u3v_read_memory *read_req;
	struct u3v_write_memory *write_req;
	struct u3v_get_stream_alignment *align;
	struct u3v_get_os_max_transfer *os_max;
	struct u3v_configure_stream *config_stream;
	struct u3v_configure_buffer *config_req;
	struct u3v_unconfigure_buffer *unconfig_req;
	struct u3v_queue_buffer *queue_req;
	struct u3v_wait_for_buffer *wait_req;
	struct u3v_control_msg *ctrl_msg;
	struct u3v_start_events *start_events;
	struct u3v_wait_for_event *wait_event;
	struct u3v_device *u3v;
	struct u3v_control *control;
	struct u3v_event *event;
	struct u3v_stream *stream;
	struct device *dev;
	int ret = 0;
	int err = 0;

	if (!filp || !filp->private_data)
		return -EINVAL;

	u3v = filp->private_data;
	dev = u3v->device;

	if (!u3v->device_connected)
		return -ENODEV;

	dev_vdbg(dev, "%s: cmd = %X, filp = %p, arg = %p\n",
		__func__, cmd, filp, (void *)(arg));

	if (cmd != U3V_IOCTL_CTRL_MSG) {
		ret = initialize(u3v);
		if (ret != 0)
			return ret;
	}

	if ((_IOC_TYPE(cmd) != U3V_MAGIC) || (_IOC_NR(cmd) > U3V_MAX_NR)) {
		dev_err(dev, "%s: Received invalid command %X\n",
			 __func__, cmd);
		return U3V_ERR_NOT_SUPPORTED;
	}

	/* Check safety of transfer between user and kernel space */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg,
			_IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user *)arg,
			_IOC_SIZE(cmd));
	if (err) {
		dev_err(dev, "%s: Failure on access check for pointer %p",
			__func__, (void *)(arg));
		return -EFAULT;
	}

	switch (cmd) {
	case U3V_IOCTL_READ:
		read_req = (struct u3v_read_memory *)(arg);
		if (!read_req)
			goto error;

		GET_INTERFACE(struct u3v_control *, control,
			u3v->control_info);
		ret = u3v_read_memory(control,
			read_req->transfer_size,
			read_req->bytes_read,
			read_req->address, NULL,
			read_req->user_buffer);
		PUT_INTERFACE(u3v->control_info);
		break;
	case U3V_IOCTL_WRITE:
		write_req = (struct u3v_write_memory *)(arg);
		if (!write_req)
			goto error;

		GET_INTERFACE(struct u3v_control *, control,
			u3v->control_info);
		ret = u3v_write_memory(control,
			write_req->transfer_size,
			write_req->bytes_written,
			write_req->address, NULL,
			write_req->user_buffer);
		PUT_INTERFACE(u3v->control_info);
		break;
	case U3V_IOCTL_GET_STREAM_ALIGNMENT:
		align = (struct u3v_get_stream_alignment *)(arg);
		if (!align)
			goto error;

		ret = u3v_get_stream_alignment(u3v, align->stream_alignment);
		break;
	case U3V_IOCTL_GET_OS_MAX_TRANSFER:
		os_max = (struct u3v_get_os_max_transfer *)(arg);
		if (!os_max)
			goto error;

		ret = u3v_get_os_max_transfer_size(u3v,
			os_max->os_max_transfer_size);
		break;
	case U3V_IOCTL_CONFIGURE_STREAM:
		config_stream = (struct u3v_configure_stream *)(arg);
		if (!config_stream)
			goto error;
		mutex_lock(&u3v->stream_info.interface_lock);

		ret = u3v_configure_stream(u3v,
			config_stream->image_buffer_size,
			config_stream->chunk_data_buffer_size,
			config_stream->max_urb_size,
			config_stream->max_leader_size,
			config_stream->max_trailer_size);

		mutex_unlock(&u3v->stream_info.interface_lock);
		break;
	case U3V_IOCTL_UNCONFIGURE_STREAM:
		mutex_lock(&u3v->stream_info.interface_lock);
		ret = u3v_destroy_stream(u3v);
		mutex_unlock(&u3v->stream_info.interface_lock);
		break;
	case U3V_IOCTL_CONFIGURE_BUFFER:
		config_req = (struct u3v_configure_buffer *)(arg);
		if (!config_req)
			goto error;

		GET_INTERFACE(struct u3v_stream *, stream, u3v->stream_info);
		ret = u3v_configure_buffer(stream,
			config_req->user_image_buffer,
			config_req->user_chunk_data_buffer,
			config_req->buffer_handle);
		PUT_INTERFACE(u3v->stream_info);
		break;
	case U3V_IOCTL_UNCONFIGURE_BUFFER:
		unconfig_req = (struct u3v_unconfigure_buffer *)(arg);
		if (!unconfig_req)
			goto error;

		GET_INTERFACE(struct u3v_stream *, stream, u3v->stream_info);
		ret = u3v_unconfigure_buffer(stream,
			unconfig_req->buffer_handle);
		PUT_INTERFACE(u3v->stream_info);
		break;
	case U3V_IOCTL_QUEUE_BUFFER:
		queue_req = (struct u3v_queue_buffer *)(arg);
		if (!queue_req)
			goto error;

		GET_INTERFACE(struct u3v_stream *, stream, u3v->stream_info);
		ret = u3v_queue_buffer(stream, queue_req->buffer_handle);
		PUT_INTERFACE(u3v->stream_info);
		break;
	case U3V_IOCTL_WAIT_FOR_BUFFER:
		wait_req = (struct u3v_wait_for_buffer *)(arg);
		if (!wait_req)
			goto error;

		GET_INTERFACE(struct u3v_stream *, stream, u3v->stream_info);
		ret = u3v_wait_for_buffer(stream,
			wait_req->buffer_handle,
			wait_req->user_leader_buffer,
			wait_req->leader_size,
			wait_req->user_trailer_buffer,
			wait_req->trailer_size,
			wait_req->buffer_complete_data);
		PUT_INTERFACE(u3v->stream_info);
		break;
	case U3V_IOCTL_CANCEL_ALL_BUFFERS:
		GET_INTERFACE(struct u3v_stream *, stream, u3v->stream_info);
		ret = u3v_cancel_all_buffers(stream);
		PUT_INTERFACE(u3v->stream_info);
		break;
	case U3V_IOCTL_CTRL_MSG:
		ctrl_msg = (struct u3v_control_msg *)(arg);
		if (!ctrl_msg)
			goto error;
		ret = u3v_control_msg(u3v->udev,
			ctrl_msg->request, ctrl_msg->request_type,
			ctrl_msg->value, ctrl_msg->index,
			ctrl_msg->data, ctrl_msg->size);
		break;
	case U3V_IOCTL_START_EVENTS:
		start_events = (struct u3v_start_events *)(arg);
		if (!start_events)
			goto error;

		mutex_lock(&u3v->event_info.interface_lock);
		ret = u3v_create_events(u3v, usb_ifnum_to_if(u3v->udev,
			u3v->event_info.idx),
			start_events->event_transfer_max_size,
			start_events->event_queue_depth);
		if (ret == 0) {
			event = (struct u3v_event *)
				(u3v->event_info.interface_ptr);
			ret = u3v_start_events(event);
			if (ret != 0)
				u3v_destroy_events(u3v);
		}
		mutex_unlock(&u3v->event_info.interface_lock);
		break;
	case U3V_IOCTL_WAIT_FOR_EVENT:
		wait_event = (struct u3v_wait_for_event *)(arg);
		if (!wait_event)
			goto error;

		GET_INTERFACE(struct u3v_event *, event, u3v->event_info);
		ret = u3v_wait_for_event(event,
			wait_event->user_buffer,
			wait_event->buffer_size,
			wait_event->event_complete_buffer);
		PUT_INTERFACE(u3v->event_info);
		break;
	case U3V_IOCTL_STOP_EVENTS:
		mutex_lock(&u3v->event_info.interface_lock);
		event = (struct u3v_event *)(u3v->event_info.interface_ptr);
		u3v_stop_events(event);
		u3v_destroy_events(u3v);
		mutex_unlock(&u3v->event_info.interface_lock);
		break;
	}
	return ret;
error:
	dev_err(dev, "%s: Received a NULL pointer as an argument\n", __func__);
	return -EINVAL;
}


/*
 * u3v_open - called when a process tries to open the device file
 */
static int u3v_open(struct inode *inode, struct file *filp)
{
	struct usb_interface *intf;
	struct u3v_device *u3v;

	intf = usb_find_interface(&u3v_driver, iminor(inode));
	if (!intf) {
		pr_err("%s: cannot find device for minor %d\n",
			__func__, iminor(inode));
		return -ENODEV;
	}
	u3v = usb_get_intfdata(intf);

	/*
	 * Ensuring that device is being used by only one process
	 * at a time.
	 */
	if (!atomic_dec_and_test(&u3v->device_available)) {
		atomic_inc(&u3v->device_available);
		dev_err(u3v->device, "%s: device is not available\n",
			__func__);
		return -EBUSY;
	}
	kref_get(&u3v->kref);
	filp->private_data = u3v;

	return 0;
}


/*
 * initialize - enumerates and initializes interfaces for this device
 */
static int initialize(struct u3v_device *u3v)
{
	int ret = 0;

	mutex_lock(&u3v->control_info.interface_lock);

	if (u3v->control_info.interface_ptr != NULL) {
		mutex_unlock(&u3v->control_info.interface_lock);
		return 0;
	}

	ret = u3v_create_control(u3v);

	mutex_unlock(&u3v->control_info.interface_lock);
	if (ret != 0)
		return ret;

	ret = get_stream_capabilities(u3v);
	/*
	 * get_stream_capabilities only fails if the reads fail.
	 * If streaming is actually not supported, it will still
	 * return 0 and we will not destroy the control interface.
	 * Initialization succeeds, but any calls to create or use
	 * the stream interface will fail.
	 */
	if (ret != 0) {
		mutex_lock(&u3v->control_info.interface_lock);
		u3v_destroy_control(u3v);
		mutex_unlock(&u3v->control_info.interface_lock);
	} else {
		u3v->u3v_info->previously_initialized = 1;
	}
	return ret;
}


/*
 * get_stream_capabilities - helper function for initialize that reads
 *	relevant stream registers to store configuration information
 * @u3v: pointer to the u3v_device struct. If stream is supported,
 *	on return sirm_addr and transfer_alignment are set in
 *	u3v_device_info
 */
static int get_stream_capabilities(struct u3v_device *u3v)
{
	struct u3v_control *control;
	struct device *dev;
	u64 sbrm_address;
	u64 u3v_capability;
	u32 si_info;
	u32 device_byte_alignment;
	int bytes_read;
	int ret;

	/*
	 * Control interface must already be set up so we can read
	 * the stream interface registers
	 */
	if (u3v == NULL || u3v->u3v_info == NULL)
		return -EINVAL;

	GET_INTERFACE(struct u3v_control *, control, u3v->control_info);
	dev = u3v->device;

	/* First get stream interface information */
	ret = u3v_read_memory(control, sizeof(sbrm_address), &bytes_read,
		ABRM_SBRM_ADDRESS, &sbrm_address, NULL);

	if (ret != 0) {
		dev_err(dev, "%s: Error reading SBRM address\n", __func__);
		goto exit;
	}
	/* Check capabilities to see if SIRM is available */
	ret = u3v_read_memory(control, sizeof(u3v_capability),
		&bytes_read, sbrm_address + SBRM_U3VCP_CAPABILITY,
		&u3v_capability, NULL);

	if (ret != 0) {
		dev_err(dev, "%s: Error reading U3VCP capability\n", __func__);
		goto exit;
	}

	if (u3v_capability & SIRM_AVAILABLE_MASK) {
		ret = u3v_read_memory(control,
			sizeof(u3v->u3v_info->sirm_addr),
			&bytes_read, sbrm_address + SBRM_SIRM_ADDRESS,
			&u3v->u3v_info->sirm_addr, NULL);

		if (ret != 0) {
			dev_err(dev, "%s: Error reading SIRM address\n",
				__func__);
			goto exit;
		}

		ret = u3v_read_memory(control, sizeof(si_info),
			&bytes_read, u3v->u3v_info->sirm_addr + SI_INFO,
			&si_info, NULL);

		if (ret != 0) {
			dev_err(dev, "%s: Error reading SI info\n", __func__);
			goto exit;
		}

		device_byte_alignment = 1 <<
			((si_info & SI_INFO_ALIGNMENT_MASK) >>
			SI_INFO_ALIGNMENT_SHIFT);

		u3v->u3v_info->transfer_alignment = lcm(device_byte_alignment,
			u3v->u3v_info->host_byte_alignment);

		dev_vdbg(dev, "%s: transfer alignment is %u\n", __func__,
			u3v->u3v_info->transfer_alignment);
	}
exit:
	PUT_INTERFACE(u3v->control_info);
	return ret;
}


/*
 * u3v_configure_stream - sets up the stream interface for the device
 * @u3v: pointer to the u3v_device struct
 * @image_buffer_size: the size of the buffers that will hold the image data.
 *	Could include size of both the image data and chunk data if they
 *	are to be acquired into the same buffer. Otherwise, it is just
 *	the size of the image data.
 * @chunk_data_buffer_size: the size of the buffers that will hold only
 *	chunk data. If the chunk data is acquired into the same buffer
 *	as the image data, this size should be 0.
 * @max_urb_size: this parameter can be used to limit the size of the
 *	URBs if necessary. If you want to use the max size supported
 *	by the OS, set this value to uint max
 * @max_leader_size: on return this will contain the size of the
 *	buffer that the caller must allocate to guarantee enough
 *	space to hold all of the leader data. This size will account
 *	for any alignment restrictions
 * @max_trailer_size: on return this will contain the size of the
 *	buffer that the caller must allocate to guarantee enough space
 *	to hold all of the trailer data. This size will account for
 *	any alignment restrictions.
 */
static int u3v_configure_stream(struct u3v_device *u3v,
	__u64 image_buffer_size, __u64 chunk_data_buffer_size,
	__u32 max_urb_size, __u32 *max_leader_size, __u32 *max_trailer_size)
{
	struct device *dev;
	u32 req_leader_size = 0;
	u32 req_trailer_size = 0;
	u32 byte_alignment = 0;
	u32 payload_size = 0;
	u32 payload_count = 0;
	u32 transfer1_size = 0;
	u32 transfer2_size = 0;
	u32 aligned_max_transfer_size = 0;
	u32 aligned_cd_buffer_size = 0;
	u32 k_max_leader_size = 0;
	u32 k_max_trailer_size = 0;
	int ret = 0;

	if (u3v == NULL)
		return -EINVAL;

	dev = u3v->device;

	ret = read_stream_registers(u3v, &req_leader_size, &req_trailer_size);
	if (ret != 0)
		return ret;

	byte_alignment = u3v->u3v_info->transfer_alignment;
	aligned_max_transfer_size = (max_urb_size / byte_alignment) *
		byte_alignment;
	k_max_leader_size = ((req_leader_size + byte_alignment - 1) /
		byte_alignment) * byte_alignment;
	k_max_leader_size = min(k_max_leader_size,
		aligned_max_transfer_size);

	if (max_leader_size != NULL) {
		ret = put_user(k_max_leader_size, max_leader_size);
		if (ret != 0) {
			dev_err(dev,
				"%s: Error copying max leader size to user\n",
				__func__);
			return ret;
		}
	}
	payload_size = aligned_max_transfer_size;
	payload_count = compatibility_div(image_buffer_size, payload_size);
	transfer1_size = (compatibility_mod(image_buffer_size, payload_size) /
		byte_alignment) * byte_alignment;
	transfer2_size =
		compatibility_mod(image_buffer_size, byte_alignment) == 0 ?
		0 : byte_alignment;
	k_max_trailer_size = ((req_trailer_size + byte_alignment - 1) /
		byte_alignment) * byte_alignment;
	k_max_trailer_size = min(k_max_trailer_size,
		aligned_max_transfer_size);

	if (max_trailer_size != NULL) {
		ret = put_user(k_max_trailer_size, max_trailer_size);
		if (ret != 0) {
			dev_err(dev,
				"%s: Error copying max trailer size to user\n",
				__func__);
			return ret;
		}
	}

	/*
	 * If there is a separate chunk data buffer, then chunk data size
	 * will be > 0. In this case image data must fit into the
	 * transfer buffers or the final transfer 1 payload. If final
	 * transfer 1 is being used for image data, then the chunk data
	 * must fit into a single URB. Chunk data does not have to be
	 * alignmed because it's a small amount of data compared to the
	 * image. In that case we are okay using the final transfer 2
	 * buffer for the transfer and then copying the data into the
	 * chunk data buffer.
	 */
	if (chunk_data_buffer_size != 0) {
		if (transfer2_size != 0) {
			dev_err(dev,
				"%s: Cannot split the image and chunk data into separate buffers because the image size is not a multiple of the required transfer alignment size\n",
				__func__);
			return U3V_ERR_IMAGE_SIZE_NOT_ALIGNED;
		}
		aligned_cd_buffer_size =
			compatibility_div((chunk_data_buffer_size +
			byte_alignment - 1), byte_alignment) * byte_alignment;
		dev_dbg(dev, "%s: aligned chunk data buffer size = %u\n",
			__func__, aligned_cd_buffer_size);
		if (transfer1_size != 0 && aligned_cd_buffer_size >
			aligned_max_transfer_size) {
			dev_err(dev, "%s: Chunk data size too big\n",
				__func__);
			return U3V_ERR_CHUNK_DATA_SIZE_TOO_BIG;
		}

		if (transfer1_size == 0) {
			payload_count +=
				compatibility_div(chunk_data_buffer_size,
				payload_size);
			transfer1_size =
				(compatibility_mod(chunk_data_buffer_size,
				payload_size) / byte_alignment) *
				byte_alignment;
			transfer2_size =
				compatibility_mod(chunk_data_buffer_size,
				byte_alignment) == 0 ? 0 : byte_alignment;
		} else {
			transfer2_size = aligned_cd_buffer_size;
		}
	}

	/* now that the buffer sizes are configured, create the stream */
	ret = u3v_create_stream(u3v, usb_ifnum_to_if(u3v->udev,
		u3v->stream_info.idx), image_buffer_size,
		chunk_data_buffer_size, k_max_leader_size,
		k_max_trailer_size, payload_size, payload_count,
		transfer1_size, transfer2_size);

	if (ret != 0)
		return ret;

	ret = write_stream_registers(u3v, k_max_leader_size,
		k_max_trailer_size, payload_size, payload_count,
		transfer1_size, transfer2_size);

	if (ret != 0) {
		/* caller already holds stream_info->interface_lock */
		u3v_destroy_stream(u3v);
	}
	return ret;
}


/*
 * read_stream_registers - helper function for configure_stream that
 *	reads the required leader and trailer sizes from the device
 * @u3v: pointer to the struct u3v_device
 * @req_leader_size: on successful return, this will point to the
 *	value for the device's required leader size
 * @req_trailer_size: on successful return, this will point to the
 *	value for the device's required trailer size
 */
static int read_stream_registers(struct u3v_device *u3v,
	u32 *req_leader_size, u32 *req_trailer_size)
{
	struct u3v_control *control;
	struct device *dev;
	u32 leader_size = 0;
	u32 trailer_size = 0;
	int bytes_read;
	int ret;

	if (u3v == NULL)
		return -EINVAL;

	dev = u3v->device;
	GET_INTERFACE(struct u3v_control *, control, u3v->control_info);

	/*
	 * Read the required size information. If device reports 0
	 * for the required leader and trailer size, we set it to
	 * at least 1024 bytes
	 */
	ret = u3v_read_memory(control, sizeof(leader_size),
		&bytes_read, u3v->u3v_info->sirm_addr + SI_REQ_LEADER_SIZE,
		req_leader_size, NULL);

	if (ret != 0) {
		dev_err(dev, "%s: Error reading required leader size\n",
			__func__);
		goto exit;
	}
	leader_size = max(leader_size, (u32)(1024));

	ret = u3v_read_memory(control, sizeof(trailer_size),
		&bytes_read, u3v->u3v_info->sirm_addr + SI_REQ_TRAILER_SIZE,
		&trailer_size, NULL);

	if (ret != 0) {
		dev_err(dev, "%s: Error reading required trailer size\n",
			__func__);
	}
	trailer_size = max(trailer_size, (u32)(1024));

	*req_leader_size = leader_size;
	*req_trailer_size = trailer_size;

exit:
	PUT_INTERFACE(u3v->control_info);
	return ret;
}


/*
 * write_stream_registers - helper function for configure_stream that
 *	writes the buffer configuration information out to the device's
 *	stream registers. These sizes were validated previously when
 *	we created the stream interface.
 * @u3v: pointer to the u3v_device struct
 * @max_leader_size: the max required size + alignment for the leader buffer
 * @max_trailer_size: the max required size + alignment for the trailer buffer
 * @payload_size: size of each payload buffer
 * @payload_count: number of payload buffers
 * @transfer1_size: size of transfer1 payload buffer
 * @transfer2_size: size of transfer2 payload buffer
 */
static int write_stream_registers(struct u3v_device *u3v,
	u32 max_leader_size, u32 max_trailer_size, u32 payload_size,
	u32 payload_count, u32 transfer1_size, u32 transfer2_size)
{
	struct u3v_control *control;
	struct device *dev;
	u32 bytes_written;
	int ret;

	if (u3v == NULL)
		return -EINVAL;

	dev = u3v->device;
	GET_INTERFACE(struct u3v_control *, control, u3v->control_info);

	ret = u3v_write_memory(control, sizeof(max_leader_size),
		&bytes_written, u3v->u3v_info->sirm_addr + SI_MAX_LEADER_SIZE,
		&max_leader_size, NULL);
	if (ret != 0)
		goto error;

	ret = u3v_write_memory(control, sizeof(payload_size),
		&bytes_written, u3v->u3v_info->sirm_addr + SI_PAYLOAD_SIZE,
		&payload_size, NULL);
	if (ret != 0)
		goto error;

	ret = u3v_write_memory(control, sizeof(payload_count),
		&bytes_written, u3v->u3v_info->sirm_addr + SI_PAYLOAD_COUNT,
		&payload_count, NULL);
	if (ret != 0)
		goto error;

	ret = u3v_write_memory(control, sizeof(transfer1_size),
		&bytes_written, u3v->u3v_info->sirm_addr + SI_TRANSFER1_SIZE,
		&transfer1_size, NULL);
	if (ret != 0)
		goto error;

	ret = u3v_write_memory(control, sizeof(transfer2_size),
		&bytes_written, u3v->u3v_info->sirm_addr + SI_TRANSFER2_SIZE,
		&transfer2_size, NULL);
	if (ret != 0)
		goto error;

	ret = u3v_write_memory(control, sizeof(max_trailer_size),
		&bytes_written, u3v->u3v_info->sirm_addr + SI_MAX_TRAILER_SIZE,
		&max_trailer_size, NULL);
	if (ret != 0)
		goto error;

	PUT_INTERFACE(u3v->control_info);
	return 0;
error:
	dev_err(dev, "%s: Error setting stream interface registers\n",
		__func__);
	PUT_INTERFACE(u3v->control_info);
	return ret;
}


/*
 * u3v_control_msg - ioctl using control endpoint 0.
 *	Does not require the control interface.
 * @udev: pointer to the usb device
 * @request: request code value
 * @requesttype: request type code value
 * @value: message value
 * @index: message index
 * @data: user buffer
 * @size: size of user buffer
 */
int u3v_control_msg(struct usb_device *udev, __u8 request, __u8 requesttype,
	__u16 value, __u16 index, void *data, __u16 size)
{
	int ret = U3V_ERR_NO_ERROR;
	int bytes_transferred = 0;
	void *kdata = NULL;

	if (data == NULL)
		return -EINVAL;

	kdata = kzalloc(size, GFP_KERNEL);
	if (kdata == NULL)
		return -ENOMEM;

	/*
	 * usb_control_msg returns number of bytes transferred if successful,
	 * or a negative error code upon failure
	 */
	bytes_transferred = usb_control_msg(udev,
		usb_rcvctrlpipe(udev, U3V_CTRL_ENDPOINT), request, requesttype,
		value, index, kdata, size, U3V_TIMEOUT);

	if (bytes_transferred != size) {
		ret = bytes_transferred;
		goto exit;
	}

	if (copy_to_user(data, kdata, size) != 0)
		ret = U3V_ERR_INTERNAL;

exit:
	kfree(kdata);
	return ret;
}


/*
 * compatibility_div - returns quotient. Created to use do_div without
 *	altering the dividend. Using do_div for 64 bit / 32 bit division
 *	to avoid __aebi_uldivmod unresolved symbol for 32 bit OSes.
 */
static u64 compatibility_div(u64 dividend, u32 divisor)
{
	u64 dividend_copy = dividend;
	do_div(dividend_copy, divisor);
	return dividend_copy;
}


/*
 * compatibility_mod - returns remainder. Created to use do_div without
 *	altering the dividend. Using do_div for 64 bit / 32 bit division
 *	to avoid __aebi_uldivmod unresolved symbol for 32 bit OSes.
 */
static u32 compatibility_mod(u64 dividend, u32 divisor)
{
	u64 dividend_copy = dividend;
	return do_div(dividend_copy, divisor);
}


/*
 * u3v_delete - frees device resources
 */
static void u3v_delete(struct kref *kref)
{
	struct u3v_device *u3v = to_u3v_device(kref);

	if (u3v == NULL)
		return;

	mutex_lock(&u3v->control_info.interface_lock);
	u3v_destroy_control(u3v);
	mutex_unlock(&u3v->control_info.interface_lock);

	mutex_lock(&u3v->stream_info.interface_lock);
	u3v_destroy_stream(u3v);
	mutex_unlock(&u3v->stream_info.interface_lock);

	mutex_lock(&u3v->event_info.interface_lock);
	u3v_destroy_events(u3v);
	mutex_unlock(&u3v->event_info.interface_lock);

	usb_put_dev(u3v->udev);
	kfree(u3v->u3v_info);
	kfree(u3v);
}


/*
 * u3v_release - called when a process closes the device file
 */
static int u3v_release(struct inode *inode, struct file *filp)
{
	struct u3v_device *u3v;

	if (filp == NULL)
		return -EINVAL;


	u3v = filp->private_data;

	mutex_lock(&u3v->control_info.interface_lock);
	u3v_destroy_control(u3v);
	mutex_unlock(&u3v->control_info.interface_lock);

	mutex_lock(&u3v->stream_info.interface_lock);
	u3v_destroy_stream(u3v);
	mutex_unlock(&u3v->stream_info.interface_lock);

	mutex_lock(&u3v->event_info.interface_lock);
	u3v_destroy_events(u3v);
	mutex_unlock(&u3v->event_info.interface_lock);

	atomic_inc(&u3v->device_available);
	kref_put(&u3v->kref, u3v_delete);
	filp->private_data = NULL;
	return 0;
}


/*
 * u3v_get_os_max_transfer_size - ioctl to get the max transfer size for
 * the os
 * @u3v - pointer to the u3v struct
 * @os_max - on return, this user pointer will be set to the os max
 *	transfer size
 */
static int u3v_get_os_max_transfer_size(struct u3v_device *u3v, __u32 *os_max)
{
	if (u3v == NULL || os_max == NULL)
		return -EINVAL;

	if (u3v->u3v_info->sirm_addr == 0)
		return U3V_ERR_NO_STREAM_INTERFACE;

	return put_user(u3v->u3v_info->os_max_transfer_size, os_max);
}


/* u3v_get_stream_alignment - ioctl that returns the required URB transfer
 *	size alignment in bytes for the current OS and camera combination.
 * @u3v: pointer to the u3v_device struct
 * @alignment: on return, the required URB transfer size alignment
 */
static int u3v_get_stream_alignment(struct u3v_device *u3v, __u32 *alignment)
{
	if (u3v == NULL || u3v->u3v_info == NULL || alignment == NULL)
		return -EINVAL;

	if (u3v->u3v_info->sirm_addr == 0)
		return U3V_ERR_NO_STREAM_INTERFACE;

	return put_user(u3v->u3v_info->transfer_alignment, alignment);
}


/*
 * find_bulk_endpoint - helper function to iterate through endpoints and
 *	return a bulk endpoint that has the correct direction as specified
 *	by dir
 * @iface_desc: interface descriptor
 * @dir_in: true if looking for an in endpoint, false for an out endpoint
 */
struct usb_endpoint_descriptor *find_bulk_endpoint(struct usb_host_interface
	*iface_desc, enum direction_t dir)
{
	struct usb_endpoint_descriptor *ep;
	int i;

	for (i = 0; i < iface_desc->desc.bNumEndpoints; i++) {
		ep = &iface_desc->endpoint[i].desc;
		if (((dir == dir_in) && usb_endpoint_is_bulk_in(ep)) ||
		    ((dir == dir_out) && usb_endpoint_is_bulk_out(ep))) {
			return ep;
		}
	}
	return NULL;
}


/*
 * populate_u3v_properties - this helper function copies necessary
 *	USB3 data to our u3v_device_info struct
 * @u3v: pointer to the u3v struct
 */
static int populate_u3v_properties(struct u3v_device *u3v)
{
	struct usb_host_interface *alts;
	unsigned char *buffer;
	int buflen;
	struct usb_device *udev = u3v->udev;
	struct u3v_device_info *u3v_info = u3v->u3v_info;

	alts = u3v->intf->cur_altsetting;
	buffer = alts->extra;
	buflen = alts->extralen;

	if ((buflen < MIN_U3V_INFO_LENGTH) ||
	    (buffer[1] != U3V_INTERFACE) ||
	    (buffer[2] != U3V_DEVICEINFO)) {

		dev_err(u3v->device,
			"%s: Failed to get a proper Camera Info descriptor.\n",
			__func__);
		dev_err(u3v->device,
			"Descriptor Type = 0x%02X (Expected 0x%02X)\n",
			buffer[1], U3V_INTERFACE);
		dev_err(u3v->device,
			"Descriptor SubType = 0x%02X (Expected 0x%02X)\n",
			buffer[2], U3V_DEVICEINFO);
		dev_err(u3v->device,
			"Descriptor Length = 0x%02X (Expected 0x%02X)\n",
			buflen, MIN_U3V_INFO_LENGTH);
		return U3V_ERR_INVALID_USB_DESCRIPTOR;
	}

	/* uint32_t gen_cp_version */
	u3v_info->gen_cp_version = le32_to_cpup((const __le32 *)&buffer[3]);

	/* uint32_t u3v_version */
	u3v_info->u3v_version = le32_to_cpup((const __le32 *)&buffer[7]);

	/* char device_guid [U3V_MAX_STR] */
	if (buffer[11] != 0)
		usb_string(udev, buffer[11], u3v_info->device_guid,
			sizeof(u3v_info->device_guid));

	/* char vendor_name [U3V_MAX_STR] */
	if (buffer[12] != 0)
		usb_string(udev, buffer[12], u3v_info->vendor_name,
			sizeof(u3v_info->vendor_name));

	/* char model_name [U3V_MAX_STR] */
	if (buffer[13] != 0)
		usb_string(udev, buffer[13], u3v_info->model_name,
			sizeof(u3v_info->model_name));

	/* char family_name [U3V_MAX_STR] */
	if (buffer[14] != 0)
		usb_string(udev, buffer[14], u3v_info->family_name,
			sizeof(u3v_info->family_name));

	/* char device_version [U3V_MAX_STR] */
	if (buffer[15] != 0)
		usb_string(udev, buffer[15], u3v_info->device_version,
			sizeof(u3v_info->device_version));

	/* char manufacturer_info [U3V_MAX_STR] */
	if (buffer[16] != 0)
		usb_string(udev, buffer[16], u3v_info->manufacturer_info,
			sizeof(u3v_info->manufacturer_info));

	/* char serial_number_u3v [U3V_MAX_STR] */
	if (buffer[17] != 0)
		usb_string(udev, buffer[17], u3v_info->serial_number_u3v,
			sizeof(u3v_info->serial_number_u3v));

	/* char user_defined_name [U3V_MAX_STR] */
	if (buffer[18] != 0)
		usb_string(udev, buffer[18], u3v_info->user_defined_name,
			sizeof(u3v_info->user_defined_name));

	/* uint8_t speed_support */
	u3v_info->speed_support = buffer[19];

	u3v_info->previously_initialized = 0;
	/*
	 * This is a workaround for issues we are seeing with EHCI
	 * on Zynq targets with the 3.2 kernel. Setting the byte alignment
	 * to a page enables us to use bounce buffering instead of DMA
	 * for transfers less than a page in size
	 */
#ifdef ZYNQ_3_2_EHCI_QUIRK
	u3v_info->host_byte_alignment = 4096;
#else
	u3v_info->host_byte_alignment = 1;
#endif

#ifndef ARCH_HAS_SG_CHAIN
	/*
	 * If chaining is not enabled for scatter-gather lists,
	 * we are limited to what would fit in a single page.
	 * The number of sg entries for a single page is
	 * SG_MAX_SINGLE_ALLOC, so with worst case fragmentation
	 * (no adjacent pages to coalesce), the maximum transfer
	 * would be the number of sg entries in a page *
	 * the page size.
	 */
	u3v_info->os_max_transfer_size = SG_MAX_SINGLE_ALLOC * PAGE_SIZE;
#else
	u3v_info->os_max_transfer_size = UINT_MAX;
#endif

	/*
	 * sirm_addr and transfer_alignment cannot be properly initialized
	 * until the control interface is created. In initialize we can
	 * read registers, so then we will populate those values.
	 */
	u3v_info->sirm_addr = 0;
	u3v_info->transfer_alignment = 0;
	return 0;
}


/*
 * enumerate_u3v_interfaces - called by probe to claim the interfaces and
 * store their endpoint information
 * @u3v - pointer to the u3v device struct
 */
static int enumerate_u3v_interfaces(struct u3v_device *u3v)
{
	struct usb_device *udev = u3v->udev;
	struct usb_interface *tmp;
	struct usb_host_interface *iface_desc;
	int num_interfaces, i;
	enum direction_t in = dir_in;
	enum direction_t out = dir_out;
	int ret = 0;

	if (u3v == NULL)
		return -EINVAL;

	num_interfaces = udev->actconfig->desc.bNumInterfaces;

	for (i = 0; i < num_interfaces; i++) {
		tmp = usb_ifnum_to_if(udev, i);
		if (tmp == NULL)
			return U3V_ERR_INVALID_USB_DESCRIPTOR;

		iface_desc = tmp->cur_altsetting;

		switch (iface_desc->desc.bInterfaceProtocol) {
		case U3V_INTERFACE_PROTOCOL_CONTROL:
			/*
			 * control interface does not need to be claimed
			 * because it is the one that probe is detecting
			 */
			u3v->control_info.idx = i;
			u3v->control_info.bulk_in =
				find_bulk_endpoint(iface_desc, in);
			u3v->control_info.bulk_out =
				find_bulk_endpoint(iface_desc, out);
			u3v->control_info.ioctl_count = 0;
			mutex_init(&u3v->control_info.interface_lock);
			mutex_init(&u3v->control_info.ioctl_count_lock);
			init_completion(&u3v->control_info.ioctl_complete);
			complete_all(&u3v->control_info.ioctl_complete);
			break;
		case U3V_INTERFACE_PROTOCOL_EVENT:
			/* claim event interface */
			ret = usb_driver_claim_interface(&u3v_driver, tmp,
				u3v);

			if (ret != 0) {
				dev_err(u3v->device,
					"%s: event interface for device %d is already claimed\n",
					__func__,
					u3v->udev->devnum);
				return -EBUSY;
			}
			u3v->event_info.idx = i;
			u3v->event_info.bulk_in =
				find_bulk_endpoint(iface_desc, in);
			u3v->event_info.bulk_out = NULL;
			u3v->event_info.ioctl_count = 0;
			mutex_init(&u3v->event_info.interface_lock);
			mutex_init(&u3v->event_info.ioctl_count_lock);
			init_completion(&u3v->event_info.ioctl_complete);
			complete_all(&u3v->event_info.ioctl_complete);
			break;
		case U3V_INTERFACE_PROTOCOL_STREAM:
			/* claim stream interface */
			ret = usb_driver_claim_interface(&u3v_driver, tmp,
				u3v);

			if (ret != 0) {
				dev_err(u3v->device,
					"%s: stream interface for device %d is already claimed\n",
					__func__,
					u3v->udev->devnum);
				return -EBUSY;
			}
			u3v->stream_info.idx = i;
			u3v->stream_info.bulk_in =
				find_bulk_endpoint(iface_desc, in);
			u3v->stream_info.bulk_out = NULL;
			u3v->stream_info.ioctl_count = 0;
			mutex_init(&u3v->stream_info.interface_lock);
			mutex_init(&u3v->stream_info.ioctl_count_lock);
			init_completion(&u3v->stream_info.ioctl_complete);
			complete_all(&u3v->stream_info.ioctl_complete);
			break;
		}
	}
	return 0;
}


/*
 * u3v_probe - called by the usb core if a device has an interface that
 *	matches the device table
 * @interface: matching usb_interface
 * @id: matching usb_device_id
 */
static int u3v_probe(struct usb_interface *interface,
	const struct usb_device_id *id)
{
	struct u3v_device *u3v;
	struct u3v_device_info *u3v_info;
	const char *hcd;
	int ret = 0;

	/* Allocate memory for the device. */
	u3v = kzalloc(sizeof(*u3v), GFP_KERNEL);
	if (u3v == NULL)
		return -ENOMEM;

	/* Allocate memory for the U3V device info. */
	u3v_info = kzalloc(sizeof(*u3v_info), GFP_KERNEL);
	if (u3v_info == NULL) {
		kfree(u3v);
		return -ENOMEM;
	}

	/* Initialize members of struct u3v_device */
	u3v->udev = usb_get_dev(interface_to_usbdev(interface));
	u3v->intf = usb_get_intf(interface);
	u3v->device = &u3v->udev->dev;
	u3v->u3v_driver = &u3v_driver;
	u3v->u3v_info = u3v_info;
	kref_init(&u3v->kref);
	atomic_set(&u3v->device_available, 1);
	u3v->device_connected = true;

	/*
	 * xhci has a bug with stalling and clearing endpoints
	 * if the stall is initiated by software. For now, we only stall
	 * if the driver is not xhci. If this is fixed in the future,
	 * we should enable it for xhci for the fixed verion and later
	 */
	hcd = bus_to_hcd(u3v->udev->bus)->driver->description;
	if (strncmp(hcd, "xhci_hcd", 8) == 0)
		u3v->stalling_disabled = true;
	else
		u3v->stalling_disabled = false;

	/* Populate u3v_info */
	if (populate_u3v_properties(u3v) < 0) {
		dev_err(u3v->device, "%s: Error populating u3v device info\n",
			__func__);
		ret = U3V_ERR_INVALID_USB_DESCRIPTOR;
		goto error;
	}

	ret = enumerate_u3v_interfaces(u3v);
	if (ret < 0)
		goto error;

	ret = sysfs_create_group(&interface->dev.kobj, &u3v_attr_grp);
	if (ret < 0)
		goto error;

	ret = usb_register_dev(u3v->intf, &u3v_class);

	if (ret < 0) {
		dev_err(u3v->device, "%s: Failed to register device\n",
			__func__);
		goto error;
	}

	dev_info(u3v->device, "%s: Registering device %d\n",
		DRIVER_DESC, u3v->udev->devnum);

	/* Save our data pointer in this interface device */
	usb_set_intfdata(interface, u3v);

	return ret;
error:
	sysfs_remove_group(&interface->dev.kobj, &u3v_attr_grp);
	kref_put(&u3v->kref, u3v_delete);
	return ret;
}


/*
 * reset_pipe - resets an interface's endpoint(s)
 * @u3v: pointer to the u3v_device struct
 * @iface_info: interface with endpoint(s) to reset
 */
int reset_pipe(struct u3v_device *u3v, struct u3v_interface_info *iface_info)
{
	int ret = 0;

	if (u3v == NULL || iface_info == NULL)
		return -EINVAL;

	if (!u3v->device_connected)
		return 0;

	if (iface_info->bulk_in != NULL) {
		ret = reset_endpoint(u3v, iface_info->bulk_in);
		if (ret != 0)
			goto exit;
	}

	if (iface_info->bulk_out != NULL)
		ret = reset_endpoint(u3v, iface_info->bulk_out);

exit:
	return ret;
}

/*
 * reset_endpoint - resets an endpoint for the device
 * @u3v: pointer to the u3v_device struct
 * @ep: endpoint to be reset
 */
static int reset_endpoint(struct u3v_device *u3v,
	struct usb_endpoint_descriptor *ep)
{
	struct usb_device *udev;
	u8 ep_addr;
	int ret;

	if (u3v == NULL)
		return -EINVAL;

	udev = u3v->udev;
	ep_addr = ep->bEndpointAddress;

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
		dev_err(u3v->device, "%s: Error %d from stalling ep %02X\n",
			__func__, ret, ep_addr);
		return ret;
	}

	if (usb_endpoint_is_bulk_in(ep))
		ret = usb_clear_halt(udev, usb_rcvbulkpipe(udev, ep_addr));
	else
		ret = usb_clear_halt(udev, usb_sndbulkpipe(udev, ep_addr));

	if (ret != 0) {
		dev_err(u3v->device, "%s: Error %d resetting ep %02X\n",
			__func__, ret, ep_addr);
	}
	return ret;
}


/*
 * u3v_disconnect - called by the usb core when the interface has been
 *	removed from the system or the driver is being unloaded
 * @interface: interface to be removed
 */
static void u3v_disconnect(struct usb_interface *interface)
{
	struct u3v_device *u3v = usb_get_intfdata(interface);
	/*
	 * Since we bind to each interface, we receive separate
	 * disconnect callbacks for each one. We are guaranteed to
	 * have at least one interface, so there should always be a
	 * valid interface 0. We only want to take these actions
	 * one time per device, so we only do so in the case
	 * that we have interface 0.
	 */
	if (interface->cur_altsetting->desc.bInterfaceNumber == 0) {
		dev_info(u3v->device,
		"%s: U3V device removed\n", __func__);

		usb_deregister_dev(interface, &u3v_class);
		sysfs_remove_group(&interface->dev.kobj, &u3v_attr_grp);
		u3v->device_connected = false;

		kref_put(&u3v->kref, u3v_delete);
	}
}


/*
 * u3v_init - register the u3v driver
 */
static int __init u3v_init(void)
{
	int result;

	/* register this driver with the USB subsystem */
	result = usb_register(&u3v_driver);
	if (result == 0)
		pr_info(DRIVER_DESC"\n");

	return result;
}


/*
 * u3v_exit - deregister the u3v driver
 */
static void __exit u3v_exit(void)
{
	usb_deregister(&u3v_driver);
}

module_init(u3v_init);
module_exit(u3v_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("National Instruments");
MODULE_DESCRIPTION(DRIVER_DESC);
