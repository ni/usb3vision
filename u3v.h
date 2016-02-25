/*
 * u3v.h: Driver for USB3 Vision(TM) class devices
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

#ifndef _U3V_H_
#define _U3V_H_

/*
 * pr_fmt has to be redefined prior to including linux/kernel.h to prevent
 * a compiler warning for its redefinition
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/ioctl.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/kernel.h>
#include <linux/kref.h>


/* General device info */
#define DRIVER_DESC "USB3 Vision Driver"
#define U3V_DEVICE_CLASS 0xEF
#define U3V_DEVICE_SUBCLASS 0x02
#define U3V_DEVICE_PROTOCOL 0x01
#define U3V_INTERFACE_CLASS 0xEF
#define U3V_INTERFACE_SUBCLASS 0x05
#define U3V_INTERFACE_PROTOCOL_CONTROL 0x00
#define U3V_INTERFACE_PROTOCOL_EVENT 0x01
#define U3V_INTERFACE_PROTOCOL_STREAM 0x02
#define U3V_INTERFACE 0x24
#define U3V_DEVICEINFO 0x01
#define MIN_U3V_INFO_LENGTH 20
#define U3V_MINOR_BASE 208
#define U3V_MAX_STR 64
#define U3V_REQUEST_ACK 0x4000
#define U3V_TIMEOUT 5000
#define	U3V_DEV_DISCONNECTED 1


struct u3v_interface_info {
	u8 idx;
	struct usb_endpoint_descriptor *bulk_in;
	struct usb_endpoint_descriptor *bulk_out;
	struct mutex interface_lock;
	struct mutex ioctl_count_lock;
	int ioctl_count;
	struct completion ioctl_complete;
	void *interface_ptr;
};


/*
 * A copy of this structure exists for each U3V device to contain its
 * private data.
 */
struct u3v_device {
	struct usb_device *udev;
	struct usb_interface *intf;
	struct device *device;
	struct kref kref;
	struct u3v_device_info *u3v_info; /* device attributes */
	struct usb_driver *u3v_driver;
	int device_state;
	atomic_t device_available;
	bool stalling_disabled;
	bool device_connected;
	struct u3v_interface_info control_info;
	struct u3v_interface_info event_info;
	struct u3v_interface_info stream_info;
};


#define to_u3v_device(d) container_of(d, struct u3v_device, kref)

/*
 * Each u3v_device contains a u3v_device_info struct to store
 * device attributes
 */
struct u3v_device_info {
	u32 gen_cp_version;
	u32 u3v_version;
	char device_guid[U3V_MAX_STR];
	char vendor_name[U3V_MAX_STR];
	char model_name[U3V_MAX_STR];
	char family_name[U3V_MAX_STR];
	char device_version[U3V_MAX_STR];
	char manufacturer_info[U3V_MAX_STR];
	char serial_number_u3v[U3V_MAX_STR];
	char user_defined_name[U3V_MAX_STR];
	u8 speed_support;
	u8 previously_initialized;
	u32 host_byte_alignment;
	u32 os_max_transfer_size;
	u64 sirm_addr;
	u32 transfer_alignment;
	u32 segmented_xfer_supported;
	u32 segmented_xfer_enabled;
};

/* Helper functions for interfaces */
int reset_pipe(struct u3v_device *u3v, struct u3v_interface_info *iface_info);

static inline void u3v_reinit_completion(struct completion *x)
{
#ifdef INIT_COMPLETION
	INIT_COMPLETION(*x);
#else
	reinit_completion(x);
#endif
}

/* Helper function for sysfs attributes */
static inline struct u3v_device *u3v_get_driver_data(struct device *dev)
{
	struct usb_interface *intf;

	if (dev == NULL)
		return NULL;

	intf = to_usb_interface(dev);
	return intf ? usb_get_intfdata(intf) : NULL;
}

/* Helper function for checking hcd */
static inline bool hcd_is_xhci(struct usb_device *udev)
{
	const char *hcd = bus_to_hcd(udev->bus)->driver->description;
	return (strncmp(hcd, "xhci_hcd", 8) == 0);
}

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

#endif /*  _U3V_H_ */
