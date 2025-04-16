/*
 * u3v_control.c: Driver for USB3 Vision(TM) class devices
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
#include "u3v_shared.h"
#include <linux/slab.h>
#include <linux/atomic.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sysfs.h>
#include <linux/usb.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0)
	#include <linux/unaligned.h>
#else
	#include <asm/unaligned.h>
#endif

/*
 * u3v_create_control - Initializes the control interface.
 * @u3v: pointer to the u3v_device struct
 */
int u3v_create_control(struct u3v_device *u3v)
{
	struct u3v_control *ctrl = NULL;
	struct device *dev = u3v->device;
	int bytes_read = 0;
	u64 sbrm_address = 0;
	__le32 max_response;
	__le32 cmd_buffer_size;
	__le32 ack_buffer_size;
	int ret = 0;

	if (u3v == NULL)
		return -EINVAL;

	if (!u3v->device_connected)
		return -ENODEV;

	if (u3v->control_info.interface_ptr != NULL) {
		dev_err(u3v->device,
			"%s: Control interface already created\n", __func__);
		return U3V_ERR_INTERNAL;
	}

	if (u3v->control_info.bulk_in == NULL ||
		u3v->control_info.bulk_out == NULL) {

		dev_err(u3v->device,
			"%s: Did not detect bulk in and bulk out endpoints in the control interface.\n",
			__func__);
		return U3V_ERR_NO_CONTROL_INTERFACE;
	}

	ctrl = kzalloc(sizeof(struct u3v_control), GFP_KERNEL);
	if (ctrl == NULL)
		return -ENOMEM;

	ctrl->u3v_dev = u3v;

	mutex_init(&ctrl->read_write_lock);

	ctrl->u3v_timeout = U3V_TIMEOUT;

	ctrl->max_ack_transfer_size =
		min(((1 << 16) + sizeof(struct ack_header)),
		(size_t)0xFFFFFFFF);

	ctrl->max_cmd_transfer_size =
		min(((1 << 16) + sizeof(struct command_header)),
		(size_t)0xFFFFFFFF);

	/* id is preincremented so we want it to start at 0 */
	ctrl->request_id = -1;

	/* Set the maximum id value so that we can skip 0 once we overflow */
	ctrl->max_request_id = -1;

	ctrl->ack_buffer =
		kzalloc(ctrl->max_ack_transfer_size, GFP_KERNEL);
	ctrl->cmd_buffer =
		kzalloc(ctrl->max_cmd_transfer_size, GFP_KERNEL);

	if (ctrl->ack_buffer == NULL || ctrl->cmd_buffer == NULL) {
		dev_err(dev, "%s: Error allocating control buffers\n",
			__func__);
		ret = -ENOMEM;
		goto error;
	}

	if (!u3v->stalling_disabled &&
	    u3v->u3v_info->legacy_ctrl_ep_stall_enabled)
		reset_pipe(u3v, &u3v->control_info);

	/*
	 * Query the device for max response time, and max buffer sizes,
	 * updating control attributes if needed
	 */
	ret = u3v_read_memory(ctrl, sizeof(__le32), &bytes_read,
		ABRM_MAX_DEVICE_RESPONSE_TIME, &max_response);

	if ((ret < 0) || (bytes_read != sizeof(__le32))) {
		dev_err(dev, "%s: Error reading max device response time\n",
			__func__);
		goto error;
	}

	ctrl->u3v_timeout = max((u32)(U3V_TIMEOUT),
		le32_to_cpu(max_response));

	ret = u3v_read_memory(ctrl, sizeof(u64), &bytes_read,
		ABRM_SBRM_ADDRESS, &sbrm_address);

	if ((ret < 0) || (bytes_read != sizeof(u64))) {
		dev_err(dev, "%s: Error reading SBRM address\n", __func__);
		goto error;
	}

	ret = u3v_read_memory(ctrl, sizeof(__le32), &bytes_read,
		sbrm_address + SBRM_MAX_CMD_TRANSFER, &cmd_buffer_size);

	if ((ret < 0) || (bytes_read != sizeof(__le32))) {
		dev_err(dev, "%s: Error reading maximum command transfer size\n",
			__func__);
		goto error;
	}

	ctrl->max_cmd_transfer_size = min(ctrl->max_cmd_transfer_size,
		le32_to_cpu(cmd_buffer_size));

	ret = u3v_read_memory(ctrl, sizeof(__le32), &bytes_read,
		sbrm_address + SBRM_MAX_ACK_TRANSFER, &ack_buffer_size);

	if ((ret < 0) || (bytes_read != sizeof(__le32))) {
		dev_err(dev, "%s: Error reading maximum ack transfer size\n",
			__func__);
		goto error;
	}

	ctrl->max_ack_transfer_size = min(ctrl->max_ack_transfer_size,
		le32_to_cpu(ack_buffer_size));

	u3v->control_info.interface_ptr = ctrl;
	return ret;

error:
	kfree(ctrl->ack_buffer);
	kfree(ctrl->cmd_buffer);
	kfree(ctrl);
	return ret;
}


/*
 * u3v_destroy_control - destroys the control interface
 * @u3v: pointer to the u3v_device struct
 */
void u3v_destroy_control(struct u3v_device *u3v)
{
	struct u3v_control *ctrl;

	if (u3v == NULL)
		return;

	ctrl = (struct u3v_control *)(u3v->control_info.interface_ptr);
	if (ctrl == NULL)
		return;

	wait_for_completion(&u3v->control_info.ioctl_complete);

	if (!u3v->stalling_disabled &&
	    u3v->u3v_info->legacy_ctrl_ep_stall_enabled)
		reset_pipe(u3v, &u3v->control_info);

	kfree(ctrl->ack_buffer);
	kfree(ctrl->cmd_buffer);
	kfree(ctrl);
	u3v->control_info.interface_ptr = NULL;
}


/*
 * u3v_read_memory - ioctl to read device registers.
 * @ctrl: pointer to control interface
 * @transfer_size: number of bytes to read
 * @bytes_read: number of bytes read
 * @address: camera's memory address to be read
 * @buffer: kernel buffer to be read into
 */
int u3v_read_memory(struct u3v_control *ctrl, u32 transfer_size,
	u32 *bytes_read, u64 address, void *buffer)
{
	struct u3v_device *u3v = NULL;
	struct device *dev = NULL;
	int max_bytes_per_read = 0;
	struct ack *ack = NULL;
	struct pending_ack_payload *pending_ack = NULL;
	int actual = 0;
	int ret = 0;
	int total_bytes_read = 0;
	bool request_acknowledged = false;
	size_t cmd_buffer_size = sizeof(struct command_header) +
		sizeof(struct read_mem_cmd_payload);
	size_t ack_buffer_size = sizeof(struct ack_header) +
		transfer_size;

	if (buffer == NULL)
		return -EINVAL;

	if (bytes_read != NULL)
		*bytes_read = 0;

	if (transfer_size == 0)
		return 0;

	if (ctrl == NULL)
		return U3V_ERR_NO_CONTROL_INTERFACE;

	max_bytes_per_read = ctrl->max_ack_transfer_size -
		sizeof(struct ack_header);
	u3v = ctrl->u3v_dev;
	dev = u3v->device;

	if (cmd_buffer_size > ctrl->max_cmd_transfer_size) {
		dev_err(dev,
			"%s: Requested command buffer of size %zu, but maximum size is %d\n",
			__func__, cmd_buffer_size, ctrl->max_cmd_transfer_size);
		return -EINVAL;
	}

	if (max_bytes_per_read <= 0) {
		dev_err(dev, "%s: Requested ack buffer of size <= 0\n",
			__func__);
		return -EINVAL;
	}

	dev_dbg(dev, "u3v_read_memory: address = %llX, transfer_size = %d",
		address, transfer_size);

	mutex_lock(&ctrl->read_write_lock);

	while (total_bytes_read < transfer_size) {
		u32 bytes_this_iteration =
			min((int)(transfer_size - total_bytes_read),
				max_bytes_per_read);

		struct command *command = (struct command *)(ctrl->cmd_buffer);
		struct read_mem_cmd_payload *payload =
			(struct read_mem_cmd_payload *)(command->payload);
		command->header.prefix = cpu_to_le32(U3V_CONTROL_PREFIX);
		command->header.flags = cpu_to_le16(U3V_REQUEST_ACK);
		command->header.cmd = cpu_to_le16(READMEM_CMD);
		command->header.length =
			cpu_to_le16(sizeof(struct read_mem_cmd_payload));
		if (ctrl->request_id + 1 == ctrl->max_request_id) {
			ctrl->request_id = 0;
		}
		command->header.request_id = cpu_to_le16(++(ctrl->request_id));
		payload->address = address + total_bytes_read;
		payload->reserved = 0;
		payload->byte_count = cpu_to_le16(bytes_this_iteration);

		ret = usb_bulk_msg(u3v->udev,
			usb_sndbulkpipe(u3v->udev,
			usb_endpoint_num(u3v->control_info.bulk_out)),
			ctrl->cmd_buffer, cmd_buffer_size,
			&actual, ctrl->u3v_timeout);

		if (ret < 0) {
			dev_err(dev, "%s: Error %d from usb_bulk_msg out\n",
				__func__, ret);
			if (!u3v->stalling_disabled)
				reset_pipe(u3v, &u3v->control_info);
			goto exit;
		}

		request_acknowledged = false;
		while (!request_acknowledged) {
			/* Reset.
			 * Ensure that we have enough room for a
			 * pending_ack_payload (e.g. if we try to
			 * read only two bytes then we would not be
			 * able to receive a pending_ack_payload,
			 * which has a size greater than 2 bytes).
			 */
			actual = 0;
			ack_buffer_size = sizeof(struct ack_header) +
				max((size_t)(bytes_this_iteration),
				sizeof(struct pending_ack_payload));
			memset(ctrl->ack_buffer, 0, ack_buffer_size);

			/* read the ack */
			ret = usb_bulk_msg(u3v->udev,
				usb_rcvbulkpipe(u3v->udev,
				usb_endpoint_num(u3v->control_info.bulk_in)),
				ctrl->ack_buffer, ack_buffer_size, &actual,
				ctrl->u3v_timeout);

			ack = (struct ack *)(ctrl->ack_buffer);

			/*
			 * Validate that we read enough bytes to process the
			 * header and that this seems like a valid GenCP
			 * response
			 */
			if (ret < 0) {
				dev_err(dev, "%s: Error %d from usb_bulk_msg in\n",
					__func__, ret);
				if (!u3v->stalling_disabled)
					reset_pipe(u3v, &u3v->control_info);

				goto exit;
			}
			if ((actual < sizeof(struct ack_header)) ||
				(ack->header.prefix != U3V_CONTROL_PREFIX) ||
				(actual != ack->header.length +
				sizeof(struct ack_header))) {

				ret = U3V_ERR_INVALID_DEVICE_RESPONSE;
				goto exit;
			}

			/*
			 * Fix for broken Basler cameras where they can get in a
			 * state where there is an extra bogus response on the
			 * pipe that needs to be thrown away. We just submit
			 * another read in that case.
			 */
			if (ack->header.ack_id == (ctrl->request_id - 1))
				continue;

			/* Inspect the acknowledge buffer */
			if (((ack->header.cmd != READMEM_ACK) &&
				(ack->header.cmd != PENDING_ACK)) ||
				(ack->header.status != U3V_ERR_NO_ERROR) ||
				(ack->header.ack_id != ctrl->request_id) ||
				((ack->header.cmd == READMEM_ACK) &&
				(ack->header.length != bytes_this_iteration)) ||
				((ack->header.cmd == PENDING_ACK) &&
				(ack->header.length !=
				sizeof(struct pending_ack_payload)))) {

				dev_err(dev,
					"%s: received an invalid READMEM_ACK buffer\n",
					__func__);
				dev_err(dev, "\tPrefix = 0x%X, expected 0x%X\n",
					ack->header.prefix, U3V_CONTROL_PREFIX);
				dev_err(dev, "\tCmd = 0x%X, expected 0x%X or 0x%X\n",
					ack->header.cmd, READMEM_ACK,
					PENDING_ACK);
				dev_err(dev, "\tStatus = 0x%X, expected 0x%X\n",
					ack->header.status, U3V_ERR_NO_ERROR);
				dev_err(dev, "\tAck Id = 0x%X, expected 0x%X\n",
					ack->header.ack_id, ctrl->request_id);
				dev_err(dev, "\tPayload length = %d, expected %zu\n",
					ack->header.length,
					ack->header.cmd == PENDING_ACK ?
					sizeof(struct pending_ack_payload) :
					bytes_this_iteration);
				if (ack->header.status != U3V_ERR_NO_ERROR)
					ret = U3V_ERR_BASE + ack->header.status;
				else
					ret = U3V_ERR_INVALID_DEVICE_RESPONSE;

				goto exit;
			}

			/*
			 * For a pending ack, update the timeout and resubmit
			 * the read request
			 */
			if (ack->header.cmd == PENDING_ACK) {
				dev_dbg(dev, "%s: received pending ack, resubmitting\n",
					__func__);
				pending_ack = (struct pending_ack_payload *)
					(ack->payload);
				ctrl->u3v_timeout = max(ctrl->u3v_timeout,
					(u32)(le16_to_cpu(
						pending_ack->timeout)));
				continue;
			}

			/* Acknowledge received successfully */
			request_acknowledged = true;
		}

		total_bytes_read += bytes_this_iteration;
	}

	if (total_bytes_read != transfer_size)
		dev_err(dev, "%s: total_bytes != xfer: total is %d and xfer is %d",
			__func__, total_bytes_read, transfer_size);

	/* Extract the data */
	memcpy(buffer, ack->payload, total_bytes_read);

exit:
	if (bytes_read != NULL)
		*bytes_read = total_bytes_read;

	mutex_unlock(&ctrl->read_write_lock);
	return ret;
}


/*
 * u3v_write_memory - ioctl to write device registers
 * @ctrl: pointer to control interface
 * @transfer_size: number of bytes to write
 * @bytes_written: number of bytes written
 * @address: camera's memory address to be written to
 * @buffer: kernel buffer to be written from
 */
int u3v_write_memory(struct u3v_control *ctrl, u32 transfer_size,
	u32 *bytes_written, u64 address, const void *buffer)
{
	struct u3v_device *u3v = NULL;
	struct device *dev = NULL;
	int max_bytes_per_write = 0;
	int ret = 0;
	int total_bytes_written = 0;
	size_t cmd_buffer_size = sizeof(struct command_header) +
		sizeof(struct read_mem_cmd_payload);
	/*
	 * Ensure that our acknowledge buffer is big enough to hold either a
	 * write_mem_ack_payload or a pending_ack_payload because we don't know
	 * for sure which one we will get.
	 */
	size_t ack_buffer_size = sizeof(struct ack_header) +
		max(sizeof(struct write_mem_ack_payload),
		sizeof(struct pending_ack_payload));
	struct ack *ack = NULL;
	struct pending_ack_payload *pending_ack = NULL;
	struct write_mem_ack_payload *write_mem_ack = NULL;
	bool request_acknowledged = false;
	int actual = 0;

	if (buffer == NULL)
		return -EINVAL;

	if (ctrl == NULL)
		return U3V_ERR_NO_CONTROL_INTERFACE;

	u3v = ctrl->u3v_dev;
	dev = u3v->device;
	max_bytes_per_write = ctrl->max_cmd_transfer_size -
		(sizeof(struct command_header) +
		 sizeof(struct write_mem_cmd_payload));

	if (bytes_written != NULL)
		*bytes_written = 0;

	if (transfer_size == 0)
		return 0;

	if (ack_buffer_size > ctrl->max_ack_transfer_size) {
		dev_err(dev,
			"%s: Requested ack buffer of size %zu, but maximum size is %d\n",
			__func__, ack_buffer_size, ctrl->max_ack_transfer_size);
		ret = -EINVAL;
		goto exit;
	}

	if (max_bytes_per_write <= 0) {
		dev_err(dev, "%s: Requested cmd buffer of size <= 0\n",
			__func__);
		ret = -EINVAL;
		goto exit;
	}

	dev_dbg(dev, "%s: write mem: address = %llX, transfer_size = %d",
		__func__, address, transfer_size);

	mutex_lock(&ctrl->read_write_lock);

	while (total_bytes_written < transfer_size) {
		u32 bytes_this_iteration =
			min((int)(transfer_size - total_bytes_written),
			     max_bytes_per_write);

		struct command *command = (struct command *)(ctrl->cmd_buffer);
		struct write_mem_cmd_payload *payload =
			(struct write_mem_cmd_payload *)(command->payload);
		command->header.prefix = cpu_to_le32(U3V_CONTROL_PREFIX);
		command->header.flags = cpu_to_le16(U3V_REQUEST_ACK);
		command->header.cmd = cpu_to_le16(WRITEMEM_CMD);
		command->header.length =
			cpu_to_le16(sizeof(struct write_mem_cmd_payload) +
			bytes_this_iteration);
		if (ctrl->request_id + 1 == ctrl->max_request_id) {
			ctrl->request_id = 0;
		}
		command->header.request_id = cpu_to_le16(++(ctrl->request_id));
		payload->address = cpu_to_le64(address + total_bytes_written);
		memcpy(payload->data, (u8 *)(buffer + total_bytes_written),
			bytes_this_iteration);
		cmd_buffer_size = sizeof(struct command_header) +
			sizeof(struct write_mem_cmd_payload) +
			bytes_this_iteration;
		ret = usb_bulk_msg(u3v->udev,
			usb_sndbulkpipe(u3v->udev,
			usb_endpoint_num(u3v->control_info.bulk_out)),
			ctrl->cmd_buffer, cmd_buffer_size,
			&actual, ctrl->u3v_timeout);

		if (ret < 0) {
			dev_err(dev, "%s: Error %d from usb_bulk_msg out\n",
				__func__, ret);
			if (!u3v->stalling_disabled)
				reset_pipe(u3v, &u3v->control_info);
			goto exit;
		}

		request_acknowledged = false;
		while (!request_acknowledged) {
			/* reset */
			actual = 0;
			memset(ctrl->ack_buffer, 0, ack_buffer_size);

			/* read the ack */
			ret = usb_bulk_msg(u3v->udev,
				usb_rcvbulkpipe(u3v->udev,
				usb_endpoint_num(u3v->control_info.bulk_in)),
				ctrl->ack_buffer, ack_buffer_size, &actual,
				ctrl->u3v_timeout);

			ack = (struct ack *)(ctrl->ack_buffer);

			/*
			 * Validate that we read enough bytes to process the
			 * header and that this seems like a valid GenCP
			 * response
			 */
			if (ret < 0) {
				dev_err(dev, "%s: Error %d from usb_bulk_msg in\n",
					__func__, ret);
				if (!u3v->stalling_disabled)
					reset_pipe(u3v, &u3v->control_info);
				goto exit;
			}
			if ((actual < sizeof(struct ack_header)) ||
				(ack->header.prefix != U3V_CONTROL_PREFIX) ||
				(actual != ack->header.length +
				sizeof(struct ack_header))) {

				ret = U3V_ERR_INVALID_DEVICE_RESPONSE;
				goto exit;
			}

			/*
			 * Fix for broken Basler cameras where they can get in a
			 * state where there is an extra bogus response on the
			 * pipe that needs to be thrown away. We just submit
			 * another read in that case.
			 */
			if (ack->header.ack_id == (ctrl->request_id - 1))
				continue;

			write_mem_ack = (struct write_mem_ack_payload *)
				(ack->payload);
			/* Inspect the acknowledge buffer */
			if (((ack->header.cmd != WRITEMEM_ACK) &&
				(ack->header.cmd != PENDING_ACK)) ||
				(ack->header.status != U3V_ERR_NO_ERROR) ||
				(ack->header.ack_id != ctrl->request_id) ||
				((ack->header.cmd == WRITEMEM_ACK) &&
				(ack->header.length !=
				sizeof(struct write_mem_ack_payload)) &&
				(ack->header.length != 0)) ||
				((ack->header.cmd == PENDING_ACK) &&
				(ack->header.length !=
				sizeof(struct pending_ack_payload))) ||
				((ack->header.cmd == WRITEMEM_ACK) &&
				(ack->header.length ==
				sizeof(struct write_mem_ack_payload)) &&
				(write_mem_ack->bytes_written !=
				bytes_this_iteration))) {

				dev_err(dev,
					"%s: received an invalid WRITEMEM_ACK buffer\n",
					__func__);
				dev_err(dev, "\tPrefix = 0x%X, expected 0x%X\n",
					ack->header.prefix, U3V_CONTROL_PREFIX);
				dev_err(dev, "\tCmd = 0x%X, expected 0x%X or 0x%X\n",
					ack->header.cmd, WRITEMEM_ACK,
					PENDING_ACK);
				dev_err(dev, "\tStatus = 0x%X, expected 0x%X\n",
					ack->header.status, U3V_ERR_NO_ERROR);
				dev_err(dev, "\tAck Id = 0x%X, expected 0x%X\n",
					ack->header.ack_id, ctrl->request_id);
				if (ack->header.cmd == WRITEMEM_ACK) {
					dev_err(dev, "\tPayload Length = %d, expected %zu\n",
						ack->header.length,
						sizeof(
						struct write_mem_ack_payload));
					dev_err(dev, "\tBytes Written = %d, expected %d\n",
						write_mem_ack->bytes_written,
						bytes_this_iteration);
				}
				if (ack->header.cmd == PENDING_ACK) {
					dev_err(dev, "\tPayload Length = %d, expected %zu\n",
						ack->header.length,
						sizeof(
						struct pending_ack_payload));
				}
				if (ack->header.status != U3V_ERR_NO_ERROR)
					ret = U3V_ERR_BASE + ack->header.status;
				else
					ret = U3V_ERR_INVALID_DEVICE_RESPONSE;

				goto exit;
			}

			/*
			 * For a pending ack, update the timeout and resubmit
			 * the read request
			 */
			if (ack->header.cmd == PENDING_ACK) {
				dev_dbg(dev, "%s: received pending ack, resubmitting\n",
					__func__);
				pending_ack = (struct pending_ack_payload *)
					(ack->payload);
				ctrl->u3v_timeout = max(ctrl->u3v_timeout,
					(u32)
					(le16_to_cpu(pending_ack->timeout)));
				continue;
			}

			/* Acknowledge received successfully */
			request_acknowledged = true;
		}

		total_bytes_written += bytes_this_iteration;
	}

	if (total_bytes_written != transfer_size)
		dev_err(dev, "%s: wrote %d bytes, but expected %d",
			__func__, total_bytes_written, transfer_size);

exit:
	if (bytes_written != NULL)
		*bytes_written = total_bytes_written;

	mutex_unlock(&ctrl->read_write_lock);
	return ret;
}
