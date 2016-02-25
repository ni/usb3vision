# usb3vision
Driver for USB3 Vision(TM) class devices

This driver is the kernel piece of a larger driver intended to acquire images from USB3 Vision class devices. USB3 Vision utilizes 
GenICam, a usermode generic programming interface that translates high-level actions into register read/write operations. Therefore, 
significant application logic outside of this kernel module is needed to incorporate GenICam and be fully compatible with the
USB3Vision specification.

Devices are enumerated at /sys/class/usbmisc/u3vX, where X is a number associated with the camera. From there you can check ./device/device_guid and ensure that you've found the device you are looking for. Then you can open /dev/devX to get a file handle to the device that you can use for interacting with the ioctl interface.

The ioctl interface is provided in u3v_shared.h:
* U3V_IOCTL_READ
* U3V_IOCTL_WRITE
* U3V_IOCTL_GET_STREAM_ALIGNMENT
* U3V_IOCTL_GET_OS_MAX_TRANSFER
* U3V_IOCTL_CONFIGURE_STREAM
* U3V_IOCTL_UNCONFIGURE_STREAM
* U3V_IOCTL_CONFIGURE_BUFFER
* U3V_IOCTL_UNCONFIGURE_BUFFER
* U3V_IOCTL_QUEUE_BUFFER
* U3V_IOCTL_WAIT_FOR_BUFFER
* U3V_IOCTL_CANCEL_ALL_BUFFERS
* U3V_IOCTL_CTRL_MSG
* U3V_IOCTL_START_EVENTS
* U3V_IOCTL_WAIT_FOR_EVENT
* U3V_IOCTL_STOP_EVENTS
* U3V_IOCTL_CONFIGURE_STREAM2

These functions take a struct as a parameter with any necessary information.

For example, let's look at configuring a buffer. The struct associated with that call looks like this:

    struct u3v_configure_buffer {
      void __user *u_image_buffer;
      void __user *u_chunk_data_buffer;
      __u64 __user *u_buffer_handle;
    };


Usermode code would create that struct and fill it in, then call the ioctl with the file descriptor you got from opening the device,
the macro from u3v_shared, and the struct.

Here's an example:

    // Do this just once, at the beginning
    char* devicePath = "/dev/u3vX";
    int fd = open(devicePath, O_RDWR);
    if (fd == -1) {
      // error handling
    }
    ----------------------------------------------------------
    // Then ioctl calls would be something like this
    struct u3v_configure_buffer configureBuffer;
    
    configureBuffer.u_image_buffer = imageBuffer;           // pointer to user image data buffer
    configureBuffer.u_chunk_data_buffer = chunkDataBuffer;  // pointer to user chunk data buffer
    configureBuffer.u_buffer_handle = bufferHandle;         // pointer to uint64_t handle that will be populated by the kernel driver
    
    int ret = ioctl(fd, U3V_IOCTL_CONFIGURE_BUFFER, &configureBuffer);
    if (ret != 0) {
      // error handling
    }
