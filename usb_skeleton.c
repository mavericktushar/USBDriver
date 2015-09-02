/*
 * USB Skeleton driver
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, version 2.
 *
 * This driver is based on the 3.19.0-15 kernel
 *
 * Linux Device Driver 3, usb-skeleton.c code for kerenel 2.6.3 
 * has been written for 3.19.0-15
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/usb.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
//#include <linux/config.h>
#include <linux/kref.h>
#include <linux/kernel.h>
//#include <linux/smp_lock.h>


MODULE_LICENSE("GPL");

/* Device Vendor Id and Product Id */
#define USB_SKEL_VENDOR_ID  0x058f
#define USB_SKEL_PRODUCT_ID 0x6387

/* table of devices that work with this driver */
static struct usb_device_id skel_table [] = {
	{ USB_DEVICE(USB_SKEL_VENDOR_ID, USB_SKEL_PRODUCT_ID) },
	{}
};

MODULE_DEVICE_TABLE(usb, skel_table);

#define USB_SKEL_MINOR_BASE	192

struct usb_skel {
	struct usb_device *udev;			/* usb device */
	struct usb_interface *interface;	/* usb_interface */
	unsigned char * bulk_in_buffer;		/* buffer to receive data */
	size_t		  bulk_in_size;         /* size of buffer */
	__u8 	      bulk_in_endpointAddr; /* address of the bulk in endpoint */
	__u8			  bulk_out_endpointAddr; /*address of the bulk out endpoint */
	struct kref   kref;
};

#define to_skel_dev(d) container_of(d, struct usb_skel, kref)

static struct usb_driver skel_driver;

static void skel_delete(struct kref *kref)
{	
	struct usb_skel *dev = to_skel_dev(kref);

	usb_put_dev(dev->udev);
	kfree (dev->bulk_in_buffer);
	kfree (dev);
}

static int skel_open (struct inode *inode, struct file *file) {
	struct usb_skel *dev;
	struct usb_interface *interface;
	int subminor;
	int retval = 0;

	subminor = iminor(inode);

	interface = usb_find_interface(&skel_driver, subminor);
	if(!interface) {
		printk("Can't find device");
		retval = -ENODEV;
		goto exit;
	}

	dev = usb_get_intfdata(interface);
	if(!dev) {
		retval = -ENODEV;
		goto exit;
	}

	kref_get(&dev->kref);

	file->private_data = dev;

	exit:

	return retval;
}

static int skel_release (struct inode *inode, struct file *file) {
	struct usb_skel *dev;

	dev = (struct usb_skel *)file->private_data;
	if (dev == NULL)
		return -ENODEV;

	/* decrement the count on our device */
	kref_put(&dev->kref, skel_delete);
	return 0;
}

static ssize_t skel_read(struct file *file, char __user *buffer, size_t count, loff_t *ppos) {
	return 0;
}

static void skel_write_bulk_callback(struct urb *urb, struct pt_regs *regs)
{
	/* sync/async unlink faults aren't errors */
	if (urb->status && 
	    !(urb->status == -ENOENT || 
	      urb->status == -ECONNRESET ||
	      urb->status == -ESHUTDOWN)) {
		printk(KERN_ALERT "nonzero write bulk status received");
	}

	/* free up our allocated buffer */
	usb_free_coherent(urb->dev, urb->transfer_buffer_length, 
			urb->transfer_buffer, urb->transfer_dma);
}

static ssize_t skel_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos){
	struct usb_skel *dev;
	int retval = 0;
	struct urb *urb = NULL;
	char *buf = NULL;

	dev = (struct usb_skel *)file->private_data;

	/* no data to write, exit */
	if(count == 0)
		goto exit;

	/* create an urb, and a buffer for it, copy the data to the urb */
	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!urb) {
		retval = -ENOMEM;
		goto error;
	}

	buf = usb_alloc_coherent(dev->udev, count, GFP_KERNEL, &urb->transfer_dma);
	if (!buf) {
		retval = -ENOMEM;
		goto error;
	}
	if (copy_from_user(buf, buffer, count)) {
		retval = -EFAULT;
		goto error;
	}

	/* initialize the urb properly */
	usb_fill_bulk_urb(urb, dev->udev,
			  usb_sndbulkpipe(dev->udev, dev->bulk_out_endpointAddr),
			  buf, count, (usb_complete_t)skel_write_bulk_callback, dev);
	urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	/* send the data out the bulk port */
	retval = usb_submit_urb(urb, GFP_KERNEL);
	if (retval) {
		printk("failed submitting write urb, error");
		goto error;
	}

	/* release our reference to this urb, the USB core will eventually free it entirely */
	usb_free_urb(urb);

exit:
	return count;

error:
	usb_free_coherent(dev->udev, count, buf, urb->transfer_dma);
	usb_free_urb(urb);
	kfree(buf);
	return retval;

	return 0;
}

static struct file_operations skel_fops = {
	.owner   = THIS_MODULE,
	.read    = skel_read,
	.write   = skel_write,
	.open    = skel_open,
	.release = skel_release
};

static struct usb_class_driver skel_class = {
	.name = "usb/skel%d",
	.fops = &skel_fops,
	//.mode = S_IFCHR | S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH,
	.minor_base = USB_SKEL_MINOR_BASE
};

static int skel_probe(struct usb_interface *interface, const struct usb_device_id *id) {
	struct usb_skel *dev = NULL;
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint;
	size_t buffer_size;
	int i;
	int retval = -ENOMEM;

	/* allocate memory for device state and initialize it */
	dev = kmalloc(sizeof(struct usb_skel), GFP_KERNEL);
	if(dev == NULL) {
		printk(KERN_ALERT "Out Of Memory");
		goto error;
	}

	memset(dev, 0x00, sizeof(*dev));
	kref_init(&dev->kref);

	dev->udev = usb_get_dev(interface_to_usbdev(interface));
	dev->interface = interface;

	/* Setup endpoint information */
	iface_desc = interface->cur_altsetting;
	for( i = 0; i < iface_desc->desc.bNumEndpoints; i++) {
		endpoint = &iface_desc->endpoint[i].desc;


		if(!dev->bulk_in_endpointAddr &&
			(endpoint->bEndpointAddress & USB_DIR_IN) &&
			((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK)
				== USB_ENDPOINT_XFER_BULK)) {
			/* Bulk In Endpoint */
			buffer_size = endpoint->wMaxPacketSize;
			dev->bulk_in_size = buffer_size;
			dev->bulk_in_endpointAddr = endpoint->bEndpointAddress;
			dev->bulk_in_buffer = kmalloc(buffer_size, GFP_KERNEL);
			if(!dev->bulk_in_buffer){
				printk(KERN_ALERT "Could not allocate bulk_in_buffer");
				goto error;
			}
		}

		if(!dev->bulk_out_endpointAddr &&
			!(endpoint->bEndpointAddress & USB_DIR_IN) &&
			((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK)
				== USB_ENDPOINT_XFER_BULK)) {
			/* Bulk Out Endpoint */
			dev->bulk_out_endpointAddr = endpoint->bEndpointAddress;

		}

	}

	if(!(dev->bulk_in_endpointAddr && dev->bulk_out_endpointAddr)) {
		printk(KERN_ALERT "Could not find both bulk_in and bulk_out endpoints");
		goto error;
	}

	/* save data pointer in inteface device */
	usb_set_intfdata(interface, dev);

	/* register the device now */
	retval = usb_register_dev(interface, &skel_class);
	if(retval) {
		printk(KERN_ALERT "Not able to get a minor for this device");
		usb_set_intfdata(interface, NULL);
		goto error;
	}

	printk(KERN_ALERT "USB skeleton device now attached");
	return 0;

	/* Error */
	error: 
	if(dev)
		kref_put(&dev->kref, skel_delete);
	return retval;
}

static void skel_disconnect(struct usb_interface *interface) {
	struct usb_skel *dev;
	//int minor = interface->minor;

	//lock_kernel();

	dev = usb_get_intfdata(interface);
	usb_set_intfdata(interface, NULL);

	usb_deregister_dev(interface, &skel_class);

	//unlock_kernel();

	kref_put(&dev->kref, skel_delete);

	printk(KERN_ALERT "USB skeleton now disconnected");
}

static struct usb_driver skel_driver = {
	//.owner = THIS_MODULE,
	.name = "skeleton",
	.id_table = skel_table,
	.probe = skel_probe,
	.disconnect = skel_disconnect
};

static int __init usb_skel_init(void) {
	int result;

	/* Register the driver with USB Subsystem */
	result = usb_register(&skel_driver);
	if (result)
		printk(KERN_ALERT "usb_register failed");

	return result;
}

static void __exit usb_skel_exit(void) {
	/* Deregister the driver with the USB Subsystem */
	usb_deregister(&skel_driver);

}

module_init(usb_skel_init);
module_exit(usb_skel_exit);