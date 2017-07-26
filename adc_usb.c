/*
 * adc_usb.c
 *
 *  Created on: 7 июн. 2017 г.
 *      Author: alex2
 */

#include "adc_usb.h"

static DEFINE_MUTEX(disconnect_mutex); //Иницилизация mutex для функции disconect

static struct usb_device_id adc_table[] = //Таблица устройств на которые будет работать драйвер
{ { USB_DEVICE(VENDOOR_ID, PRODUTCT_ID) }, { } /* Terminating entry */
};

MODULE_DEVICE_TABLE(usb, adc_table); // иницилизация таблицы устройств

static struct file_operations adc_fops = // структура для функций для работы с драйвером
{
		.owner          = THIS_MODULE,
		.write          = adc_write,      // функция записи
		.read           = adc_read,       // функция чтения
		.open           = adc_open,       // функция открытия
		.release        = adc_release,    // функция закрытия
		.unlocked_ioctl = adc_unlocked_ioctl,
		//.mmap           = adc_mmap,
		.poll           = adc_poll,

};



static struct usb_class_driver adc_class = { // структура данных для драйвера
		.name = "adc-rls1", // название устойсва в /dev/
		.fops = &adc_fops,  // запись данных о фукциях для работы

};


static long adc_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)// функция управления вводом выводом
{
	unsigned long tbs = ramdisk_size; // размер виртуального диска
	void __user *ioargp = (void __user *)arg; //переменная для пердаыи в userspace
	int retval =0;
	struct adc_device_struct *dev = NULL;
	dev = file->private_data;

	switch (cmd) {// команда от user
	default:
		return -EINVAL;

	case MMAP_DEV_CMD_GET_BUFSIZE: // команда о размере буфера
		if (copy_to_user(ioargp, &tbs, sizeof(tbs)))// отправка размера буфера
			return -EFAULT;
		return 0;
	break;
	case DEV_CMD_SET_START:
		retval = usb_submit_urb(dev->iso_urb, GFP_KERNEL);

			if (retval) {
				DBG_ERR("submitting int urb failed (%d)", retval);
				dev->int_running = 0;
				return retval;
			}
			DBG_INFO("device start");
		break;
		
	}
	return retval;
}


static unsigned int adc_poll(struct file *file, poll_table * wait){
	poll_wait(file,&wq,wait);
	//DBG_INFO("входящий poll  через %ld тактов ", jiffies);
	if(atomic_read(&data_ready)){
		return POLLIN|POLLRDNORM;
	}
	return 0;
}




static ssize_t adc_write(struct file *file, const char __user *buf, size_t cnt,
		loff_t *off) {

	return 0;
}


static ssize_t adc_read(struct file *file, char __user *buf, size_t cnt,loff_t *off) {
	struct adc_device_struct *dev = NULL;

	int retval;
	int nbytes =0, maxbytes, bytes_to_do;

/*	if (down_interruptible(&dev->sem)) {
	 retval = -ERESTARTSYS;
	 goto exit;
	}*/

	dev = file->private_data;

	if(!dev){
		DBG_ERR("dev NULL");
		return -1;
	}


	wait_event_interruptible(wq, (atomic_read(&data_ready)));

	if (down_interruptible(&dev->sem)) {
			retval = -ERESTARTSYS;
			goto exit;
		}



		maxbytes = ramdisk_size - *off;
		bytes_to_do = maxbytes > cnt ? cnt : maxbytes;
		if (bytes_to_do == 0)
			pr_warning("Reached end of the device on a read");



		nbytes =copy_to_user(buf, user_buffer + *off, bytes_to_do);
		//*off += nbytes;
		//up(&dev->sem);

		//dev->count =dev->count - ((int)(cnt/1023));
		//pr_info("\n Leaving the   READ function, nbytes=%d, pos=%d\n",
			//		nbytes, dev->count);
		memset(dev->iso_buffer,0,1023);
		atomic_set(&data_ready,0);
		exit:

		up(&dev->sem);

		return nbytes;
}

static int adc_open(struct inode* inode, struct file* file) {

	struct adc_device_struct *dev = NULL;
	struct usb_interface *interface;
	int subminor;
	int retval = 0;


	DBG_INFO("Open device");

	subminor = iminor(inode);
	mutex_lock(&disconnect_mutex);

	interface = usb_find_interface(&adc_driver, subminor);

	if (!interface) {
		DBG_ERR("can't find device for minor %d", subminor);
		retval = -ENODEV;
		goto exit;
	}

	dev = usb_get_intfdata(interface);

	if (!dev) {
		retval = -ENODEV;
		goto exit;
	}

	if (down_interruptible(&dev->sem)) {
		retval = -ERESTARTSYS;
		goto exit;
	}

	dev->count = 0;

	dev->iso_urb->dev                      = dev->udev;
	dev->iso_urb->context                  = dev;
	dev->iso_urb->pipe                     = usb_rcvisocpipe(dev->udev,dev->iso_in_endpoint->bEndpointAddress);
	dev->iso_urb->interval                 = dev->iso_in_endpoint->bInterval;
	dev->iso_urb->complete                 = adc_iso_callbak;
	dev->iso_urb->transfer_flags           = URB_ISO_ASAP;
	dev->iso_urb->transfer_buffer          = dev->iso_buffer;
	dev->iso_urb->number_of_packets        = ISO_PAKETS;
	dev->iso_urb->transfer_buffer_length   = dev->iso_in_endpoint->wMaxPacketSize;
	dev->iso_urb->iso_frame_desc[0].length = dev->iso_in_endpoint->wMaxPacketSize;
	dev->iso_urb->iso_frame_desc[0].offset = 0;

	//retval = usb_submit_urb(dev->iso_urb, GFP_KERNEL);

	/*if (retval) {
		DBG_ERR("submitting int urb failed (%d)", retval);
		dev->int_running = 0;
		goto unlock_exit;
	}*/

	dev->int_running = 1;
	dev->openned     = 1;

	file->private_data = dev;

	mb();

	DBG_INFO("Device opened");



	//unlock_exit: up(&dev->sem);

	exit: mutex_unlock(&disconnect_mutex);
	return retval;
}

static int adc_release(struct inode* node, struct file* file) {

	struct adc_device_struct *dev = NULL;
	int retval = 0;

	DBG_INFO("Release driver");

	dev = file->private_data;

	/*	if (down_interruptible(&dev->sem)) {
	 retval = -ERESTARTSYS;
	 goto exit;
	 }*/

	if (!dev) {
		DBG_ERR("dev is NULL");
		retval = -ENODEV;
		goto exit;
	}

	if (dev->openned <= 0) {
		DBG_ERR("device not opened");
		retval = -ENODEV;
		goto exit;
	}

	if (!dev->udev) {
		DBG_DEBUG("device unplugged before the file was released");
		adc_delete(dev);
		goto exit;
	}

	adc_abort_transfers(dev);
	dev->int_running = 0;
	dev->openned = 0;



	exit: 
	return retval;
}

/*static void adc_int_callback(struct urb *urb){

 struct adc_device_struct *dev = urb->context;




 usb_submit_urb(dev->int_urb,GFP_ATOMIC);
 //printk("enter");

 }*/

static void adc_iso_callbak(struct urb *urb) {
	int retval;

	static int zero_transfer = 0;

	struct adc_device_struct *dev = urb->context;


    memcpy(user_buffer,dev->iso_buffer,1023);


	if (urb->status ==0) {
		if (urb->status == -ENOENT || urb->status == -ECONNRESET
				|| urb->status == -ESHUTDOWN) {
			DBG_ERR("non-zero urb status return");
			return;
		}
	} else {
		DBG_ERR("non-zero urb status (%d)", urb->status);
	}

	if (urb->actual_length == 0) {
		zero_transfer++;
		DBG_ERR("Zero transfer (%d)", zero_transfer);
	}


	if (dev->int_running && dev->udev) {

		retval = usb_submit_urb(dev->iso_urb, GFP_ATOMIC);

		if (retval) {
			DBG_ERR("submitting correction control URB failed (%d)", retval);
			dev->int_running = 0;

		}

	}


	wake_up_interruptible(&wq);
	atomic_set(&data_ready, 1);


	memset(dev->iso_buffer,0,1023);
}

static int adc_probe(struct usb_interface *interface,
		const struct usb_device_id *id) {

	__u8 i;
	int iso_end_size;
	struct usb_device *udev = interface_to_usbdev(interface);
	struct adc_device_struct *dev = NULL;
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint;

	int retval = -ENODEV;

	printk(
			KERN_INFO "Производитель: АО 'Изумруд' АПЦ-001: адресс %04X:%04X подключено\n",
			id->idVendor, id->idProduct);

	dev = kzalloc(sizeof(struct adc_device_struct), GFP_KERNEL);

	//sema_init(&dev->sem, 1);

	if (!dev) {
		DBG_ERR("cannot allocate memory for struct adc_device");
		retval = -ENOMEM;
		goto error;
	}

	dev->udev = udev;
	//dev->interface = interface;
	iface_desc = interface->cur_altsetting;

	for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {

		endpoint = &iface_desc->endpoint[i].desc;

		if ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK)
				== USB_ENDPOINT_XFER_ISOC) {
			dev->iso_in_endpoint = endpoint;
		}

		if ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK)
				== USB_ENDPOINT_XFER_INT) {
			dev->int_in_endpoint = endpoint;
		}

	}
	if (!dev->iso_in_endpoint) {
		DBG_ERR("could not find iso in endpoint");
		goto error;
	}

	sema_init(&dev->sem, 1);

	iso_end_size = le16_to_cpu(dev->iso_in_endpoint->wMaxPacketSize);

	dev->iso_buffer = kmalloc(1023, GFP_KERNEL);
	user_buffer = kmalloc(1023, GFP_KERNEL);

	if (!dev->iso_buffer) {
		DBG_ERR("could not allocate iso_buffer");
		retval = -ENOMEM;
		goto error;
	}

	dev->count = 0;

	dev->iso_urb = usb_alloc_urb(ISO_PAKETS, GFP_ATOMIC);

	if (!dev->iso_urb) {
		DBG_ERR("could not allocate int_in_urb");
		retval = -ENOMEM;
		goto error;
	}



	usb_set_intfdata(interface, dev);

	retval = usb_register_dev(interface, &adc_class);

	if (retval) {
		DBG_ERR("not able to get a minor for this device.");
		usb_set_intfdata(interface, NULL);
		goto error;
	}

	dev->minor = interface->minor;
	dev->iso_buf_len = 1023;

	DBG_INFO("USB ADC now attached to /dev/adc_rls1.0%d",
			interface->minor - ML_MINOR_BASE);



	return retval;

	error: adc_delete(dev);

	return retval;
}
static inline void adc_delete(struct adc_device_struct *dev) {

	adc_abort_transfers(dev);
	if (dev->iso_urb) {
		usb_free_urb(dev->iso_urb);
	}
	kfree(dev->iso_buffer);
	kfree(dev);

}

static void adc_abort_transfers(struct adc_device_struct *dev) {

	if (!dev) {
		DBG_ERR("udev is NULL");
		return;
	}

	if (!dev->udev) {
		DBG_ERR("udev is NULL");
		return;
	}
	if (dev->udev->state == USB_STATE_NOTATTACHED) {
		DBG_ERR("udev not attached");
		return;
	}

	if (dev->int_running) {
		dev->int_running = 0;
		mb();
		if (dev->iso_urb) {
			usb_kill_urb(dev->iso_urb);
		}

	}

}

static void adc_disconnect(struct usb_interface *interface) {

	struct adc_device_struct *dev;
	int minor;

	mutex_lock(&disconnect_mutex);

	dev = usb_get_intfdata(interface);
	usb_set_intfdata(interface, NULL);

	minor = dev->minor;

	usb_deregister_dev(interface, &adc_class);

	dev->udev = NULL;
	//up(&dev->sem);

	mutex_unlock(&disconnect_mutex);

	printk(KERN_INFO "АПЦ-001  отключено\n");

}


static struct usb_driver adc_driver = {

		.name        = "adc_driver",
		.id_table    = adc_table,
		.probe       = adc_probe,
		.disconnect  = adc_disconnect,

};


static int __init adc_init(void) {
	int result;

	result = usb_register(&adc_driver);
	if (result < 0) {
		printk(KERN_ERR "Ошибка иницилизации АЦП-001" __FILE__ "драйвер");
		return -1;
	}
	//ramdisk = kmalloc(ramdisk_size, GFP_KERNEL);
	atomic_set(&data_ready, 0);


	return result;

}

static void __exit adc_exit(void) {

	usb_deregister(&adc_driver);

	printk(KERN_INFO "АПЦ-001  deinit\n");
	//kfree(ramdisk);




}

module_init(adc_init);
module_exit(adc_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Aлексей Чиликин");
MODULE_DESCRIPTION("Драйвер АЦП-001");

