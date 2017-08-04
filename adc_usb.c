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



	void __user *ioargp = (void __user *)arg; //переменная для передачи в userspace
	int retval =0;
	struct adc_device_struct *dev = NULL;
	dev = file->private_data;




	switch (cmd) {// команда от user
	case DEV_OFFSET_DATA:
		retval = copy_to_user(ioargp,&offset_data,sizeof(offset_data));
	break;

	case DEV_CMD_SET_START:
		retval = usb_submit_urb(dev->iso_razvertka_urb, GFP_KERNEL);

		if (retval) {
						DBG_ERR("submitting int urb failed (%d)", retval);
						dev->int_running = 0;
						return retval;
		}

		retval = usb_submit_urb(dev->iso_tochno_urb, GFP_KERNEL);

			if (retval) {
				DBG_ERR("submitting int urb failed (%d)", retval);
				dev->int_running = 0;
				return retval;
			}
			dev->int_running = 1;
			DBG_INFO("device start");
		break;
	case DEV_CMD_SET_STOP:
		adc_abort_transfers(dev);
		DBG_INFO("device stop");
		dev->int_running = 0;

		break;
	default:
			return -EINVAL;
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
};




static ssize_t adc_write(struct file *file, const char __user *buf, size_t cnt,
		loff_t *off) {

	return 0;
}


static ssize_t adc_read(struct file *file, char __user *buf, size_t cnt,loff_t *off) {

	struct adc_device_struct *dev = NULL;


	int nbytes =0;

	dev = file->private_data;

	if(!dev){
		DBG_ERR("dev NULL");
		return -1;
	}


	wait_event_interruptible(wq, (atomic_read(&data_ready)));

	if (down_interruptible(&dev->sem)) {
		up(&dev->sem);
		return  -ERESTARTSYS;

	}

		nbytes =copy_to_user(buf, user_buffer + *off, cnt);

		atomic_set(&data_ready,0);

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
		mutex_unlock(&disconnect_mutex);
		return retval;
	}

	dev = usb_get_intfdata(interface);


	if (!dev) {
		retval = -ENODEV;
		mutex_unlock(&disconnect_mutex);
		return retval;
	}


	if (down_interruptible(&dev->sem)) {
		retval = -ERESTARTSYS;
		mutex_unlock(&disconnect_mutex);
		return retval;
	}


	dev->count = 0;

	dev->iso_razvertka_urb->dev                      = dev->udev;
	dev->iso_razvertka_urb->context                  = dev;
	dev->iso_razvertka_urb->pipe                     = usb_rcvisocpipe(dev->udev,dev->iso_razvertka_endpoint->bEndpointAddress);
	dev->iso_razvertka_urb->interval                 = dev->iso_razvertka_endpoint->bInterval;
	dev->iso_razvertka_urb->complete                 = adc_iso_razvertka_callbak;
	dev->iso_razvertka_urb->transfer_flags           = URB_ISO_ASAP;
	dev->iso_razvertka_urb->transfer_buffer          = dev->iso_razvertka_buffer;
	dev->iso_razvertka_urb->number_of_packets        = ISO_PAKETS;
	dev->iso_razvertka_urb->transfer_buffer_length   = dev->iso_razvertka_endpoint->wMaxPacketSize;
	dev->iso_razvertka_urb->iso_frame_desc[0].length = dev->iso_razvertka_endpoint->wMaxPacketSize;
	dev->iso_razvertka_urb->iso_frame_desc[0].offset = 0;

	dev->iso_tochno_urb->dev                      = dev->udev;
	dev->iso_tochno_urb->context                  = dev;
	dev->iso_tochno_urb->pipe                     = usb_rcvisocpipe(dev->udev,dev->iso_tochno_endpoint->bEndpointAddress);
	dev->iso_tochno_urb->interval                 = dev->iso_tochno_endpoint->bInterval;
	dev->iso_tochno_urb->complete                 = adc_iso_tochno_callbak;
	dev->iso_tochno_urb->transfer_flags           = URB_ISO_ASAP;
	dev->iso_tochno_urb->transfer_buffer          = dev->iso_tochno_buffer;
	dev->iso_tochno_urb->number_of_packets        = ISO_PAKETS;
	dev->iso_tochno_urb->transfer_buffer_length   = dev->iso_tochno_endpoint->wMaxPacketSize;
	dev->iso_tochno_urb->iso_frame_desc[0].length = dev->iso_tochno_endpoint->wMaxPacketSize;
	dev->iso_tochno_urb->iso_frame_desc[0].offset = 0;

	dev->openned = 1;

	file->private_data = dev;

	mb();
	mutex_unlock(&disconnect_mutex);
	up(&dev->sem);

	DBG_INFO("Device opened");

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

static void adc_iso_razvertka_callbak(struct urb *urb) {
	int retval;

	static int zero_transfer = 0;

	struct adc_device_struct *dev = urb->context;


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

	 memcpy(user_buffer+offset_razvertka,dev->iso_razvertka_buffer,dev->iso_razvertka_buf_len);


	if (dev->int_running && dev->udev) {

		retval = usb_submit_urb(dev->iso_razvertka_urb, GFP_ATOMIC);

		if (retval) {
			DBG_ERR("submitting correction control URB failed (%d), развертка", retval);
			dev->int_running = 0;

		}

	}

	offset_data = offset_razvertka;

	wake_up_interruptible(&wq);
	atomic_set(&data_ready, 1);


	memset(dev->iso_razvertka_buffer,0,dev->iso_razvertka_buf_len);
}

static void adc_iso_tochno_callbak(struct urb *urb) {
	int retval;

	static int zero_transfer = 0;
	struct adc_device_struct *dev = urb->context;



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
	memcpy(user_buffer+offset_tochno,dev->iso_tochno_buffer,dev->iso_tocho_buf_len);


	if (dev->int_running && dev->udev) {

		retval = usb_submit_urb(dev->iso_tochno_urb, GFP_ATOMIC);

		if (retval) {
			DBG_ERR("submitting correction control URB failed (%d), endpoint  точная дальность", retval);
			dev->int_running = 0;

		}

	}

	offset_data = offset_tochno;

	wake_up_interruptible(&wq);
	atomic_set(&data_ready, 1);


	memset(dev->iso_tochno_buffer,0,dev->iso_tocho_buf_len);
}

static int adc_probe(struct usb_interface *interface,
		const struct usb_device_id *id) {

	__u8 i;
	int iso_end_size_razvertka;
	int iso_end_size_tochno;
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
		switch(i){
		case(razvertka):
				dev->iso_razvertka_endpoint = endpoint;
		break;
		case(dalnost_tochno):
				dev->iso_tochno_endpoint= endpoint;
		break;
		}


	}
	if (!dev->iso_razvertka_endpoint) {
		DBG_ERR("конечная точка с разверткой не найдена");
		goto error;
	}
	if (!dev->iso_tochno_endpoint) {
			DBG_ERR("конечная точка с точной дальностью не найдена");
			goto error;
	}



	sema_init(&dev->sem, 1);

	iso_end_size_razvertka = le16_to_cpu(dev->iso_razvertka_endpoint->wMaxPacketSize);
	iso_end_size_tochno    = le16_to_cpu(dev->iso_tochno_endpoint->wMaxPacketSize);

	dev->iso_razvertka_buffer  = kmalloc(iso_end_size_razvertka, GFP_KERNEL);// выделение памяти под буффер usb развертки
	dev->iso_tochno_buffer     = kmalloc(iso_end_size_tochno, GFP_KERNEL);   // выделение памяти под буффер usb точной дальности
	user_buffer                = kmalloc(2048, GFP_KERNEL);                  // выделение памяти под буффер для передачи в userspace

	if (!dev->iso_razvertka_buffer) {
		DBG_ERR("could not allocate iso_razvertka_buffer");
		retval = -ENOMEM;
		goto error;
	}

	if (!dev->iso_tochno_endpoint) {
			DBG_ERR("could not allocate iso_tochno_buffer");
			retval = -ENOMEM;
			goto error;
	}


	dev->count = 0;

	dev->iso_razvertka_urb = usb_alloc_urb(ISO_PAKETS, GFP_ATOMIC);
	dev->iso_tochno_urb     = usb_alloc_urb(ISO_PAKETS, GFP_ATOMIC);

	if (!dev->iso_razvertka_urb) {
		DBG_ERR("could not allocate iso_razvertka_urb");
		retval = -ENOMEM;
		goto error;
	}

	if (!dev->iso_tochno_urb) {
			DBG_ERR("could not allocate iso_tocho_urb");
			retval = -ENOMEM;
			goto error;
	}
	    dev->minor = interface->minor;
		dev->iso_razvertka_buf_len = iso_end_size_razvertka;
		dev->iso_tocho_buf_len     = iso_end_size_tochno;



	usb_set_intfdata(interface, dev);

	retval = usb_register_dev(interface, &adc_class);

	if (retval) {
		DBG_ERR("not able to get a minor for this device.");
		usb_set_intfdata(interface, NULL);
		goto error;
	}



	DBG_INFO("USB ADC now attached to /dev/adc_rls1.0%d",
			interface->minor - ML_MINOR_BASE);

	return retval;

	error: adc_delete(dev);

	return retval;
}
static inline void adc_delete(struct adc_device_struct *dev) {

	adc_abort_transfers(dev);
	if (dev->iso_razvertka_urb) {
		usb_free_urb(dev->iso_razvertka_urb);
	}

	if (dev->iso_tochno_urb) {
			usb_free_urb(dev->iso_tochno_urb);
		}
	kfree(dev->iso_razvertka_buffer);
	kfree(dev->iso_tochno_buffer);
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
		if (dev->iso_razvertka_urb) {
			usb_kill_urb(dev->iso_razvertka_urb);
		}

		if (dev->iso_tochno_urb) {
			usb_kill_urb(dev->iso_tochno_urb);
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

