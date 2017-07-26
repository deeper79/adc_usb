/*
 * adc_usb.h
 *
 *  Created on: 14 июн. 2017 г.
 *      Author: alex2
 */
#ifndef _ADC_USB_
#define _ADC_USB_

#include <linux/module.h>
#include <linux/init.h>

#include <linux/slab.h>			/* kmalloc() */
#include <linux/usb.h>			/* USB stuff */
#include <linux/mutex.h>		/* mutexes */
#include <linux/ioctl.h>

#include <asm/uaccess.h>	/* copy_*_user */
#include <linux/sched.h>
#include <linux/pid.h>
#include <linux/string.h>
#include <linux/pagemap.h>
#include <linux/mm.h>
#include <linux/wait.h> /* wait_queue_head_t, wait_event_interruptible, wake_up_interruptible  */
#include <uapi/linux/stat.h> /* S_IRUSR */
#include <linux/delay.h> /* usleep_range */
#include <linux/kthread.h>
#include <linux/poll.h>
#include <asm/pgtable.h>
#include <linux/errno.h>	/* error codes */
#include <linux/ioctl.h>
#include <linux/cdev.h>


#define VENDOOR_ID  0x0000
#define PRODUTCT_ID 0x0001

#ifdef CONFIG_USB_DYNAMIC_MINORS
#define ML_MINOR_BASE	0
#else
#define ML_MINOR_BASE	96
#endif
#define DEBUG_LEVEL_DEBUG		0x1F
#define DEBUG_LEVEL_INFO		0x0F
#define DEBUG_LEVEL_WARN		0x07
#define DEBUG_LEVEL_ERROR		0x03
#define DEBUG_LEVEL_CRITICAL	0x01
#define ISO_PAKETS              0x01

#define MMAP_DEV_CMD_GET_BUFSIZE 1	/* defines our IOCTL cmd */
#define DEV_CMD_SET_START   2	/* defines our IOCTL cmd */
#define DEV_CMD_SET_STOP    3	/* defines our IOCTL cmd */

static DECLARE_WAIT_QUEUE_HEAD(wq);

static int debug_level = DEBUG_LEVEL_DEBUG;

#define DBG_DEBUG(fmt, args...) \
if ((debug_level & DEBUG_LEVEL_DEBUG) == DEBUG_LEVEL_DEBUG) \
	printk( KERN_DEBUG "[debug] %s(%d): " fmt "\n", \
			__FUNCTION__, __LINE__, ## args)
#define DBG_INFO(fmt, args...) \
if ((debug_level & DEBUG_LEVEL_INFO) == DEBUG_LEVEL_INFO) \
	printk( KERN_DEBUG "[info]  %s(%d): " fmt "\n", \
			__FUNCTION__, __LINE__, ## args)
#define DBG_WARN(fmt, args...) \
if ((debug_level & DEBUG_LEVEL_WARN) == DEBUG_LEVEL_WARN) \
	printk( KERN_DEBUG "[warn]  %s(%d): " fmt "\n", \
			__FUNCTION__, __LINE__, ## args)
#define DBG_ERR(fmt, args...) \
if ((debug_level & DEBUG_LEVEL_ERROR) == DEBUG_LEVEL_ERROR) \
	printk( KERN_DEBUG "[err]   %s(%d): " fmt "\n", \
			__FUNCTION__, __LINE__, ## args)
#define DBG_CRIT(fmt, args...) \
if ((debug_level & DEBUG_LEVEL_CRITICAL) == DEBUG_LEVEL_CRITICAL) \
	printk( KERN_DEBUG "[crit]  %s(%d): " fmt "\n", \
			__FUNCTION__, __LINE__, ## args)


#define LED_MAJOR 42

static struct usb_driver adc_driver;

static atomic_t data_ready;



static size_t ramdisk_size = (16*PAGE_SIZE);





struct adc_device_struct{
	struct usb_device              *udev;
	struct usb_intreface           *interface;
	unsigned char	minor;

    struct usb_endpoint_descriptor *int_in_endpoint;
    struct urb                     *int_urb;
    char                           *int_buffer;

    struct usb_endpoint_descriptor *iso_in_endpoint;
    struct urb                     *iso_urb;
    char                           *iso_buffer;
    size_t                          iso_buf_len;

    __u8                            int_running;
    __u8                            openned;
    __u8                            count;
    struct semaphore                sem; /* Locks this structure */


    int vmas; /* active mappings */
    int order;                /* the current allocation order */
    int qset; /* the current array size */

    struct cdev cdev;

};

static char* user_buffer;




static int  adc_probe(struct usb_interface *inter, const struct usb_device_id *id);
static void adc_iso_callbak(struct urb *urb);

//static void adc_int_callback(struct urb *urb);

static int  adc_release(struct inode* node, struct file* file);
static int  adc_open(struct inode* node, struct file* file);

static ssize_t      adc_write(struct file *file, const char __user *buf, size_t cnt,loff_t *off);
static ssize_t      adc_read(struct file *file, char __user *buf, size_t cnt, loff_t *off);
static inline void  adc_delete(struct adc_device_struct *dev);
static void         adc_abort_transfers(struct adc_device_struct *dev);
//static int adc_mmap(struct file *file,struct vm_area_struct *vma);


static long adc_unlocked_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
static unsigned int adc_poll(struct file *file, poll_table * wait);







#endif
