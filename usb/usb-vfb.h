#ifndef __USB_VFB_H__
#define __USB_VFB_H__

#define WIDTH			320
#define HEIGHT			240
#define BYTE_DEPTH		2
#define FB_SIZE			WIDTH*HEIGHT*BYTE_DEPTH
#define FP_PAGE_COUNT		PAGE_ALIGN(FB_SIZE)/PAGE_SIZE
#define PIXELS_IN_PAGE		PAGE_SIZE/BYTE_DEPTH

struct videopage
{
        int                             x, y;
        unsigned char           *mem;
        unsigned long            length;
        atomic_t                     toUpdate;
};

/* Structure to hold all of our device specific stuff */
struct usb_skel {
	struct usb_device	*udev;			/* the usb device for this device */
	struct usb_interface	*interface;		/* the interface for this device */
	struct semaphore	limit_sem;		/* limiting the number of writes in progress */
	struct usb_anchor	submitted;		/* in case we need to retract our submissions */
	struct urb		*bulk_in_urb;		/* the urb to read data with */
	unsigned char           *bulk_in_buffer;	/* the buffer to receive data */
	size_t			bulk_in_size;		/* the size of the receive buffer */
	size_t			bulk_in_filled;		/* number of bytes in the buffer */
	size_t			bulk_in_copied;		/* already copied to user space */
	__u8			bulk_in_endpointAddr;	/* the address of the bulk in endpoint */
	__u8			bulk_out_endpointAddr;	/* the address of the bulk out endpoint */
	int			errors;			/* the last request tanked */
	int			open_count;		/* count the number of openers */
	bool			ongoing_read;		/* a read is going on */
	spinlock_t		err_lock;		/* lock for errors */
	struct kref		kref;
	struct mutex		io_mutex;		/* synchronize I/O with disconnect */
	wait_queue_head_t	bulk_in_wait;		/* to wait for an ongoing read */

    struct device                                           *gdev;      
    struct fb_info                                          *info;

    struct usb_endpoint_descriptor *bulk_out_ep;
    unsigned int bulk_out_packet_size;
	struct videopage videopages[FP_PAGE_COUNT];
    unsigned long pseudo_palette[17];
};
void display_update(struct fb_info *info, struct list_head *pagelist);
int vfb_usb_probe(struct usb_skel *dev);
#endif
