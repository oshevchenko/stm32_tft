/*
 *  linux/drivers/video/vfb.c -- Virtual frame buffer device
 *
 *      Copyright (C) 2002 James Simmons
 *
 *	Copyright (C) 1997 Geert Uytterhoeven
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>

#include <linux/fb.h>
#include <linux/init.h>

#include <linux/slab.h>
#include <linux/kref.h>
#include <linux/uaccess.h>
#include <linux/usb.h>
#include <linux/mutex.h>

#include "usb-vfb.h"



static void display_fillrect(struct fb_info *p, const struct fb_fillrect *rect);
static void display_imageblit(struct fb_info *p, const struct fb_image *image);
static void display_copyarea(struct fb_info *p, const struct fb_copyarea *area);
static ssize_t display_write(struct fb_info *p, const char __user *buf, 
                                size_t count, loff_t *ppos); 
static int display_setcolreg(unsigned regno,
                               unsigned red, unsigned green, unsigned blue,
                               unsigned transp, struct fb_info *info);
//---------------
//static void display_update(struct fb_info *info, struct list_head *pagelist);
static struct fb_ops display_fbops = {
        .owner        = THIS_MODULE,
        .fb_read      = fb_sys_read,
        .fb_write     = display_write,
        .fb_fillrect  = display_fillrect,
        .fb_copyarea  = display_copyarea,
        .fb_imageblit = display_imageblit,
        .fb_setcolreg   = display_setcolreg,
};

static struct fb_fix_screeninfo fixed_info =
{
        .id = "STM32LCD",
        .type        = FB_TYPE_PACKED_PIXELS,
        .visual      = FB_VISUAL_TRUECOLOR,
        .accel       = FB_ACCEL_NONE,
        .line_length = WIDTH * BYTE_DEPTH,
};

static struct fb_var_screeninfo var_info =
{
        .xres                   =       WIDTH,
        .yres                   =       HEIGHT,
        .xres_virtual   =       WIDTH,
        .yres_virtual   =       HEIGHT,
        .width                  =       WIDTH,
        .height                 =       HEIGHT,
        .bits_per_pixel =       16,
        .red                    =       {11, 5, 0},
        .green                  =       {5, 6, 0},
        .blue                   =       {0, 5, 0},
        .activate               =       FB_ACTIVATE_NOW,
        .vmode                  =       FB_VMODE_NONINTERLACED,
};

static struct fb_deferred_io display_defio = {
        .delay          = HZ/6,
        .deferred_io    = &display_update,
};

static void display_touch(struct fb_info *info, int x, int y, int w, int h) 
{
        int firstPage;
        int lastPage;
        int i;

        struct usb_skel *dev=info->par;

        firstPage=((y*WIDTH)+x)*BYTE_DEPTH/PAGE_SIZE-1;
        lastPage=(((y+h)*WIDTH)+x+w)*BYTE_DEPTH/PAGE_SIZE+1;
        if(firstPage<0)
                firstPage=0;
        if(lastPage>FP_PAGE_COUNT)
                lastPage=FP_PAGE_COUNT;
        for(i=firstPage;i<lastPage;i++)
                atomic_dec(&dev->videopages[i].toUpdate);

        schedule_delayed_work(&info->deferred_work, info->fbdefio->delay);
}

static void display_fillrect(struct fb_info *p, const struct fb_fillrect *rect)
{
        sys_fillrect(p, rect);
        display_touch(p, rect->dx, rect->dy, rect->width, rect->height);
}

static void display_imageblit(struct fb_info *p, const struct fb_image *image)
{
        sys_imageblit(p, image);
        display_touch(p, image->dx, image->dy, image->width, image->height);
}

static void display_copyarea(struct fb_info *p, const struct fb_copyarea *area)
{
        sys_copyarea(p, area);
        display_touch(p, area->dx, area->dy, area->width, area->height);
}

static ssize_t display_write(struct fb_info *p, const char __user *buf, 
                                size_t count, loff_t *ppos)
{       
        int retval;
        retval=fb_sys_write(p, buf, count, ppos);
        display_touch(p, 0, 0, p->var.xres, p->var.yres);
        return retval;
}

    /*
     *  RAM we reserve for the frame buffer. This defines the maximum screen
     *  size
     *
     *  The default can be overridden if the driver is compiled as a module
     */

#define VIDEOMEMSIZE	(1*1024*1024)	/* 1 MB */

static void *videomemory;
static u_long videomemorysize = VIDEOMEMSIZE;
module_param(videomemorysize, ulong, 0);

/**********************************************************************
 *
 * Memory management
 *
 **********************************************************************/
static void *rvmalloc(unsigned long size)
{
	void *mem;
	unsigned long adr;

	size = PAGE_ALIGN(size);
	mem = vmalloc_32(size);
	if (!mem)
		return NULL;

	/*
	 * VFB must clear memory to prevent kernel info
	 * leakage into userspace
	 * VGA-based drivers MUST NOT clear memory if
	 * they want to be able to take over vgacon
	 */

	memset(mem, 0, size);
	adr = (unsigned long) mem;
	while (size > 0) {
		SetPageReserved(vmalloc_to_page((void *)adr));
		adr += PAGE_SIZE;
		size -= PAGE_SIZE;
	}

	return mem;
}
#if 0
static void rvfree(void *mem, unsigned long size)
{
	unsigned long adr;

	if (!mem)
		return;

	adr = (unsigned long) mem;
	while ((long) size > 0) {
		ClearPageReserved(vmalloc_to_page((void *)adr));
		adr += PAGE_SIZE;
		size -= PAGE_SIZE;
	}
	vfree(mem);
}

static struct fb_var_screeninfo vfb_default = {
	.xres =		320,
	.yres =		240,
	.xres_virtual =	320,
	.yres_virtual =	240,
	.bits_per_pixel = 16,
	.red =		{ 0, 5, 0 },
      	.green =	{ 5, 6, 0 },
      	.blue =		{ 11, 5, 0 },
      	.activate =	FB_ACTIVATE_TEST,
      	.height =	-1,
      	.width =	-1,
      	.pixclock =	20000,
      	.left_margin =	64,
      	.right_margin =	64,
      	.upper_margin =	32,
      	.lower_margin =	32,
      	.hsync_len =	64,
      	.vsync_len =	2,
      	.vmode =	FB_VMODE_NONINTERLACED,
};

static struct fb_fix_screeninfo vfb_fix = {
	.id =		"Virtual FB",
	.type =		FB_TYPE_PACKED_PIXELS,
	.visual =	FB_VISUAL_PSEUDOCOLOR,
	.xpanstep =	1,
	.ypanstep =	1,
	.ywrapstep =	1,
	.accel =	FB_ACCEL_NONE,
};

static bool vfb_enable __initdata = 0;	/* disabled by default */
module_param(vfb_enable, bool, 0);

static int vfb_check_var(struct fb_var_screeninfo *var,
			 struct fb_info *info);
static int vfb_set_par(struct fb_info *info);
static int vfb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
			 u_int transp, struct fb_info *info);
static int vfb_pan_display(struct fb_var_screeninfo *var,
			   struct fb_info *info);
static int vfb_mmap(struct fb_info *info,
		    struct vm_area_struct *vma);

static struct fb_ops vfb_ops = {
	.fb_read        = fb_sys_read,
	.fb_write       = display_write,
	.fb_check_var	= vfb_check_var,
	.fb_set_par	= vfb_set_par,
	.fb_setcolreg	= vfb_setcolreg,
	.fb_pan_display	= vfb_pan_display,
	.fb_fillrect	= display_fillrect,
	.fb_copyarea	= display_copyarea,
	.fb_imageblit	= display_imageblit,
	.fb_mmap	= vfb_mmap,
};

    /*
     *  Internal routines
     */

static u_long get_line_length(int xres_virtual, int bpp)
{
	u_long length;

	length = xres_virtual * bpp;
	length = (length + 31) & ~31;
	length >>= 3;
	return (length);
}

    /*
     *  Setting the video mode has been split into two parts.
     *  First part, xxxfb_check_var, must not write anything
     *  to hardware, it should only verify and adjust var.
     *  This means it doesn't alter par but it does use hardware
     *  data from it to check this var. 
     */

static int vfb_check_var(struct fb_var_screeninfo *var,
			 struct fb_info *info)
{
	u_long line_length;

	/*
	 *  FB_VMODE_CONUPDATE and FB_VMODE_SMOOTH_XPAN are equal!
	 *  as FB_VMODE_SMOOTH_XPAN is only used internally
	 */

	if (var->vmode & FB_VMODE_CONUPDATE) {
		var->vmode |= FB_VMODE_YWRAP;
		var->xoffset = info->var.xoffset;
		var->yoffset = info->var.yoffset;
	}

	/*
	 *  Some very basic checks
	 */
	if (!var->xres)
		var->xres = 1;
	if (!var->yres)
		var->yres = 1;
	if (var->xres > var->xres_virtual)
		var->xres_virtual = var->xres;
	if (var->yres > var->yres_virtual)
		var->yres_virtual = var->yres;
	if (var->bits_per_pixel <= 1)
		var->bits_per_pixel = 1;
	else if (var->bits_per_pixel <= 8)
		var->bits_per_pixel = 8;
	else if (var->bits_per_pixel <= 16)
		var->bits_per_pixel = 16;
	else if (var->bits_per_pixel <= 24)
		var->bits_per_pixel = 24;
	else if (var->bits_per_pixel <= 32)
		var->bits_per_pixel = 32;
	else
		return -EINVAL;

	if (var->xres_virtual < var->xoffset + var->xres)
		var->xres_virtual = var->xoffset + var->xres;
	if (var->yres_virtual < var->yoffset + var->yres)
		var->yres_virtual = var->yoffset + var->yres;

	/*
	 *  Memory limit
	 */
	line_length =
	    get_line_length(var->xres_virtual, var->bits_per_pixel);
	if (line_length * var->yres_virtual > videomemorysize)
		return -ENOMEM;

	/*
	 * Now that we checked it we alter var. The reason being is that the video
	 * mode passed in might not work but slight changes to it might make it 
	 * work. This way we let the user know what is acceptable.
	 */
	switch (var->bits_per_pixel) {
	case 1:
	case 8:
		var->red.offset = 0;
		var->red.length = 8;
		var->green.offset = 0;
		var->green.length = 8;
		var->blue.offset = 0;
		var->blue.length = 8;
		var->transp.offset = 0;
		var->transp.length = 0;
		break;
	case 16:		/* RGBA 5551 */
		if (var->transp.length) {
			var->red.offset = 0;
			var->red.length = 5;
			var->green.offset = 5;
			var->green.length = 5;
			var->blue.offset = 10;
			var->blue.length = 5;
			var->transp.offset = 15;
			var->transp.length = 1;
		} else {	/* RGB 565 */
			var->red.offset = 0;
			var->red.length = 5;
			var->green.offset = 5;
			var->green.length = 6;
			var->blue.offset = 11;
			var->blue.length = 5;
			var->transp.offset = 0;
			var->transp.length = 0;
		}
		break;
	case 24:		/* RGB 888 */
		var->red.offset = 0;
		var->red.length = 8;
		var->green.offset = 8;
		var->green.length = 8;
		var->blue.offset = 16;
		var->blue.length = 8;
		var->transp.offset = 0;
		var->transp.length = 0;
		break;
	case 32:		/* RGBA 8888 */
		var->red.offset = 0;
		var->red.length = 8;
		var->green.offset = 8;
		var->green.length = 8;
		var->blue.offset = 16;
		var->blue.length = 8;
		var->transp.offset = 24;
		var->transp.length = 8;
		break;
	}
	var->red.msb_right = 0;
	var->green.msb_right = 0;
	var->blue.msb_right = 0;
	var->transp.msb_right = 0;

	return 0;
}

/* This routine actually sets the video mode. It's in here where we
 * the hardware state info->par and fix which can be affected by the 
 * change in par. For this driver it doesn't do much. 
 */
static int vfb_set_par(struct fb_info *info)
{
	info->fix.line_length = get_line_length(info->var.xres_virtual,
						info->var.bits_per_pixel);
	return 0;
}

    /*
     *  Set a single color register. The values supplied are already
     *  rounded down to the hardware's capabilities (according to the
     *  entries in the var structure). Return != 0 for invalid regno.
     */

static int vfb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
			 u_int transp, struct fb_info *info)
{
	if (regno >= 256)	/* no. of hw registers */
		return 1;
	/*
	 * Program hardware... do anything you want with transp
	 */

	/* grayscale works only partially under directcolor */
	if (info->var.grayscale) {
		/* grayscale = 0.30*R + 0.59*G + 0.11*B */
		red = green = blue =
		    (red * 77 + green * 151 + blue * 28) >> 8;
	}

	/* Directcolor:
	 *   var->{color}.offset contains start of bitfield
	 *   var->{color}.length contains length of bitfield
	 *   {hardwarespecific} contains width of RAMDAC
	 *   cmap[X] is programmed to (X << red.offset) | (X << green.offset) | (X << blue.offset)
	 *   RAMDAC[X] is programmed to (red, green, blue)
	 *
	 * Pseudocolor:
	 *    var->{color}.offset is 0 unless the palette index takes less than
	 *                        bits_per_pixel bits and is stored in the upper
	 *                        bits of the pixel value
	 *    var->{color}.length is set so that 1 << length is the number of available
	 *                        palette entries
	 *    cmap is not used
	 *    RAMDAC[X] is programmed to (red, green, blue)
	 *
	 * Truecolor:
	 *    does not use DAC. Usually 3 are present.
	 *    var->{color}.offset contains start of bitfield
	 *    var->{color}.length contains length of bitfield
	 *    cmap is programmed to (red << red.offset) | (green << green.offset) |
	 *                      (blue << blue.offset) | (transp << transp.offset)
	 *    RAMDAC does not exist
	 */
#define CNVT_TOHW(val,width) ((((val)<<(width))+0x7FFF-(val))>>16)
	switch (info->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
	case FB_VISUAL_PSEUDOCOLOR:
		red = CNVT_TOHW(red, info->var.red.length);
		green = CNVT_TOHW(green, info->var.green.length);
		blue = CNVT_TOHW(blue, info->var.blue.length);
		transp = CNVT_TOHW(transp, info->var.transp.length);
		break;
	case FB_VISUAL_DIRECTCOLOR:
		red = CNVT_TOHW(red, 8);	/* expect 8 bit DAC */
		green = CNVT_TOHW(green, 8);
		blue = CNVT_TOHW(blue, 8);
		/* hey, there is bug in transp handling... */
		transp = CNVT_TOHW(transp, 8);
		break;
	}
#undef CNVT_TOHW
	/* Truecolor has hardware independent palette */
	if (info->fix.visual == FB_VISUAL_TRUECOLOR) {
		u32 v;

		if (regno >= 16)
			return 1;

		v = (red << info->var.red.offset) |
		    (green << info->var.green.offset) |
		    (blue << info->var.blue.offset) |
		    (transp << info->var.transp.offset);
		switch (info->var.bits_per_pixel) {
		case 8:
			break;
		case 16:
			((u32 *) (info->pseudo_palette))[regno] = v;
			break;
		case 24:
		case 32:
			((u32 *) (info->pseudo_palette))[regno] = v;
			break;
		}
		return 0;
	}
	return 0;
}

    /*
     *  Pan or Wrap the Display
     *
     *  This call looks only at xoffset, yoffset and the FB_VMODE_YWRAP flag
     */

static int vfb_pan_display(struct fb_var_screeninfo *var,
			   struct fb_info *info)
{
	if (var->vmode & FB_VMODE_YWRAP) {
		if (var->yoffset >= info->var.yres_virtual ||
		    var->xoffset)
			return -EINVAL;
	} else {
		if (var->xoffset + info->var.xres > info->var.xres_virtual ||
		    var->yoffset + info->var.yres > info->var.yres_virtual)
			return -EINVAL;
	}
	info->var.xoffset = var->xoffset;
	info->var.yoffset = var->yoffset;
	if (var->vmode & FB_VMODE_YWRAP)
		info->var.vmode |= FB_VMODE_YWRAP;
	else
		info->var.vmode &= ~FB_VMODE_YWRAP;
	return 0;
}

    /*
     *  Most drivers don't need their own mmap function 
     */

static int vfb_mmap(struct fb_info *info,
		    struct vm_area_struct *vma)
{
	unsigned long start = vma->vm_start;
	unsigned long size = vma->vm_end - vma->vm_start;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	unsigned long page, pos;

	if (vma->vm_pgoff > (~0UL >> PAGE_SHIFT))
		return -EINVAL;
	if (size > info->fix.smem_len)
		return -EINVAL;
	if (offset > info->fix.smem_len - size)
		return -EINVAL;

	pos = (unsigned long)info->fix.smem_start + offset;

	while (size > 0) {
		page = vmalloc_to_pfn((void *)pos);
		if (remap_pfn_range(vma, start, page, PAGE_SIZE, PAGE_SHARED)) {
			return -EAGAIN;
		}
		start += PAGE_SIZE;
		pos += PAGE_SIZE;
		if (size > PAGE_SIZE)
			size -= PAGE_SIZE;
		else
			size = 0;
	}

	return 0;

}

#ifndef MODULE
/*
 * The virtual framebuffer driver is only enabled if explicitly
 * requested by passing 'video=vfb:' (or any actual options).
 */
static int __init vfb_setup(char *options)
{
	char *this_opt;

	vfb_enable = 0;

	if (!options)
		return 1;

	vfb_enable = 1;

	if (!*options)
		return 1;

	while ((this_opt = strsep(&options, ",")) != NULL) {
		if (!*this_opt)
			continue;
		/* Test disable for backwards compatibility */
		if (!strcmp(this_opt, "disable"))
			vfb_enable = 0;
	}
	return 1;
}
#endif  /*  MODULE  */

    /*
     *  Initialisation
     */
#endif
int vfb_usb_probe(struct usb_skel *dev)
{
	int retval;
	int i;
	/******************/
    dev->gdev = &dev->udev->dev;
    dev->info = framebuffer_alloc(0, dev->gdev);
    dev->info->par = dev;
    dev->info->dev = dev->gdev;

    if (!dev->info) 
    {
        printk("Can not allocate memory for fb_info structure\n");
        retval = -ENOMEM;
        goto exit;
    }

    dev->info->fix = fixed_info;
    dev->info->var = var_info;

    dev->info->fix.smem_len=FP_PAGE_COUNT*PAGE_SIZE;

    printk("Allocating framebuffer: %d bytes [%lu pages]\n",dev->info->fix.smem_len,FP_PAGE_COUNT);
    
    videomemory=rvmalloc(dev->info->fix.smem_len);   
    if (!videomemory) 
    {
        printk("Can not allocate memory for framebuffer\n");
        retval = -ENOMEM;
        goto exit;
    }
        
    dev->info->fix.smem_start =(unsigned long)(videomemory);
    dev->info->fbops = &display_fbops;
    dev->info->flags = FBINFO_FLAG_DEFAULT|FBINFO_VIRTFB;
    dev->info->screen_base = videomemory;

        
    memset((void *)dev->info->fix.smem_start, 0, dev->info->fix.smem_len);

    for(i=0;i<FP_PAGE_COUNT;i++)
    {
        dev->videopages[i].mem=(void *)(dev->info->fix.smem_start+PAGE_SIZE*i);
        dev->videopages[i].length=PAGE_SIZE;
        atomic_set(&dev->videopages[i].toUpdate,-1);
        dev->videopages[i].y=(((unsigned long)(PAGE_SIZE*i)>>1)/WIDTH);
        dev->videopages[i].x=((unsigned long)(PAGE_SIZE*i)>>1)-dev->videopages[i].y*WIDTH;
    }
    dev->videopages[FP_PAGE_COUNT-1].length=FB_SIZE-(FP_PAGE_COUNT-1)*PAGE_SIZE;
    dev->info->pseudo_palette = &dev->pseudo_palette;
              
    dev->info->fbdefio=&display_defio;

    fb_deferred_io_init(dev->info);

    printk("info.fix.smem_start=%lu\ninfo.fix.smem_len=%d\ninfo.screen_size=%lu\n",
    	dev->info->fix.smem_start,dev->info->fix.smem_len,dev->info->screen_size);
    retval = register_framebuffer(dev->info);
    if (retval < 0) {
    	printk("Unable to register_frambuffer\n");
    	goto error_buff;
    }
    return retval;

error_buff:
    vfree(videomemory);
exit:
    return retval;
/******************/

}

#define CNVT_TOHW(val,width) ((((val)<<(width))+0x7FFF-(val))>>16)

static int display_setcolreg(unsigned regno,
                               unsigned red, unsigned green, unsigned blue,
                               unsigned transp, struct fb_info *info)
{
        int ret = 1;
        if (info->var.grayscale)
                red = green = blue = (19595 * red + 38470 * green +
                                      7471 * blue) >> 16;
        switch (info->fix.visual) {
        case FB_VISUAL_TRUECOLOR:
                if (regno < 16) {
                        u32 *pal = info->pseudo_palette;
                        u32 value;

                        red = CNVT_TOHW(red, info->var.red.length);
                        green = CNVT_TOHW(green, info->var.green.length);
                        blue = CNVT_TOHW(blue, info->var.blue.length);
                        transp = CNVT_TOHW(transp, info->var.transp.length);

                        value = (red << info->var.red.offset) |
                                (green << info->var.green.offset) |
                                (blue << info->var.blue.offset) |
                                (transp << info->var.transp.offset);

                        pal[regno] = value;
                        ret = 0;
                }
                break;
        case FB_VISUAL_STATIC_PSEUDOCOLOR:
        case FB_VISUAL_PSEUDOCOLOR:
                break;
        }
        return ret;
}
#if 0
static int vfb_probe(struct platform_device *dev)
{
	struct fb_info *info;
	int retval = -ENOMEM;

	/*
	 * For real video cards we use ioremap.
	 */
	if (!(videomemory = rvmalloc(videomemorysize)))
		return retval;

	info = framebuffer_alloc(sizeof(u32) * 256, &dev->dev);
	if (!info)
		goto err;

	info->screen_base = (char __iomem *)videomemory;
	info->fbops = &vfb_ops;

	retval = fb_find_mode(&info->var, info, NULL,
			      NULL, 0, NULL, 8);

	if (!retval || (retval == 4))
		info->var = vfb_default;

	vfb_fix.smem_start = (unsigned long) videomemory;
	vfb_fix.smem_len = videomemorysize;
	info->fix = vfb_fix;
	info->pseudo_palette = info->par;
	info->par = NULL;
	info->flags = FBINFO_FLAG_DEFAULT;

	retval = fb_alloc_cmap(&info->cmap, 256, 0);
	if (retval < 0)
		goto err1;

	retval = register_framebuffer(info);
	if (retval < 0)
		goto err2;
	platform_set_drvdata(dev, info);

	fb_info(info, "VFB device, using %ldK blue offset %d length %d\n",
		videomemorysize >> 10, info->var.blue.offset, info->var.blue.length);
	return 0;
err2:
	fb_dealloc_cmap(&info->cmap);
err1:
	framebuffer_release(info);
err:
	rvfree(videomemory, videomemorysize);
	return retval;
}

static int vfb_remove(struct platform_device *dev)
{
	struct fb_info *info = platform_get_drvdata(dev);

	if (info) {
		unregister_framebuffer(info);
		rvfree(videomemory, videomemorysize);
		fb_dealloc_cmap(&info->cmap);
		framebuffer_release(info);
	}
	return 0;
}

static struct platform_driver vfb_driver = {
	.probe	= vfb_probe,
	.remove = vfb_remove,
	.driver = {
		.name	= "vfb",
	},
};

static struct platform_device *vfb_device;

static int __init vfb_init(void)
{
	int ret = 0;

#ifndef MODULE
	char *option = NULL;

	if (fb_get_options("vfb", &option))
		return -ENODEV;
	vfb_setup(option);
#endif

	if (!vfb_enable)
		return -ENXIO;

	ret = platform_driver_register(&vfb_driver);

	if (!ret) {
		vfb_device = platform_device_alloc("vfb", 0);

		if (vfb_device)
			ret = platform_device_add(vfb_device);
		else
			ret = -ENOMEM;

		if (ret) {
			platform_device_put(vfb_device);
			platform_driver_unregister(&vfb_driver);
		}
	}

	return ret;
}

module_init(vfb_init);

#ifdef MODULE
static void __exit vfb_exit(void)
{
	platform_device_unregister(vfb_device);
	platform_driver_unregister(&vfb_driver);
}

module_exit(vfb_exit);


MODULE_LICENSE("GPL");
#endif				/* MODULE */
#endif
