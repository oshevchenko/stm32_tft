insmod fb_sys_fops.ko
insmod syscopyarea.ko
insmod sysfillrect.ko
insmod sysimgblt.ko
#insmod vfb.ko vfb_enable=1
insmod vfb_lcd.ko
fbset -g 320 240 320 240 16
