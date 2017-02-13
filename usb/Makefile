OPENWRT = ${HOME}/workspace/openwrt
KERNELDIR = ${OPENWRT}/build_dir/target-mips_34kc_musl-1.1.16/linux-ar71xx_generic/linux-4.4.14
export STAGING_DIR=${OPENWRT}/staging_dir/toolchain-mips_34kc_gcc-5.3.0_musl-1.1.16/bin
# Comment/uncomment the following line to disable/enable debugging
DEBUG = y
#obj-m	:= vfb.o
#obj-m	:= usb-skeleton.o
obj-m	:= vfb_lcd.o
vfb_lcd-objs := usb-vfb.o vfb-core.o

# Add your debugging flag (or not) to EXTRA_CFLAGS
ifeq ($(DEBUG),y)
  DEBFLAGS = -O -g # "-O" is needed to expand inlines
else
  DEBFLAGS = -O2
endif

EXTRA_CFLAGS += $(DEBFLAGS)

PWD       := $(shell pwd)

all:
	$(MAKE) -C $(KERNELDIR) ARCH="mips" CC="${OPENWRT}/staging_dir/toolchain-mips_34kc_gcc-5.3.0_musl-1.1.16/bin/mips-openwrt-linux-musl-gcc" LD="${OPENWRT}/staging_dir/toolchain-mips_34kc_gcc-5.3.0_musl-1.1.16/bin/mips-openwrt-linux-musl-ld" M=$(PWD)

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions

