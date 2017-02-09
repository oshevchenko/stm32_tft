#!/bin/sh

OPENWRT=${HOME}/workspace/openwrt
KERNELDIR=${OPENWRT}/build_dir/target-mips_34kc_musl-1.1.16/linux-ar71xx_generic/linux-4.4.14
export STAGING_DIR=${OPENWRT}/staging_dir/toolchain-mips_34kc_gcc-5.3.0_musl-1.1.16/bin
PATH=$PATH:${OPENWRT}/staging_dir/toolchain-mips_34kc_gcc-5.3.0_musl-1.1.16/bin
export PATH
make CC=mips-openwrt-linux-musl-gcc LD=mips-openwrt-linux-musl-ld
