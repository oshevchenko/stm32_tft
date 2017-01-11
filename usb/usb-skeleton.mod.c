#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
 .name = KBUILD_MODNAME,
 .init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
 .exit = cleanup_module,
#endif
 .arch = MODULE_ARCH_INIT,
};

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0xd4733cff, "module_layout" },
	{ 0x4b52ed7c, "noop_llseek" },
	{ 0xff389b63, "usb_deregister" },
	{ 0xeb3d960c, "usb_register_driver" },
	{ 0xcd0550ab, "usb_unanchor_urb" },
	{ 0x3dd877ca, "usb_anchor_urb" },
	{ 0x33d169c9, "_copy_from_user" },
	{ 0xf1afe957, "usb_alloc_coherent" },
	{ 0x21b08e39, "down_trylock" },
	{ 0x2addc0be, "down_interruptible" },
	{ 0x2da418b5, "copy_to_user" },
	{ 0x75bb675a, "finish_wait" },
	{ 0x4292364c, "schedule" },
	{ 0x622fa02a, "prepare_to_wait" },
	{ 0xc8b57c27, "autoremove_wake_function" },
	{ 0x95435001, "current_task" },
	{ 0xdd3afd66, "mutex_lock_interruptible" },
	{ 0xa3bbd047, "usb_submit_urb" },
	{ 0x8ff4079b, "pv_irq_ops" },
	{ 0xf1faac3a, "_raw_spin_lock_irq" },
	{ 0xf781259f, "usb_register_dev" },
	{ 0xb6bcdacf, "usb_alloc_urb" },
	{ 0x12da5bb2, "__kmalloc" },
	{ 0xb3945d82, "usb_get_dev" },
	{ 0x48eb0c0d, "__init_waitqueue_head" },
	{ 0xce65a0a8, "__mutex_init" },
	{ 0x83800bfa, "kref_init" },
	{ 0x6fe340df, "kmem_cache_alloc_trace" },
	{ 0x62c1ade7, "malloc_sizes" },
	{ 0x37a0cba, "kfree" },
	{ 0x4f42db44, "usb_put_dev" },
	{ 0x2672ca7a, "usb_free_urb" },
	{ 0x7e5d5a9e, "_dev_info" },
	{ 0xe0a07776, "usb_deregister_dev" },
	{ 0xb2ba53eb, "dev_set_drvdata" },
	{ 0xe45f60d8, "__wake_up" },
	{ 0xc4554217, "up" },
	{ 0x6705c4e, "usb_free_coherent" },
	{ 0x67f7403e, "_raw_spin_lock" },
	{ 0xa8749777, "usb_autopm_get_interface" },
	{ 0x9775cdc, "kref_get" },
	{ 0x50eedeb8, "printk" },
	{ 0xd9575b33, "usb_find_interface" },
	{ 0xd5b037e1, "kref_put" },
	{ 0x51123b9a, "usb_autopm_put_interface" },
	{ 0xa5dd69ea, "mutex_lock" },
	{ 0x96de5def, "usb_kill_urb" },
	{ 0x8c4db1b5, "usb_kill_anchored_urbs" },
	{ 0xeec24d31, "usb_wait_anchor_empty_timeout" },
	{ 0xcc99e735, "mutex_unlock" },
	{ 0x74f13e03, "dev_get_drvdata" },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=usbcore";

MODULE_ALIAS("usb:vFFF0pFFF0d*dc*dsc*dp*ic*isc*ip*");
MODULE_ALIAS("usb:v0525pA4A0d*dc*dsc*dp*ic*isc*ip*");
