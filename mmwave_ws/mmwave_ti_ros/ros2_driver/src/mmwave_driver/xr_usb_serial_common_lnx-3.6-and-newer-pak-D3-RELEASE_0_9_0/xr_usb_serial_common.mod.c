#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
#include <linux/export-internal.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

#ifdef CONFIG_UNWINDER_ORC
#include <asm/orc_header.h>
ORC_HEADER;
#endif

BUILD_SALT;
BUILD_LTO_INFO;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif



static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0x3213f038, "mutex_unlock" },
	{ 0x920bbbe9, "tty_standard_install" },
	{ 0x296695f, "refcount_warn_saturate" },
	{ 0x2d3385d3, "system_wq" },
	{ 0xc5b6f236, "queue_work_on" },
	{ 0x284f0c94, "usb_put_intf" },
	{ 0x37a0cba, "kfree" },
	{ 0x5602c65e, "gpiochip_remove" },
	{ 0xe77cf69f, "device_remove_file" },
	{ 0xb3b27998, "tty_port_tty_get" },
	{ 0x2b232cbe, "tty_vhangup" },
	{ 0xe3dffa38, "tty_kref_put" },
	{ 0x2cbfcb58, "tty_unregister_device" },
	{ 0x3b509c18, "usb_free_urb" },
	{ 0x3c7b0252, "usb_free_coherent" },
	{ 0x481be5c8, "usb_driver_release_interface" },
	{ 0xa648e561, "__ubsan_handle_shift_out_of_bounds" },
	{ 0x4c03a563, "random_kmalloc_seed" },
	{ 0xb22ef59e, "kmalloc_caches" },
	{ 0xd8dcb679, "kmalloc_trace" },
	{ 0xd4125490, "usb_autopm_get_interface" },
	{ 0x72c0aabe, "usb_autopm_put_interface" },
	{ 0x7ad117cd, "tty_port_open" },
	{ 0xbd394d8, "tty_termios_baud_rate" },
	{ 0x6729d3df, "__get_user_4" },
	{ 0x13c49cc2, "_copy_from_user" },
	{ 0xc6cbbc89, "capable" },
	{ 0x6b10bee1, "_copy_to_user" },
	{ 0xb2fd5ceb, "__put_user_4" },
	{ 0x7dfa79ba, "usb_ifnum_to_if" },
	{ 0xcefb0c9f, "__mutex_init" },
	{ 0x28ea66ee, "tty_port_init" },
	{ 0x815670eb, "usb_alloc_coherent" },
	{ 0xd119754e, "usb_alloc_urb" },
	{ 0xf0b2461, "_dev_warn" },
	{ 0x5139ec05, "device_create_file" },
	{ 0xeb233a45, "__kmalloc" },
	{ 0xe456497a, "_dev_info" },
	{ 0x4f3d80a9, "usb_driver_claim_interface" },
	{ 0xe36ef9db, "usb_get_intf" },
	{ 0x377f91ad, "tty_port_register_device" },
	{ 0x3c65c082, "gpiochip_add_data_with_key" },
	{ 0xcbd4898c, "fortify_panic" },
	{ 0xbdfb6dbb, "__fentry__" },
	{ 0x8427cc7b, "_raw_spin_lock_irq" },
	{ 0x4b750f53, "_raw_spin_unlock_irq" },
	{ 0x5b8239ca, "__x86_return_thunk" },
	{ 0x7c91e60d, "usb_control_msg" },
	{ 0xf248a526, "__dynamic_dev_dbg" },
	{ 0xa657001, "__tty_alloc_driver" },
	{ 0x67b27ec1, "tty_std_termios" },
	{ 0x527c7f49, "tty_register_driver" },
	{ 0xbc621c09, "usb_register_driver" },
	{ 0x122c3a7e, "_printk" },
	{ 0x7554316e, "tty_unregister_driver" },
	{ 0xb1d16cd2, "tty_driver_kref_put" },
	{ 0x288087ed, "usb_submit_urb" },
	{ 0x548d9097, "_dev_err" },
	{ 0x87a21cb3, "__ubsan_handle_out_of_bounds" },
	{ 0x3f997aba, "usb_autopm_put_interface_async" },
	{ 0x8368c2ff, "usb_kill_urb" },
	{ 0x3c12dfe, "cancel_work_sync" },
	{ 0xba8fbd64, "_raw_spin_lock" },
	{ 0xb5b54b34, "_raw_spin_unlock" },
	{ 0x3b93c5f7, "tty_port_put" },
	{ 0x3c3ff9fd, "sprintf" },
	{ 0x6ebe366f, "ktime_get_mono_fast_ns" },
	{ 0x280fa88e, "tty_port_tty_hangup" },
	{ 0x113eee41, "tty_port_tty_wakeup" },
	{ 0x6196e796, "tty_port_hangup" },
	{ 0x28b76828, "tty_port_close" },
	{ 0x7f4714a, "usb_deregister" },
	{ 0x34db050b, "_raw_spin_lock_irqsave" },
	{ 0xd35cce70, "_raw_spin_unlock_irqrestore" },
	{ 0x54b1fac6, "__ubsan_handle_load_invalid_value" },
	{ 0x69acdf38, "memcpy" },
	{ 0xa493a000, "usb_autopm_get_interface_async" },
	{ 0x803a9074, "__tty_insert_flip_string_flags" },
	{ 0x4cb5f893, "tty_flip_buffer_push" },
	{ 0xf0fdf6cb, "__stack_chk_fail" },
	{ 0x4dfa8d4b, "mutex_lock" },
	{ 0x2273f01b, "module_layout" },
};

MODULE_INFO(depends, "");

MODULE_ALIAS("usb:v04E2p1410d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1411d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1412d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1414d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1420d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1421d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1422d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1424d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1400d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1401d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1402d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1403d*dc*dsc*dp*ic*isc*ip*in*");

MODULE_INFO(srcversion, "C062A4E71EE7EF390B85471");
