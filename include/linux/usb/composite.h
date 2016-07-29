/*
 * composite.h -- framework for usb gadgets which are composite devices
 *
 * Copyright (C) 2006-2008 David Brownell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef	__LINUX_USB_COMPOSITE_H
#define	__LINUX_USB_COMPOSITE_H


#include <linux/bcd.h>
#include <linux/version.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/log2.h>
#include <linux/configfs.h>
#include <linux/switch.h>	

#define FUNC_SUSPEND_OPT_SUSP_MASK BIT(0)
#define FUNC_SUSPEND_OPT_RW_EN_MASK BIT(1)

#define USB_GADGET_DELAYED_STATUS       0x7fff	

#define USB_COMP_EP0_BUFSIZ	4096

#define USB_MS_TO_HS_INTERVAL(x)	(ilog2((x * 1000 / 125)) + 1)
struct usb_configuration;

struct usb_os_desc_ext_prop {
	struct list_head	entry;
	u8			type;
	int			name_len;
	char			*name;
	int			data_len;
	char			*data;
	struct config_item	item;
};

struct usb_os_desc {
	char			*ext_compat_id;
	struct list_head	ext_prop;
	int			ext_prop_len;
	int			ext_prop_count;
	struct mutex		*opts_mutex;
	struct config_group	group;
	struct module		*owner;
};

struct usb_os_desc_table {
	int			if_id;
	struct usb_os_desc	*os_desc;
};


struct usb_function {
	const char			*name;
	int				intf_id;
	struct usb_gadget_strings	**strings;
	struct usb_descriptor_header	**fs_descriptors;
	struct usb_descriptor_header	**hs_descriptors;
	struct usb_descriptor_header	**ss_descriptors;

	struct usb_configuration	*config;

	struct usb_os_desc_table	*os_desc_table;
	unsigned			os_desc_n;


	
	int			(*bind)(struct usb_configuration *,
					struct usb_function *);
	void			(*unbind)(struct usb_configuration *,
					struct usb_function *);
	void			(*free_func)(struct usb_function *f);
	struct module		*mod;

	
	int			(*set_alt)(struct usb_function *,
					unsigned interface, unsigned alt);
	int			(*get_alt)(struct usb_function *,
					unsigned interface);
	void			(*disable)(struct usb_function *);
	int			(*setup)(struct usb_function *,
					const struct usb_ctrlrequest *);
	void			(*suspend)(struct usb_function *);
	void			(*resume)(struct usb_function *);

	
	int			(*get_status)(struct usb_function *);
	int			(*func_suspend)(struct usb_function *,
						u8 suspend_opt);
	unsigned		func_is_suspended:1;
	unsigned		func_wakeup_allowed:1;
	unsigned		func_wakeup_pending:1;
	
	
	struct list_head		list;
	DECLARE_BITMAP(endpoints, 32);
	const struct usb_function_instance *fi;
};

int usb_add_function(struct usb_configuration *, struct usb_function *);

int usb_function_deactivate(struct usb_function *);
int usb_function_activate(struct usb_function *);

int usb_interface_id(struct usb_configuration *, struct usb_function *);
int usb_func_wakeup(struct usb_function *func);

int usb_get_func_interface_id(struct usb_function *func);

int config_ep_by_speed(struct usb_gadget *g, struct usb_function *f,
			struct usb_ep *_ep);

#define	MAX_CONFIG_INTERFACES		16	

struct usb_configuration {
	const char			*label;
	struct usb_gadget_strings	**strings;
	const struct usb_descriptor_header **descriptors;


	
	void			(*unbind)(struct usb_configuration *);
	int			(*setup)(struct usb_configuration *,
					const struct usb_ctrlrequest *);

	
	u8			bConfigurationValue;
	u8			iConfiguration;
	u8			bmAttributes;
	u16			MaxPower;

	struct usb_composite_dev	*cdev;

	
	
	struct list_head	list;
	struct list_head	functions;
	u8			next_interface_id;
	unsigned		superspeed:1;
	unsigned		highspeed:1;
	unsigned		fullspeed:1;
	struct usb_function	*interface[MAX_CONFIG_INTERFACES];

	
	int			num_ineps_used;
	int			num_outeps_used;
};

int usb_add_config(struct usb_composite_dev *,
		struct usb_configuration *,
		int (*)(struct usb_configuration *));

void usb_remove_config(struct usb_composite_dev *,
		struct usb_configuration *);

enum {
	USB_GADGET_MANUFACTURER_IDX	= 0,
	USB_GADGET_PRODUCT_IDX,
	USB_GADGET_SERIAL_IDX,
	USB_GADGET_FIRST_AVAIL_IDX,
};

struct usb_composite_driver {
	const char				*name;
	const struct usb_device_descriptor	*dev;
	struct usb_gadget_strings		**strings;
	enum usb_device_speed			max_speed;
	unsigned		needs_serial:1;

	int			(*bind)(struct usb_composite_dev *cdev);
	int			(*unbind)(struct usb_composite_dev *);

	void			(*disconnect)(struct usb_composite_dev *);

	
	void			(*suspend)(struct usb_composite_dev *);
	void			(*resume)(struct usb_composite_dev *);
	struct usb_gadget_driver		gadget_driver;
};

extern int usb_composite_probe(struct usb_composite_driver *driver);
extern void usb_composite_unregister(struct usb_composite_driver *driver);

#define module_usb_composite_driver(__usb_composite_driver) \
	module_driver(__usb_composite_driver, usb_composite_probe, \
		       usb_composite_unregister)

extern void usb_composite_setup_continue(struct usb_composite_dev *cdev);
extern int composite_dev_prepare(struct usb_composite_driver *composite,
		struct usb_composite_dev *cdev);
extern int composite_os_desc_req_prepare(struct usb_composite_dev *cdev,
					 struct usb_ep *ep0);
void composite_dev_cleanup(struct usb_composite_dev *cdev);

static inline struct usb_composite_driver *to_cdriver(
		struct usb_gadget_driver *gdrv)
{
	return container_of(gdrv, struct usb_composite_driver, gadget_driver);
}

#define OS_STRING_QW_SIGN_LEN		14
#define OS_STRING_IDX			0xEE

struct usb_composite_dev {
	struct usb_gadget		*gadget;
	struct usb_request		*req;
	struct usb_request		*os_desc_req;

	struct usb_configuration	*config;

	
	u8				qw_sign[OS_STRING_QW_SIGN_LEN];
	u8				b_vendor_code;
	struct usb_configuration	*os_desc_config;
	unsigned int			use_os_string:1;

	
	
	unsigned int			suspended:1;
	struct usb_device_descriptor	desc;
	struct list_head		configs;
	struct list_head		gstrings;
	struct usb_composite_driver	*driver;
	struct switch_dev		sw_function_switch_on;
	struct switch_dev		sw_function_switch_off;
	struct switch_dev		sw_connect2pc;	

	u8				next_string_id;
	char				*def_manufacturer;

	unsigned			deactivations;

	int				delayed_status;

	
	spinlock_t			lock;
};

extern int usb_string_id(struct usb_composite_dev *c);
extern int usb_string_ids_tab(struct usb_composite_dev *c,
			      struct usb_string *str);
extern struct usb_string *usb_gstrings_attach(struct usb_composite_dev *cdev,
		struct usb_gadget_strings **sp, unsigned n_strings);

extern int usb_string_ids_n(struct usb_composite_dev *c, unsigned n);

extern void composite_disconnect(struct usb_gadget *gadget);
extern int composite_setup(struct usb_gadget *gadget,
		const struct usb_ctrlrequest *ctrl);

struct usb_composite_overwrite {
	u16	idVendor;
	u16	idProduct;
	u16	bcdDevice;
	char	*serial_number;
	char	*manufacturer;
	char	*product;
};
#define USB_GADGET_COMPOSITE_OPTIONS()					\
	static struct usb_composite_overwrite coverwrite;		\
									\
	module_param_named(idVendor, coverwrite.idVendor, ushort, S_IRUGO); \
	MODULE_PARM_DESC(idVendor, "USB Vendor ID");			\
									\
	module_param_named(idProduct, coverwrite.idProduct, ushort, S_IRUGO); \
	MODULE_PARM_DESC(idProduct, "USB Product ID");			\
									\
	module_param_named(bcdDevice, coverwrite.bcdDevice, ushort, S_IRUGO); \
	MODULE_PARM_DESC(bcdDevice, "USB Device version (BCD)");	\
									\
	module_param_named(iSerialNumber, coverwrite.serial_number, charp, \
			S_IRUGO); \
	MODULE_PARM_DESC(iSerialNumber, "SerialNumber string");		\
									\
	module_param_named(iManufacturer, coverwrite.manufacturer, charp, \
			S_IRUGO); \
	MODULE_PARM_DESC(iManufacturer, "USB Manufacturer string");	\
									\
	module_param_named(iProduct, coverwrite.product, charp, S_IRUGO); \
	MODULE_PARM_DESC(iProduct, "USB Product string")

void usb_composite_overwrite_options(struct usb_composite_dev *cdev,
		struct usb_composite_overwrite *covr);

static inline u16 get_default_bcdDevice(void)
{
	u16 bcdDevice;

	bcdDevice = bin2bcd((LINUX_VERSION_CODE >> 16 & 0xff)) << 8;
	bcdDevice |= bin2bcd((LINUX_VERSION_CODE >> 8 & 0xff));
	return bcdDevice;
}

struct usb_function_driver {
	const char *name;
	struct module *mod;
	struct list_head list;
	struct usb_function_instance *(*alloc_inst)(void);
	struct usb_function *(*alloc_func)(struct usb_function_instance *inst);
};

struct usb_function_instance {
	struct config_group group;
	struct list_head cfs_list;
	struct usb_function_driver *fd;
	struct usb_function *f;
	int (*set_inst_name)(struct usb_function_instance *inst,
			      const char *name);
	void (*free_func_inst)(struct usb_function_instance *inst);
};

void usb_function_unregister(struct usb_function_driver *f);
int usb_function_register(struct usb_function_driver *newf);
void usb_put_function_instance(struct usb_function_instance *fi);
void usb_put_function(struct usb_function *f);
struct usb_function_instance *usb_get_function_instance(const char *name);
struct usb_function *usb_get_function(struct usb_function_instance *fi);

struct usb_configuration *usb_get_config(struct usb_composite_dev *cdev,
		int val);
int usb_add_config_only(struct usb_composite_dev *cdev,
		struct usb_configuration *config);
void usb_remove_function(struct usb_configuration *c, struct usb_function *f);

#define DECLARE_USB_FUNCTION(_name, _inst_alloc, _func_alloc)		\
	static struct usb_function_driver _name ## usb_func = {		\
		.name = __stringify(_name),				\
		.mod  = THIS_MODULE,					\
		.alloc_inst = _inst_alloc,				\
		.alloc_func = _func_alloc,				\
	};								\
	MODULE_ALIAS("usbfunc:"__stringify(_name));

#define DECLARE_USB_FUNCTION_INIT(_name, _inst_alloc, _func_alloc)	\
	DECLARE_USB_FUNCTION(_name, _inst_alloc, _func_alloc)		\
	static int __init _name ## mod_init(void)			\
	{								\
		return usb_function_register(&_name ## usb_func);	\
	}								\
	static void __exit _name ## mod_exit(void)			\
	{								\
		usb_function_unregister(&_name ## usb_func);		\
	}								\
	module_init(_name ## mod_init);					\
	module_exit(_name ## mod_exit)

#define DBG(d, fmt, args...) \
	dev_dbg(&(d)->gadget->dev , fmt , ## args)
#define VDBG(d, fmt, args...) \
	dev_vdbg(&(d)->gadget->dev , fmt , ## args)
#define ERROR(d, fmt, args...) \
	dev_err(&(d)->gadget->dev , fmt , ## args)
#define WARNING(d, fmt, args...) \
	dev_warn(&(d)->gadget->dev , fmt , ## args)
#define INFO(d, fmt, args...) \
	dev_info(&(d)->gadget->dev , fmt , ## args)

#endif	
