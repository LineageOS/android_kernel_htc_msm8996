#
# USB peripheral controller drivers
#
subdir-ccflags-$(CONFIG_USB_GADGET_DEBUG)	:= -DDEBUG
subdir-ccflags-$(CONFIG_USB_GADGET_VERBOSE)	+= -DVERBOSE_DEBUG
ccflags-y				+= -I$(srctree)/drivers/usb/gadget/udc -I$(srctree)/drivers/usb/gadget/function

obj-$(CONFIG_USB_LIBCOMPOSITE)	+= libcomposite.o
libcomposite-y			:= usbstring.o config.o epautoconf.o
#++ 2015/11/16 USB Team, PCN00038 ++
libcomposite-y			+= functions.o u_f.o
#-- 2015/11/16 USB Team, PCN00038 --

obj-y				+= debug.o
g_android-y			:= android.o

obj-$(CONFIG_USB_GADGET)	+= udc/ function/ legacy/

obj-$(CONFIG_USB_G_ANDROID)	+= g_android.o

obj-$(CONFIG_USB_CI13XXX_MSM)	+= ci13xxx_msm.o

obj-$(CONFIG_USB_CI13XXX_MSM_HSIC)	+= ci13xxx_msm_hsic.o
