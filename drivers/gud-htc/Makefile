#
# Makefile for the <t-base core and trusted UI drivers
#

ifneq (,$(findstring N70,$(PRIVATE_RCMS_NAME)))
obj-$(CONFIG_TRUSTONIC_TEE) := 310d/MobiCoreDriver/
obj-$(CONFIG_TRUSTONIC_TRUSTED_UI) += 310d/TlcTui/
else
obj-$(CONFIG_TRUSTONIC_TEE) := 310a/MobiCoreDriver/
obj-$(CONFIG_TRUSTONIC_TRUSTED_UI) += 310a/TlcTui/
endif
