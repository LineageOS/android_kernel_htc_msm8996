#Android makefile to build kernel as a part of Android Build
PERL		= perl

KERNEL_TARGET := $(strip $(INSTALLED_KERNEL_TARGET))
ifeq ($(KERNEL_TARGET),)
INSTALLED_KERNEL_TARGET := $(PRODUCT_OUT)/kernel
endif

TARGET_KERNEL_ARCH := $(strip $(TARGET_KERNEL_ARCH))
ifeq ($(TARGET_KERNEL_ARCH),)
KERNEL_ARCH := arm
else
KERNEL_ARCH := $(TARGET_KERNEL_ARCH)
endif

TARGET_KERNEL_HEADER_ARCH := $(strip $(TARGET_KERNEL_HEADER_ARCH))
ifeq ($(TARGET_KERNEL_HEADER_ARCH),)
KERNEL_HEADER_ARCH := $(KERNEL_ARCH)
else
$(warning Forcing kernel header generation only for '$(TARGET_KERNEL_HEADER_ARCH)')
KERNEL_HEADER_ARCH := $(TARGET_KERNEL_HEADER_ARCH)
endif

KERNEL_HEADER_DEFCONFIG := $(strip $(KERNEL_HEADER_DEFCONFIG))
ifeq ($(KERNEL_HEADER_DEFCONFIG),)
KERNEL_HEADER_DEFCONFIG := $(KERNEL_DEFCONFIG)
endif

# Force 32-bit binder IPC for 64bit kernel with 32bit userspace
ifeq ($(KERNEL_ARCH),arm64)
ifeq ($(TARGET_ARCH),arm)
KERNEL_CONFIG_OVERRIDE := CONFIG_ANDROID_BINDER_IPC_32BIT=y
endif
endif

TARGET_KERNEL_CROSS_COMPILE_PREFIX := $(strip $(TARGET_KERNEL_CROSS_COMPILE_PREFIX))
ifeq ($(TARGET_KERNEL_CROSS_COMPILE_PREFIX),)
KERNEL_CROSS_COMPILE := arm-eabi-
else
KERNEL_CROSS_COMPILE := $(TARGET_KERNEL_CROSS_COMPILE_PREFIX)
endif

ifeq ($(TARGET_PREBUILT_KERNEL),)

KERNEL_GCC_NOANDROID_CHK := $(shell (echo "int main() {return 0;}" | $(KERNEL_CROSS_COMPILE)gcc -E -mno-android - > /dev/null 2>&1 ; echo $$?))
ifeq ($(strip $(KERNEL_GCC_NOANDROID_CHK)),0)
KERNEL_CFLAGS := KCFLAGS=-mno-android
endif

KERNEL_OUT := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ
KERNEL_CONFIG := $(KERNEL_OUT)/.config

ifeq ($(KERNEL_DEFCONFIG)$(wildcard $(KERNEL_CONFIG)),)
$(error Kernel configuration not defined, cannot build kernel)
else

ifeq ($(TARGET_USES_UNCOMPRESSED_KERNEL),true)
$(info Using uncompressed kernel)
TARGET_PREBUILT_INT_KERNEL := $(KERNEL_OUT)/arch/$(KERNEL_ARCH)/boot/Image
else
ifeq ($(KERNEL_ARCH),arm64)
TARGET_PREBUILT_INT_KERNEL := $(KERNEL_OUT)/arch/$(KERNEL_ARCH)/boot/Image.gz
else
TARGET_PREBUILT_INT_KERNEL := $(KERNEL_OUT)/arch/$(KERNEL_ARCH)/boot/zImage
endif
endif

ifeq ($(TARGET_KERNEL_APPEND_DTB), true)
$(info Using appended DTB)
TARGET_PREBUILT_INT_KERNEL := $(TARGET_PREBUILT_INT_KERNEL)-dtb
endif

KERNEL_HEADERS_INSTALL := $(KERNEL_OUT)/usr
KERNEL_MODULES_INSTALL := system
KERNEL_MODULES_OUT := $(TARGET_OUT)/lib/modules

TARGET_PREBUILT_KERNEL := $(TARGET_PREBUILT_INT_KERNEL)

KERNEL_ENABLE_EXFAT ?= $(shell cat kernel/arch/$(KERNEL_ARCH)/configs/$(KERNEL_DEFCONFIG) | egrep -v "^\s*\#" | egrep "CONFIG_EXFAT_FS" | sed 's/^\s*CONFIG_EXFAT_FS\s*=\s*//' )
KERNEL_EXFAT_PATH ?= $(shell cat kernel/arch/$(KERNEL_ARCH)/configs/$(KERNEL_DEFCONFIG) | egrep -v "^\s*\#" | egrep "CONFIG_EXFAT_PATH" | sed 's/^\s*CONFIG_EXFAT_PATH\s*=\s*\"//' | sed 's/\".*//' )
KERNEL_EXFAT_VERSION ?= $(shell cat kernel/arch/$(KERNEL_ARCH)/configs/$(KERNEL_DEFCONFIG) | egrep -v "^\s*\#" | egrep "CONFIG_EXFAT_VERSION" | sed 's/^\s*CONFIG_EXFAT_VERSION\s*=\s*\"//' | sed 's/\".*//' )
BUILD_PATH ?= $(shell pwd)

define mv-modules
mdpath=`find $(KERNEL_MODULES_OUT) -type f -name modules.dep`;\
if [ "$$mdpath" != "" ];then\
mpath=`dirname $$mdpath`;\
ko=`find $$mpath/kernel -type f -name *.ko`;\
for i in $$ko; do mv $$i $(KERNEL_MODULES_OUT)/; done;\
fi
endef

define clean-module-folder
mdpath=`find $(KERNEL_MODULES_OUT) -type f -name modules.dep`;\
if [ "$$mdpath" != "" ];then\
mpath=`dirname $$mdpath`; rm -rf $$mpath;\
fi
endef

$(KERNEL_OUT):
	mkdir -p $(KERNEL_OUT)

$(KERNEL_CONFIG): $(KERNEL_OUT)
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=$(KERNEL_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) PRIVATE_RCMS_NAME=$(PRIVATE_RCMS_NAME) PRIVATE_SKU_NAME=$(PRIVATE_SKU_NAME) $(KERNEL_DEFCONFIG)
	$(hide) if [ ! -z "$(KERNEL_CONFIG_OVERRIDE)" ]; then \
			echo "Overriding kernel config with '$(KERNEL_CONFIG_OVERRIDE)'"; \
			echo $(KERNEL_CONFIG_OVERRIDE) >> $(KERNEL_OUT)/.config; \
			$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=$(KERNEL_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) PRIVATE_RCMS_NAME=$(PRIVATE_RCMS_NAME) PRIVATE_SKU_NAME=$(PRIVATE_SKU_NAME) oldconfig; fi

$(TARGET_PREBUILT_INT_KERNEL): $(KERNEL_OUT) $(KERNEL_HEADERS_INSTALL)
	$(hide) echo "Building kernel..."
	$(hide) rm -rf $(KERNEL_OUT)/arch/$(KERNEL_ARCH)/boot/dts
ifeq ($(KERNEL_ENABLE_EXFAT), m)
	cp vendor/tuxera/exfat/tuxera_update_htc.sh kernel/
	cp vendor/tuxera/exfat/update_tuxera.sh kernel/
	cp vendor/tuxera/exfat/build_exfat.sh kernel/
	cp -rf vendor/tuxera/exfat/texfat kernel/fs/
	cp -rf vendor/tuxera/exfat/$(KERNEL_EXFAT_PATH) kernel/fs/
	mkdir -p $(KERNEL_OUT)/fs/$(KERNEL_EXFAT_PATH)
endif
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=$(KERNEL_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) PRIVATE_RCMS_NAME=$(PRIVATE_RCMS_NAME) PRIVATE_SKU_NAME=$(PRIVATE_SKU_NAME) $(KERNEL_CFLAGS)
ifeq ($(KERNEL_ENABLE_EXFAT), m)
ifeq ($(HTC_DEBUG_FLAG), DEBUG)
ifeq ($(strip $(KERNEL_EXFAT_VERSION)),)
	./kernel/update_tuxera.sh -p $(KERNEL_EXFAT_PATH) -t target/htc.d/htc -o $(KERNEL_OUT)
else
	./kernel/update_tuxera.sh -p $(KERNEL_EXFAT_PATH) -t $(KERNEL_EXFAT_VERSION) -o $(KERNEL_OUT)
endif
else
ifeq ($(TARGET_BUILD_VARIANT), user)
	$(warning "User-Release")
ifeq ($(strip $(KERNEL_EXFAT_VERSION)),)
	./kernel/update_tuxera.sh -p $(KERNEL_EXFAT_PATH) -t target/htc.d/htc -o $(KERNEL_OUT) -r
else
	./kernel/update_tuxera.sh -p $(KERNEL_EXFAT_PATH) -t $(KERNEL_EXFAT_VERSION) -o $(KERNEL_OUT) -r
endif
else
	$(warning "NonUser-Release")
ifeq ($(strip $(KERNEL_EXFAT_VERSION)),)
	./kernel/update_tuxera.sh -p $(KERNEL_EXFAT_PATH) -t target/htc.d/htc -o $(KERNEL_OUT) -u
else
	./kernel/update_tuxera.sh -p $(KERNEL_EXFAT_PATH) -t $(KERNEL_EXFAT_VERSION) -o $(KERNEL_OUT) -u
endif
endif

endif
endif
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=$(KERNEL_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) PRIVATE_RCMS_NAME=$(PRIVATE_RCMS_NAME) PRIVATE_SKU_NAME=$(PRIVATE_SKU_NAME) $(KERNEL_CFLAGS)
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=$(KERNEL_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) PRIVATE_RCMS_NAME=$(PRIVATE_RCMS_NAME) PRIVATE_SKU_NAME=$(PRIVATE_SKU_NAME) $(KERNEL_CFLAGS) modules
	$(MAKE) -C kernel O=../$(KERNEL_OUT) INSTALL_MOD_PATH=../../$(KERNEL_MODULES_INSTALL) INSTALL_MOD_STRIP=1 ARCH=$(KERNEL_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) PRIVATE_RCMS_NAME=$(PRIVATE_RCMS_NAME) PRIVATE_SKU_NAME=$(PRIVATE_SKU_NAME) modules_install
ifeq ($(KERNEL_ENABLE_EXFAT), m)
ifeq ($(HTC_DEBUG_FLAG), DEBUG)
	# Build exfat modules for DEBUG
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=$(KERNEL_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) PRIVATE_RCMS_NAME=$(PRIVATE_RCMS_NAME) PRIVATE_SKU_NAME=$(PRIVATE_SKU_NAME) SUBDIRS=$(BUILD_PATH)/kernel/fs/$(KERNEL_EXFAT_PATH)/objects modules
	$(MAKE) -C kernel O=../$(KERNEL_OUT) SUBDIRS=fs/$(KERNEL_EXFAT_PATH)/objects INSTALL_MOD_PATH=../../$(KERNEL_MODULES_INSTALL) ARCH=$(KERNEL_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) PRIVATE_RCMS_NAME=$(PRIVATE_RCMS_NAME) PRIVATE_SKU_NAME=$(PRIVATE_SKU_NAME) modules_install
	cp kernel/fs/$(KERNEL_EXFAT_PATH)/objects/texfat.ko $(KERNEL_MODULES_OUT)/
	mkdir -p $(TARGET_OUT)/bin/
	cp -rf kernel/fs/$(KERNEL_EXFAT_PATH)/bin/* $(TARGET_OUT)/bin/
else
ifeq ($(TARGET_BUILD_VARIANT), user)
	# Build exfat modules for NonDebug-USER
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=$(KERNEL_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) PRIVATE_RCMS_NAME=$(PRIVATE_RCMS_NAME) PRIVATE_SKU_NAME=$(PRIVATE_SKU_NAME) SUBDIRS=$(BUILD_PATH)/kernel/fs/$(KERNEL_EXFAT_PATH)/objects-user modules
	$(MAKE) -C kernel O=../$(KERNEL_OUT) SUBDIRS=fs/$(KERNEL_EXFAT_PATH)/objects-user INSTALL_MOD_PATH=../../$(KERNEL_MODULES_INSTALL) ARCH=$(KERNEL_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) PRIVATE_RCMS_NAME=$(PRIVATE_RCMS_NAME) PRIVATE_SKU_NAME=$(PRIVATE_SKU_NAME) modules_install
	cp kernel/fs/$(KERNEL_EXFAT_PATH)/objects-user/texfat.ko $(KERNEL_MODULES_OUT)/
	mkdir -p $(TARGET_OUT)/bin/
	cp -rf kernel/fs/$(KERNEL_EXFAT_PATH)/bin/* $(TARGET_OUT)/bin/
else
	# Build exfat modules for NonDebug-USERDEBUG
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=$(KERNEL_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) PRIVATE_RCMS_NAME=$(PRIVATE_RCMS_NAME) PRIVATE_SKU_NAME=$(PRIVATE_SKU_NAME) SUBDIRS=$(BUILD_PATH)/kernel/fs/$(KERNEL_EXFAT_PATH)/objects-userdebug modules
	$(MAKE) -C kernel O=../$(KERNEL_OUT) SUBDIRS=fs/$(KERNEL_EXFAT_PATH)/objects-userdebug INSTALL_MOD_PATH=../../$(KERNEL_MODULES_INSTALL) ARCH=$(KERNEL_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) PRIVATE_RCMS_NAME=$(PRIVATE_RCMS_NAME) PRIVATE_SKU_NAME=$(PRIVATE_SKU_NAME) modules_install
	cp kernel/fs/$(KERNEL_EXFAT_PATH)/objects-userdebug/texfat.ko $(KERNEL_MODULES_OUT)/
	mkdir -p $(TARGET_OUT)/bin/
	cp -rf kernel/fs/$(KERNEL_EXFAT_PATH)/bin/* $(TARGET_OUT)/bin/
endif
endif
endif
	$(mv-modules)
	$(clean-module-folder)

ifeq ($(KERNEL_ENABLE_EXFAT), m)
	rm kernel/tuxera_update_htc.sh
	rm kernel/update_tuxera.sh
	rm kernel/build_exfat.sh
	rm -rf kernel/fs/texfat*
endif

	$(info start build keydar_build_kernel_modules.sh)
	vendor/mocana/scripts/keydar_build_kernel_modules.sh -v -M -c $(KERNEL_CROSS_COMPILE) -s `pwd`/vendor/mocana/src/mss -k 3.18 -K `pwd`/$(KERNEL_OUT) -e `pwd`/vendor/mocana/src/ecryptfs-mocana -a `pwd`/vendor/mocana/src/crypto-api-template -D `pwd`/$(KERNEL_MODULES_OUT)

ifeq ($(MOCANA_FIPS_MODULE), true)
	$(info start build keydar_build_kernel_modules.sh)
	vendor/mocana/scripts/keydar_build_kernel_modules.sh -v -M -c $(KERNEL_CROSS_COMPILE) -s `pwd`/vendor/mocana/src/mss -k 3.18 -K `pwd`/$(KERNEL_OUT) -r `pwd`/vendor/mocana/src/dm-crypt-mocana -a `pwd`/vendor/mocana/src/crypto-api-template -D `pwd`/$(KERNEL_MODULES_OUT)
endif

$(KERNEL_HEADERS_INSTALL): $(KERNEL_OUT)
	$(hide) if [ ! -z "$(KERNEL_HEADER_DEFCONFIG)" ]; then \
			$(hide) rm -f ../$(KERNEL_CONFIG); \
			$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=$(KERNEL_HEADER_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) PRIVATE_RCMS_NAME=$(PRIVATE_RCMS_NAME) PRIVATE_SKU_NAME=$(PRIVATE_SKU_NAME) $(KERNEL_HEADER_DEFCONFIG); \
			$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=$(KERNEL_HEADER_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) PRIVATE_RCMS_NAME=$(PRIVATE_RCMS_NAME) PRIVATE_SKU_NAME=$(PRIVATE_SKU_NAME) headers_install; fi
	$(hide) if [ "$(KERNEL_HEADER_DEFCONFIG)" != "$(KERNEL_DEFCONFIG)" ]; then \
			echo "Used a different defconfig for header generation"; \
			$(hide) rm -f ../$(KERNEL_CONFIG); \
			$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=$(KERNEL_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) PRIVATE_RCMS_NAME=$(PRIVATE_RCMS_NAME) PRIVATE_SKU_NAME=$(PRIVATE_SKU_NAME) $(KERNEL_DEFCONFIG); fi
	$(hide) if [ ! -z "$(KERNEL_CONFIG_OVERRIDE)" ]; then \
			echo "Overriding kernel config with '$(KERNEL_CONFIG_OVERRIDE)'"; \
			echo $(KERNEL_CONFIG_OVERRIDE) >> $(KERNEL_OUT)/.config; \
			$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=$(KERNEL_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) PRIVATE_RCMS_NAME=$(PRIVATE_RCMS_NAME) PRIVATE_SKU_NAME=$(PRIVATE_SKU_NAME) oldconfig; fi

kerneltags: $(KERNEL_OUT) $(KERNEL_CONFIG)
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=$(KERNEL_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) PRIVATE_RCMS_NAME=$(PRIVATE_RCMS_NAME) PRIVATE_SKU_NAME=$(PRIVATE_SKU_NAME) tags

kernelconfig: $(KERNEL_OUT) $(KERNEL_CONFIG)
	env KCONFIG_NOTIMESTAMP=true \
	     $(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=$(KERNEL_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) PRIVATE_RCMS_NAME=$(PRIVATE_RCMS_NAME) PRIVATE_SKU_NAME=$(PRIVATE_SKU_NAME) menuconfig
	env KCONFIG_NOTIMESTAMP=true \
	     $(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=$(KERNEL_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) PRIVATE_RCMS_NAME=$(PRIVATE_RCMS_NAME) PRIVATE_SKU_NAME=$(PRIVATE_SKU_NAME) savedefconfig
	cp $(KERNEL_OUT)/defconfig kernel/arch/$(KERNEL_ARCH)/configs/$(KERNEL_DEFCONFIG)

endif
endif

# Hack defconfig for QTI USFP
ifeq ($(HTC_QTI_FINGERPRINT), 1)
    $(shell sed -i 's/CONFIG_MSM_QBT1000=n/CONFIG_MSM_QBT1000=y/g' kernel/arch/$(KERNEL_ARCH)/configs/$(KERNEL_DEFCONFIG))
    $(shell sed -i 's/CONFIG_EEPROM_AT24=n/CONFIG_EEPROM_AT24=y/g' kernel/arch/$(KERNEL_ARCH)/configs/$(KERNEL_DEFCONFIG))
endif
