#
# Makefile for miscellaneous I2C chip drivers.
#
# Do not add new drivers to this directory! It is DEPRECATED.
#
# Device drivers are better grouped according to the functionality they
# implement rather than to the bus they are connected to. In particular:
# * Hardware monitoring chip drivers go to drivers/hwmon
# * RTC chip drivers go to drivers/rtc
# * I/O expander drivers go to drivers/gpio
#

obj-$(CONFIG_LASER_STMVL53L0_SUPPORT)   += stmvl53l0.o \
					   vl53l0_api.o \
					   vl53l0_api_calibration.o \
					   vl53l0_api_core.o \
					   vl53l0_api_histogram.o \
					   vl53l0_api_ranging.o \
					   vl53l0_api_strings.o \
					   vl53l0_i2c.o \
					   vl53l0_platform.o \
