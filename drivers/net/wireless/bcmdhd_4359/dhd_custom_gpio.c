/*
 * Customer code to add GPIO control during WLAN start/stop
 *
 * Copyright (C) 1999-2016, Broadcom Corporation
 * 
 *      Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2 (the "GPL"),
 * available at http://www.broadcom.com/licenses/GPLv2.php, with the
 * following added to such license:
 * 
 *      As a special exception, the copyright holders of this software give you
 * permission to link this software with independent modules, and to copy and
 * distribute the resulting executable under terms of your choice, provided that
 * you also meet, for each linked independent module, the terms and conditions of
 * the license of that module.  An independent module is a module which is not
 * derived from this software.  The special exception does not apply to any
 * modifications of the software.
 * 
 *      Notwithstanding the above, under no circumstances may you combine this
 * software in any way with any other Broadcom software provided under a license
 * other than the GPL, without Broadcom's express prior written consent.
 *
 *
 * <<Broadcom-WL-IPTag/Open:>>
 *
 * $Id: dhd_custom_gpio.c 614447 2016-01-22 08:07:50Z $
 */

#include <typedefs.h>
#include <linuxver.h>
#include <osl.h>
#include <bcmutils.h>
#include <dngl_stats.h>
#include <dhd.h>
#include <dhd_linux.h>

#include <wlioctl.h>
#if defined(WL_WIRELESS_EXT)
#include <wl_iw.h>
#endif

#define WL_ERROR(x) printf x
#define WL_TRACE(x)

#if defined(CUSTOMER_HW2)


#endif 

#if defined(OOB_INTR_ONLY)

#if defined(BCMLXSDMMC)
extern int sdioh_mmc_irq(int irq);
#endif 


static int dhd_oob_gpio_num = -1;

module_param(dhd_oob_gpio_num, int, 0644);
MODULE_PARM_DESC(dhd_oob_gpio_num, "DHD oob gpio number");

int dhd_customer_oob_irq_map(void *adapter, unsigned long *irq_flags_ptr)
{
	int  host_oob_irq = 0;

#if defined(CUSTOMER_HW2)
	host_oob_irq = wifi_platform_get_irq_number(adapter, irq_flags_ptr);

#else
#if defined(CUSTOM_OOB_GPIO_NUM)
	if (dhd_oob_gpio_num < 0) {
		dhd_oob_gpio_num = CUSTOM_OOB_GPIO_NUM;
	}
#endif 

	if (dhd_oob_gpio_num < 0) {
		WL_ERROR(("%s: ERROR customer specific Host GPIO is NOT defined \n",
		__FUNCTION__));
		return (dhd_oob_gpio_num);
	}

	WL_ERROR(("%s: customer specific Host GPIO number is (%d)\n",
	         __FUNCTION__, dhd_oob_gpio_num));

#endif 

	return (host_oob_irq);
}
#endif 

int
dhd_customer_gpio_wlan_ctrl(void *adapter, int onoff)
{
	int err = 0;

	return err;
}

#ifdef GET_CUSTOM_MAC_ENABLE
int
dhd_custom_get_mac_address(void *adapter, unsigned char *buf)
{
	int ret = 0;

	WL_TRACE(("%s Enter\n", __FUNCTION__));
	if (!buf)
		return -EINVAL;

	
#if (defined(CUSTOMER_HW2) || 0) && (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35))
	ret = wifi_platform_get_mac_addr(adapter, buf);
#endif

#ifdef EXAMPLE_GET_MAC
	
	{
		struct ether_addr ea_example = {{0x00, 0x11, 0x22, 0x33, 0x44, 0xFF}};
		bcopy((char *)&ea_example, buf, sizeof(struct ether_addr));
	}
#endif 

	return ret;
}
#endif 

#if !defined(WL_WIRELESS_EXT)
struct cntry_locales_custom {
	char iso_abbrev[WLC_CNTRY_BUF_SZ];	
	char custom_locale[WLC_CNTRY_BUF_SZ];	
	int32 custom_locale_rev;		
};
#endif 

const struct cntry_locales_custom translate_custom_table[] = {
#ifdef EXAMPLE_TABLE
	{"",   "XY", 4},  
	{"US", "US", 69}, 
	{"CA", "US", 69}, 
	{"EU", "EU", 5},  
	{"AT", "EU", 5},
	{"BE", "EU", 5},
	{"BG", "EU", 5},
	{"CY", "EU", 5},
	{"CZ", "EU", 5},
	{"DK", "EU", 5},
	{"EE", "EU", 5},
	{"FI", "EU", 5},
	{"FR", "EU", 5},
	{"DE", "EU", 5},
	{"GR", "EU", 5},
	{"HU", "EU", 5},
	{"IE", "EU", 5},
	{"IT", "EU", 5},
	{"LV", "EU", 5},
	{"LI", "EU", 5},
	{"LT", "EU", 5},
	{"LU", "EU", 5},
	{"MT", "EU", 5},
	{"NL", "EU", 5},
	{"PL", "EU", 5},
	{"PT", "EU", 5},
	{"RO", "EU", 5},
	{"SK", "EU", 5},
	{"SI", "EU", 5},
	{"ES", "EU", 5},
	{"SE", "EU", 5},
	{"GB", "EU", 5},
	{"KR", "XY", 3},
	{"AU", "XY", 3},
	{"CN", "XY", 3}, 
	{"TW", "XY", 3},
	{"AR", "XY", 3},
	{"MX", "XY", 3},
	{"IL", "IL", 0},
	{"CH", "CH", 0},
	{"TR", "TR", 0},
	{"NO", "NO", 0},
#endif 
#if defined(CUSTOMER_HW2) && !defined(CUSTOMER_HW_ONE)
#if defined(BCM4335_CHIP)
	{"",   "XZ", 11},  
#endif
	{"AE", "AE", 1},
	{"AR", "AR", 1},
	{"AT", "AT", 1},
	{"AU", "AU", 2},
	{"BE", "BE", 1},
	{"BG", "BG", 1},
	{"BN", "BN", 1},
	{"CA", "CA", 2},
	{"CH", "CH", 1},
	{"CY", "CY", 1},
	{"CZ", "CZ", 1},
	{"DE", "DE", 3},
	{"DK", "DK", 1},
	{"EE", "EE", 1},
	{"ES", "ES", 1},
	{"FI", "FI", 1},
	{"FR", "FR", 1},
	{"GB", "GB", 1},
	{"GR", "GR", 1},
	{"HR", "HR", 1},
	{"HU", "HU", 1},
	{"IE", "IE", 1},
	{"IS", "IS", 1},
	{"IT", "IT", 1},
	{"ID", "ID", 1},
	{"JP", "JP", 8},
	{"KR", "KR", 24},
	{"KW", "KW", 1},
	{"LI", "LI", 1},
	{"LT", "LT", 1},
	{"LU", "LU", 1},
	{"LV", "LV", 1},
	{"MA", "MA", 1},
	{"MT", "MT", 1},
	{"MX", "MX", 1},
	{"NL", "NL", 1},
	{"NO", "NO", 1},
	{"PL", "PL", 1},
	{"PT", "PT", 1},
	{"PY", "PY", 1},
	{"RO", "RO", 1},
	{"SE", "SE", 1},
	{"SI", "SI", 1},
	{"SK", "SK", 1},
	{"TR", "TR", 7},
	{"TW", "TW", 1},
	{"IR", "XZ", 11},	
	{"SD", "XZ", 11},	
	{"SY", "XZ", 11},	
	{"GL", "XZ", 11},	
	{"PS", "XZ", 11},	
	{"TL", "XZ", 11},	
	{"MH", "XZ", 11},	
#ifdef BCM4330_CHIP
	{"RU", "RU", 1},
	{"US", "US", 5}
#endif
#elif defined(CUSTOMER_HW_ONE)
	{"",   "XZ", 11}, 
	{"AM", "AM", 1},
	{"AR", "AR", 21},
	{"AU", "AU", 6},
	{"BG", "BG", 4},
	{"BH", "BH", 4},
	{"BR", "BR", 17},
	{"CA", "CA", 31},
	{"CL", "CL", 0},
	{"CN", "CN", 38},
	{"CO", "CO", 17},
	{"DE", "DE", 7},
	{"DZ", "XZ", 11},
	{"EG", "EG", 13},
	{"ES", "ES", 4},
	{"FR", "FR", 5},
	{"HK", "HK", 2},
	{"ID", "ID", 5},
	{"IL", "IL", 7},
	{"IN", "IN", 3},
	{"IQ", "IQ", 0},
	{"JO", "JO", 3},
	{"JP", "JP", 45},
	{"KR", "KR", 4},
	{"KW", "KW", 5},
	{"KZ", "KZ", 0},
	{"LB", "LB", 3},
	{"LY", "XZ", 11},
	{"MA", "IL", 7},
	{"MM", "MM", 0},
	{"MY", "MY", 19},
	{"MX", "MX", 20},
	{"NZ", "NZ", 4},
	{"OM", "OM", 4},
	{"PE", "PE", 20},
	{"PH", "PH", 5},
	{"PR", "PR", 20},
	{"QA", "QA", 0},
	{"RS", "RS", 2},
	{"RU", "KZ", 0},
	{"SA", "SA", 0},
	{"SG", "SG", 12},
	{"TH", "TH", 5},
	{"TN", "TN", 0},
	{"TR", "TR", 7},
	{"TW", "TW", 65},
	{"UA", "UA", 6},
	{"US", "US", 0},
	{"VE", "VE", 3},
	{"VN", "VN", 4},
	{"YE", "YE", 0},
	{"ZA", "ZA", 6},
#endif
};

#ifdef CUSTOMER_HW_ONE
extern dhd_pub_t *priv_dhdp;
const struct cntry_locales_custom translate_custom_hotspot_table[] = {
	{"",   "IL", 7}, 
	{"AM", "AM", 1},
	{"AR", "AR", 23},
	{"AU", "AU", 4},
	{"BG", "BG", 3},
	{"BH", "BH", 4},
	{"BO", "BO", 5},
	{"BR", "BR", 2},
	{"CA", "CA", 31},
	{"CL", "CL", 0},
	{"CN", "CN", 38},
	{"CO", "CO", 17},
	{"DE", "DE", 6},
	{"DZ", "IL", 7},
	{"EG", "EG", 2},
	{"ES", "ES", 3},
	{"FR", "FR", 3},
	{"GY", "GY", 205},
	{"HK", "HK", 1},
	{"ID", "ID", 11},
	{"IL", "IL", 5},
	{"IN", "IN", 2},
	{"IQ", "IQ", 0},
	{"JM", "JM", 205},
	{"JO", "JO", 4},
	{"JP", "JP", 45},
	{"KR", "KR", 4},
	{"KW", "KW", 5},
	{"KZ", "KZ", 212},
	{"LB", "LB", 3},
	{"LY", "IL", 7},
	{"MA", "MA", 2},
	{"MM", "MM", 5},
	{"MY", "MY", 19},
	{"MX", "MX", 24},
	{"NG", "NG", 205},
	{"NZ", "NZ", 2},
	{"OM", "OM", 4},
	{"PE", "PE", 20},
	{"PH", "PH", 3},
	{"PK", "PK", 2},
	{"PR", "PR", 20},
	{"QA", "QA", 0},
	{"RS", "RS", 2},
	{"RU", "IL", 7},
	{"SA", "SA", 0},
	{"SG", "SG", 11},
	{"TH", "TH", 3},
	{"TN", "TN", 0},
	{"TR", "TR", 7},
	{"TW", "TW", 65},
	{"UA", "UA", 12},
	{"US", "US", 111},
	{"VE", "VE", 2},
	{"VN", "VN", 2},
	{"YE", "YE", 0},
	{"ZA", "ZA", 3},
};
#endif 

#ifdef CUSTOM_COUNTRY_CODE
void get_customized_country_code(void *adapter, char *country_iso_code,
  wl_country_t *cspec, u32 flags)
#else
void get_customized_country_code(void *adapter, char *country_iso_code, wl_country_t *cspec)
#endif 
{
#if defined(CUSTOMER_HW2) && (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 39)) && \
	!defined(CUSTOMER_HW_ONE)

	struct cntry_locales_custom *cloc_ptr;

	if (!cspec)
		return;
#ifdef CUSTOM_COUNTRY_CODE
	cloc_ptr = wifi_platform_get_country_code(adapter, country_iso_code,
	           flags);
#else
	cloc_ptr = wifi_platform_get_country_code(adapter, country_iso_code);
#endif 
	if (cloc_ptr) {
		strlcpy(cspec->ccode, cloc_ptr->custom_locale, WLC_CNTRY_BUF_SZ);
		cspec->rev = cloc_ptr->custom_locale_rev;
	}
	return;
#else
	int size, i;
#ifdef CUSTOMER_HW_ONE
	const struct cntry_locales_custom *cloc_ptr;
	printf("%s op_mode = %d\n", __FUNCTION__, priv_dhdp->op_mode);
	if (priv_dhdp->op_mode == DHD_FLAG_HOSTAP_MODE) {
		size = ARRAYSIZE(translate_custom_hotspot_table);
		cloc_ptr = translate_custom_hotspot_table;
	} else {
		size = ARRAYSIZE(translate_custom_table);
		cloc_ptr = translate_custom_table;
	}
#else
	size = ARRAYSIZE(translate_custom_table);
#endif 
	if (cspec == 0)
		 return;

	if (size == 0)
		 return;
#ifdef CUSTOMER_HW_ONE
	for (i = 0; i < size; i++) {
		if (strcmp(country_iso_code, cloc_ptr[i].iso_abbrev) == 0) {
			memcpy(cspec->ccode,
				cloc_ptr[i].custom_locale, WLC_CNTRY_BUF_SZ);
			cspec->rev = cloc_ptr[i].custom_locale_rev;
			return;
		}
	}

	
	memcpy(cspec->ccode, cloc_ptr[0].custom_locale, WLC_CNTRY_BUF_SZ);
	cspec->rev = cloc_ptr[0].custom_locale_rev;
#else
	for (i = 0; i < size; i++) {
		if (strcmp(country_iso_code, translate_custom_table[i].iso_abbrev) == 0) {
			memcpy(cspec->ccode,
				translate_custom_table[i].custom_locale, WLC_CNTRY_BUF_SZ);
			cspec->rev = translate_custom_table[i].custom_locale_rev;
			return;
		}
	}
#endif 

#if defined(EXAMPLE_TABLE)
	
	memcpy(cspec->ccode, translate_custom_table[0].custom_locale, WLC_CNTRY_BUF_SZ);
	cspec->rev = translate_custom_table[0].custom_locale_rev;
#endif 
	return;
#endif 
}
