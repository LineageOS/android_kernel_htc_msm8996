/*
 * Copyright (C) 2015 HTC, Inc.
 * Author: Dyson Lee <Dyson@intel.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

static bool bundle_headset = false;

#define REQUEST_RESET_DELAYED (HZ / 10) /* 100 ms */ /*++ 2015/07/07 USB Team, PCN00010 ++*/

bool is_bundle_headset(void)
{
	return bundle_headset;
}
EXPORT_SYMBOL_GPL(is_bundle_headset);

bool is_autosuspend(const u16 idVendor, const u16 idProduct, int on)
{
	bool match_vpid = false;
	if (idVendor == 0x170D && idProduct == 0x0523) // Avnera test headset
		match_vpid = true;
	else if (idVendor == 0x0BDA && idProduct == 0x4805) //Lifebeam headset
		match_vpid = true;
	else if (idVendor == 0x0ECB && idProduct == 0x1ECB) //JBL headset
		match_vpid = true;
	else
		match_vpid = false;
	//FIXME: need other patch to unlock ohio lock
	//if (match_vpid == true && on == 1)
	//	manual_unlock_ohio();
	if (match_vpid && on)
		bundle_headset = true;
	else
		bundle_headset = false;

	return match_vpid;
}
EXPORT_SYMBOL_GPL(is_autosuspend);
