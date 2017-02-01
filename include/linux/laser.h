/*
 * Driver model for laser
 *
 * Copyright (C) 2013 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __LINUX_LASER_H_INCLUDED
#define __LINUX_LASER_H_INCLUDED

int Laser_poweron_by_camera(void);
int Laser_poweroff_by_camera(void);

#endif          /* __LINUX_LASER_H_INCLUDED */
