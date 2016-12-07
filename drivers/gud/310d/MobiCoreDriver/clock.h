/*
 * Copyright (c) 2013-2015 TRUSTONIC LIMITED
 * All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef _MC_CLOCK_H_
#define _MC_CLOCK_H_

#include "platform.h"	

#ifdef MC_CRYPTO_CLOCK_MANAGEMENT

int mc_clock_init(void);
void mc_clock_exit(void);
int mc_clock_enable(void);
void mc_clock_disable(void);

#else 

static inline int mc_clock_init(void)
{
	return 0;
}

static inline void mc_clock_exit(void)
{
}

static inline int mc_clock_enable(void)
{
	return 0;
}

static inline void mc_clock_disable(void)
{
}

#endif 

#endif 
