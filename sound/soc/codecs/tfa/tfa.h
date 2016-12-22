/*
 *Copyright 2015 NXP Semiconductors
 *
 *Licensed under the Apache License, Version 2.0 (the "License");
 *you may not use this file except in compliance with the License.
 *You may obtain a copy of the License at
 *
 *http://www.apache.org/licenses/LICENSE-2.0
 *
 *Unless required by applicable law or agreed to in writing, software
 *distributed under the License is distributed on an "AS IS" BASIS,
 *WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *See the License for the specific language governing permissions and
 *limitations under the License.
 */

#ifndef TFA_H_
#define TFA_H_

#define TFA_MAX_CNT_LENGTH (256*1024)

enum tfa_error {
	tfa_error_ok, 
	tfa_error_device, 
	tfa_error_bad_param, 
	tfa_error_noclock, 
	tfa_error_timeout, 
	tfa_error_dsp, 
	tfa_error_container, 
	tfa_error_max 
};

enum tfa_error tfa_load_cnt(void *cnt, int length);

enum Tfa98xx_Error
tfa_probe(unsigned char slave_address, int *pDevice);

enum tfa_error tfa_start(int profile, int *vstep);

enum tfa_error tfa_stop(void);

void tfa_deinit(void);

enum tfa_error tfa_reset(void);

enum Tfa98xx_Error tfa_write_filters(int dev_idx, int prof_idx);

#endif 
