/*
 * flash.h
 *
 *  Created on: Feb 14, 2020
 *      Author: Vladyslav_Pyrohov
 */

#ifndef MAIN_INCLUDE_PERSISTSERVICE_H_
#define MAIN_INCLUDE_PERSISTSERVICE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "nvs_flash.h"
#include "nvs.h"


void flash_init(void);

void read_param_from_flash(const char* key, uint64_t* out_value);

void write_param_to_flash(const char* key, uint64_t value);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_INCLUDE_PERSISTSERVICE_H_ */
