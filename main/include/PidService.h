/*
 * PidService.h
 *
 *  Created on: Feb 22, 2020
 *      Author: vladyslav_pyrohov
 */

#ifndef MAIN_INCLUDE_PIDSERVICE_H_
#define MAIN_INCLUDE_PIDSERVICE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include "string.h"
#include "cJSON.h"
#include <pid.h>
#include "include/PersistService.h"

void pid_init(void);

void setup_parameters(char *data);

float pid_control(float data);

#ifdef __cplusplus
}
#endif
#endif /* MAIN_INCLUDE_PIDSERVICE_H_ */
