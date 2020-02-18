/*
 * ServoService.h
 *
 *  Created on: Feb 18, 2020
 *      Author: Vladyslav_Pyrohov
 */

#ifndef MAIN_INCLUDE_SERVOSERVICE_H_
#define MAIN_INCLUDE_SERVOSERVICE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_system.h"

void mcpwm_example_gpio_initialize(void);

void servo_init(void);

void servo_control(uint32_t reference);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_INCLUDE_SERVOSERVICE_H_ */
