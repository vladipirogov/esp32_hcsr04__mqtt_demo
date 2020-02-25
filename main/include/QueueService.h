/*
 * QueueService.h
 *
 *  Created on: Feb 22, 2020
 *      Author: vladyslav_pyrohov
 */

#ifndef MAIN_INCLUDE_QUEUESERVICE_H_
#define MAIN_INCLUDE_QUEUESERVICE_H_

#ifdef __cplusplus
extern "C" {
#endif

#define ULTRASONIC "ultrasonic"
#define TEMPERATURE "temperature"
#define GYROSCOPE "gyroscope"

typedef struct {
	const char *type;
	const char *value;
} Tdata;


#ifdef __cplusplus
}
#endif
#endif /* MAIN_INCLUDE_QUEUESERVICE_H_ */
