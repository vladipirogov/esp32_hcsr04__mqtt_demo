/*
 * pid.h
 *
 *  Created on: 8 февр. 2020 г.
 *      Author: admin
 */

#ifndef COMPONENTS_INCLUDE_PID_H_
#define COMPONENTS_INCLUDE_PID_H_

#include <stdint.h>
#include "stdbool.h"
#include <esp_timer.h>
#include <stdio.h>

#define AUTOMATIC	1
#define MANUAL	0
#define DIRECT  0
#define REVERSE  1
#define REFERENCE "reference"
#define PARAM "param"

typedef union Param {
	 	uint64_t int_val;
	 	float float_val;
	 } Param;


	 typedef struct Parameters {
	 	 Param kp;
	 	 Param ki;
	 	 Param kd;
	 	 double min_limit;
	 	 double max_limit;
	 	 unsigned long sample_time;

	 	 int direction;
	 	 int pOn;
	 	} Parameters;

	 typedef struct {
	 	float input;
	 	float output;
	 	Param setpoint;
	 } Reference;

class Pid {

public:

    Pid(Reference * reference, const Parameters *source);

    void pid_set_mode(int Mode);

    bool pid_compute();

    void pid_set_output_limits(uint32_t, uint32_t);

    void pid_set_tunings(float, float, float);

	void pid_set_controller_direction(int);

    void pid_set_sample_time(int);

    void pid_setpoint(float* setpoint);

    int pid_get_mode(void);

private:

	 unsigned long last_time;
	 float output_sum, last_input;
	 bool in_auto;
	 Parameters parameters;
	 Reference *reference;

	 void initialize(void);

};

#endif /* COMPONENTS_INCLUDE_PID_H_ */
