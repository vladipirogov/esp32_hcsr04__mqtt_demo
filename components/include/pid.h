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
	 	 Param kp;                  // * (P)roportional Tuning Parameter
	 	 Param ki;                  // * (I)ntegral Tuning Parameter
	 	 Param kd;                  // * (D)erivative Tuning Parameter
	 	 uint32_t min_limit;
	 	 uint32_t max_limit;
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

  //commonly used functions **************************************************************************
   Pid(Reference * reference, const Parameters *source);//   Setpoint.  Initial tuning parameters are also set here.
                                          //   (overload for specifying proportional mode)

    void pid_set_mode(int Mode);               // * sets PID to either Manual (0) or Auto (non-0)

    bool pid_compute();                       // * performs the PID calculation.  it should be
                                          //   called every time loop() cycles. ON/OFF and
                                          //   calculation frequency can be set using SetMode
                                          //   SetSampleTime respectively

    void pid_set_output_limits(uint32_t, uint32_t); // * clamps the output to a specific range. 0-255 by default, but
										                      //   it's likely the user will want to change this depending on
										                      //   the application

  //available but not commonly used functions ********************************************************
    void pid_set_tunings(float, float,       // * While most users will set the tunings once in the
                    float);         	    //   constructor, this function gives the user the option
                                          //   of changing tunings during runtime for Adaptive control

	void pid_set_controller_direction(int);	  // * Sets the Direction, or "Action" of the controller. DIRECT
										  //   means the output will increase when error is positive. REVERSE
										  //   means the opposite.  it's very unlikely that this will be needed
										  //   once it is set in the constructor.
    void pid_set_sample_time(int);              // * sets the frequency, in Milliseconds, with which
                                          //   the PID calculation is performed.  default is 100

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
