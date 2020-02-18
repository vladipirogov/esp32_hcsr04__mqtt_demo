/*
 * pid.c
 *
 *  Created on: 8 ����. 2020 �.
 *      Author: admin
 */
#include "../include/pid.h"

#include "stdbool.h"
#include <esp_timer.h>
#include <stdio.h>

Pid::Pid(Reference *in_reference, const Parameters *source) {

	this->reference = in_reference;
	parameters.sample_time = source->sample_time;

    inAuto = false;
    pid_set_output_limits(source->min_limit, source->max_limit);				//default output limit corresponds to						//default Controller Sample Time is 0.1 seconds
    pid_set_controller_direction(source->direction);
    pid_set_tunings(source->kp.float_val, source->ki.float_val, source->kd.float_val, source->pOn);
    last_time = esp_timer_get_time()/1000 - parameters.sample_time;
}


bool Pid::pid_compute()
{
   if(!inAuto) return false;
   unsigned long now = esp_timer_get_time()/1000;
   unsigned long timeChange = (now - last_time);
   if(timeChange>=parameters.sample_time)
   {
      /*Compute all the working error variables*/
      float input = reference->input;
      float error = reference->setpoint.float_val - input;
      float dInput = (input - last_input)/timeChange;
      output_sum+= (parameters.ki.float_val * error);
      if(output_sum > parameters.max_limit) output_sum= parameters.max_limit;
      else if(output_sum < parameters.min_limit) output_sum= parameters.min_limit;

      printf("Pid setpoint = %f\n", reference->setpoint.float_val);
      printf("TIME now: %lu\n", now);
      printf("Last time: %lu\n", last_time);
      printf("Input: %f\n", input);
      printf("Error: %f\n", error);
      printf("Diff input = %f\n", dInput);
      printf("Integral sum = %f\n", output_sum);
      printf("parameters: KP =  %f, KI = %f, KD = %f\n", parameters.kp.float_val, parameters.ki.float_val, parameters.kd.float_val);
      printf("max limit = %d, min limit = %d\n", parameters.max_limit, parameters.min_limit);
      printf("PON = %d\n", pOnE);



      /*Output result*/
	   float output = parameters.kp.float_val*error + output_sum + parameters.kd.float_val*dInput;

	    if(output >  parameters.max_limit) output =  parameters.max_limit;
      else if(output < parameters.min_limit) output = parameters.min_limit;
	    reference->output = output;

	    printf("Output: %f\n", reference->output);
	    printf("------------------");
	    printf("------------------\n");
      /*Remember some variables for next time*/
      last_input = input;
      last_time = now;
	    return true;
   }
   else return false;
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void Pid::pid_set_tunings(float Kp, float Ki, float Kd, int POn)
{
   if (Kp<0 || Ki<0 || Kd<0) return;

   parameters.pOn = POn;
   pOnE = POn == P_ON_E;

   parameters.kp.float_val = Kp;
   parameters.ki.float_val = Ki;
   parameters.kd.float_val = Kd;
   printf("parameters: KP =  %f, KI = %f, KD = %f\n", parameters.kp.float_val, parameters.ki.float_val, parameters.kd.float_val);

  if(parameters.direction ==REVERSE)
   {
	  parameters.kp.float_val = (0 - parameters.kp.float_val);
	  parameters.ki.float_val = (0 - parameters.ki.float_val);
	  parameters.kd.float_val = (0 - parameters.kd.float_val);
   }
}

/* SetTunings(...)*************************************************************
 * Set Tunings using the last-rembered POn setting
 ******************************************************************************/
void Pid::pid_set_tunings(float Kp, float Ki, float Kd){
    pid_set_tunings(Kp, Ki, Kd, parameters.pOn);
}

/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed
 ******************************************************************************/
void Pid::pid_set_sample_time(int new_sample_time)
{
   if (new_sample_time > 0)
   {
      float ratio  = (float)new_sample_time
                      / (float)parameters.sample_time;
      parameters.ki.float_val *= ratio;
      parameters.kd.float_val /= ratio;
      parameters.sample_time = (unsigned long)new_sample_time;
   }
}

/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void Pid::pid_set_output_limits(uint32_t Min, uint32_t Max)
{
   if(Min >= Max) return;
   parameters.min_limit = Min;
   parameters.max_limit = Max;

   if(inAuto)
   {
	   if(reference->output > parameters.max_limit) reference->output = parameters.max_limit;
	   else if(reference->output < parameters.min_limit) reference->output = parameters.min_limit;

	   if(output_sum > parameters.max_limit) output_sum= parameters.max_limit;
	   else if(output_sum < parameters.min_limit) output_sum= parameters.min_limit;
   }
}

/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void Pid::initialize(void)
{
   output_sum = reference->output;
   last_input = reference->input;
   if(output_sum > parameters.max_limit) output_sum = parameters.max_limit;
   else if(output_sum < parameters.min_limit) output_sum = parameters.min_limit;
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void Pid::pid_set_mode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto && !inAuto)
    {  /*we just went from manual to auto*/
        initialize();
    }
    inAuto = newAuto;
}


/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void Pid::pid_set_controller_direction(int Direction)
{
   if(inAuto && Direction !=parameters.direction)
   {
	   parameters.kp.float_val = (0 - parameters.kp.float_val);
	   parameters.ki.float_val = (0 - parameters.ki.float_val);
	   parameters.kd.float_val = (0 - parameters.kd.float_val);
   }
   parameters.direction = Direction;
}

/* Status Funcions*************************************************************
 ******************************************************************************/

int Pid::pid_get_mode(void){ return  inAuto ? AUTOMATIC : MANUAL;}

void Pid::pid_setpoint(float* setpoint) {
	reference->setpoint.float_val = *setpoint;
}