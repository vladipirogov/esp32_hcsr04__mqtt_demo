/*
 * pid.c
 *
 *  Created on: 8 февр. 2020 г.
 *      Author: admin
 */
#include <pid.h>

Pid::Pid(Reference *in_reference, const Parameters *source) {

	this->reference = in_reference;
	this->parameters.max_limit = source->max_limit;
	this->parameters.min_limit = source->min_limit;
	parameters.sample_time = source->sample_time;

    this->in_auto = false;
    pid_set_controller_direction(source->direction);
    pid_set_tunings(source->kp.float_val, source->ki.float_val, source->kd.float_val);
    last_time = esp_timer_get_time()/1000 - parameters.sample_time;
}


bool Pid::pid_compute() {
   if(!this->in_auto) return false;

   unsigned long now = esp_timer_get_time()/1000;
   unsigned long time_change = (now - last_time);

   if(time_change>=parameters.sample_time) {

      float input = reference->input;
      float error = reference->setpoint.float_val - input;
      float dInput = (input - last_input)/time_change;
      output_sum+= (parameters.ki.float_val * error);
      if(output_sum > parameters.max_limit) output_sum= parameters.max_limit;
      else if(output_sum < parameters.min_limit) output_sum= parameters.min_limit;

      printf("Pid setpoint = %f\n", reference->setpoint.float_val);
      printf("Input: %f\n", input);
      printf("Error: %f\n", error);
      printf("Diff input = %f\n", dInput);
      printf("Integral sum = %f\n", output_sum);
      printf("parameters: KP =  %f, KI = %f, KD = %f\n", parameters.kp.float_val, parameters.ki.float_val, parameters.kd.float_val);
      printf("max limit = %f, min limit = %f\n", parameters.max_limit, parameters.min_limit);
      printf("PID mode is %d\n", in_auto);

	  float output = parameters.kp.float_val*error + output_sum + parameters.kd.float_val*dInput;

	  if(output >  parameters.max_limit) output =  parameters.max_limit;
      else if(output < parameters.min_limit) output = parameters.min_limit;
	    reference->output = output;

		printf("Output: %f\n", reference->output);
		printf("------------------");
		printf("------------------\n");

      last_input = input;
      last_time = now;
	    return true;
   }
   else return false;
}


void Pid::pid_set_tunings(float kp, float ki, float kd) {
   if (kp<0 || ki<0 || kd<0) return;

   parameters.kp.float_val = kp;
   parameters.ki.float_val = ki;
   parameters.kd.float_val = kd;
   printf("parameters: KP =  %f, KI = %f, KD = %f\n", parameters.kp.float_val, parameters.ki.float_val, parameters.kd.float_val);

  if(parameters.direction == REVERSE)
   {
	  parameters.kp.float_val = (0 - parameters.kp.float_val);
	  parameters.ki.float_val = (0 - parameters.ki.float_val);
	  parameters.kd.float_val = (0 - parameters.kd.float_val);
   }
}


void Pid::pid_set_sample_time(int new_sample_time) {
   if (new_sample_time > 0)
   {
      float ratio  = (float)new_sample_time
                      / (float)parameters.sample_time;
      parameters.ki.float_val *= ratio;
      parameters.kd.float_val /= ratio;
      parameters.sample_time = (unsigned long)new_sample_time;
   }
}


void Pid::initialize(void) {
   output_sum = reference->output;
   last_input = reference->input;
   if(output_sum > parameters.max_limit) output_sum = parameters.max_limit;
   else if(output_sum < parameters.min_limit) output_sum = parameters.min_limit;
}


void Pid::pid_set_mode(int mode) {
    bool in_auto = (mode == AUTOMATIC);
    if(in_auto && !this->in_auto) {
        initialize();
    }
    this->in_auto = in_auto;
}



void Pid::pid_set_controller_direction(int Direction) {
   if(this->in_auto && Direction !=parameters.direction)
   {
	   parameters.kp.float_val = (0 - parameters.kp.float_val);
	   parameters.ki.float_val = (0 - parameters.ki.float_val);
	   parameters.kd.float_val = (0 - parameters.kd.float_val);
   }
   parameters.direction = Direction;
}



int Pid::pid_get_mode(void) {
	return  this->in_auto ? AUTOMATIC : MANUAL;
}

void Pid::pid_setpoint(float* setpoint) {
	reference->setpoint.float_val = *setpoint;
}
