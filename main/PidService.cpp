#include "include/PidService.h"

Reference reference;
Pid *pid;

void pid_init(void) {
	//Initialize pid parameters
	printf("Initialization PID\n");
	    Parameters parameters;
	    	parameters.min_limit = -180.0;
			parameters.max_limit = 180.0;
			parameters.sample_time = 100;
			parameters.direction = DIRECT;

	    read_param_from_flash("kp", &parameters.kp.int_val);
	    read_param_from_flash("ki", &parameters.ki.int_val);
	    read_param_from_flash("kd", &parameters.kd.int_val);
	    read_param_from_flash("setpoint", &reference.setpoint.int_val);

	    //Initialize pid control
	    pid = new Pid(&reference, &parameters);

	    pid->pid_set_mode(AUTOMATIC);
	    printf("PID was initialized\n");
}

/**
 * {"kp": 1, "ki":0, "kd":0}
 */
void setup_parameters(char *data) {
	 //printf("Setup pid parameters: %.*s\r\n", data);
	cJSON *root = cJSON_Parse(data);
	cJSON *action = cJSON_GetObjectItem(root,"action");
	if(action == NULL) {
		printf("Action undefined!");
		return;
	}
	printf("Action: %s\r\n", action->valuestring);
	if(strcmp(action->valuestring, PARAM) == 0) {
		cJSON *kp_json = cJSON_GetObjectItem(root,"kp");
		cJSON *ki_json = cJSON_GetObjectItem(root,"ki");
		cJSON *kd_json = cJSON_GetObjectItem(root,"kd");
		if(ki_json != NULL && ki_json != NULL && kd_json != NULL) {
			Param kp_param;
			kp_param.float_val = (float)kp_json->valuedouble;
			Param ki_param;
			ki_param.float_val = (float)ki_json->valuedouble;
			Param kd_param;
			kd_param.float_val = (float)kd_json->valuedouble;

			printf("MQTT set parameters KP: %f, KI: %f, KD: %f\n", kp_param.float_val, ki_param.float_val, kd_param.float_val);
			pid->pid_set_tunings(kp_param.float_val, ki_param.float_val, kd_param.float_val);

			write_param_to_flash("kp", kp_param.int_val);
			write_param_to_flash("ki", ki_param.int_val);
			write_param_to_flash("kd", kd_param.int_val);
		}
	}
	else if(strcmp(action->valuestring, REFERENCE) == 0) {
		cJSON *setpoint_json = cJSON_GetObjectItem(root,"setpoint");
		Param setpoint;
		setpoint.float_val = (float)setpoint_json->valuedouble;
		printf("MQTT pid setpoint = %f\n", setpoint.float_val);
		pid->pid_setpoint(&setpoint.float_val);
		write_param_to_flash("setpoint", reference.setpoint.int_val);
	}
	else if(strcmp(action->valuestring, MODE) == 0) {
		cJSON *mode_json = cJSON_GetObjectItem(root,"mode");
		pid->pid_set_mode(mode_json->valueint);
	}
	else {
		printf("Can not setup pid values!");
	}
	cJSON_Delete(root);
}

float map_val(float val, float I_Min, float I_Max, float O_Min, float O_Max){
    return(val/(I_Max-I_Min)*(O_Max-O_Min) + O_Min);
}

float pid_control(float data) {
	reference.input = data;
	pid->pid_compute();
	return  reference.output > 0 ? reference.output : 90 + reference.output ;
}
