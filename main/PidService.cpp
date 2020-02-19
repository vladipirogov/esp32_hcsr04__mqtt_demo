#include <stdio.h>
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "cJSON.h"
#include <pid.h>
#include "include/PersistService.h"
#include "freertos/queue.h"
#include "include/MqttService.h"
#include "include/ServoService.h"


Reference reference;
Pid *pid;

void pid_init(void) {
	//Initialize pid parameters
	printf("Initialization PID\n");
	    Parameters parameters;
	    	parameters.min_limit = 0;
			parameters.max_limit = 90;
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
 * {"kp": 20, "ki":0, "kd":0}
 */
void setup_pid_parameters(char *data) {
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
	else {
		printf("Can not setup pid values!");
	}
}

void pid_control(void* xQueue) {
while (true) {
	float data = 0.0;
	xQueueReceive( (QueueHandle_t)xQueue, &data, pdMS_TO_TICKS( 200 ) );
	reference.input = data;
	pid->pid_compute();
	uint32_t angle = reference.output;
	servo_control(angle);
	vTaskDelay(100 / portTICK_PERIOD_MS);
}
}
