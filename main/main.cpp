/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <include/mqtt_handle.h>
#include <stdio.h>
#include <ultrasonic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "cJSON.h"
#include "include/persist.h"
#include <pid.h>

//You can get these value from the datasheet of servo you use, in general pulse width varies between 1000 to 2000 mocrosecond
#define SERVO_MIN_PULSEWIDTH 1000 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2000 //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 90 //Maximum angle in degree upto which servo can rotate
#define MAX_DISTANCE_CM 500 // 5m max
//#define TRIGGER_GPIO 17
//#define ECHO_GPIO 16
#define BLINK_GPIO CONFIG_BLINK_GPIO
#define FILTER_K 0.5

const TickType_t xBlockTime = pdMS_TO_TICKS( 200 );
QueueHandle_t xQueue = NULL;
Reference reference;
Pid *pid;

extern "C" {
	void app_main(void);
}

extern void task_initI2C(void*);
extern void task_display(void*);

/**
 *
 */
static void mcpwm_example_gpio_initialize(void)
{
    printf("initializing mcpwm servo control gpio......\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 18);    //Set GPIO 18 as PWM0A, to which servo is connected
}

/**
 * @brief Use this function to calcute pulse width for per degree rotation
 *
 * @param  degree_of_rotation the angle in degree to which servo has to rotate
 *
 * @return
 *     - calculated pulse width
 */
static uint32_t servo_per_degree_init(uint32_t degree_of_rotation)
{
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
    return cal_pulsewidth;
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

/**
 *
 */
void ultrasonic_control(void *pvParamters)
{
    ultrasonic_sensor_t sensor;
        sensor.trigger_pin = GPIO_NUM_17;
		sensor.echo_pin = GPIO_NUM_16;

    ultrasonic_init(&sensor);
    uint32_t distance = 0;

    //Initialize pid parameters
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

    uint32_t angle, count;
	//1. mcpwm gpio initialization
	mcpwm_example_gpio_initialize();

	//2. initial mcpwm configuration
	printf("Configuring Initial Parameters of mcpwm......\n");
	mcpwm_config_t pwm_config;
			pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
			pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
			pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
			pwm_config.counter_mode = MCPWM_UP_COUNTER;
			pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

	mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings

    while (true)
    {
    	distance = 0;
        esp_err_t res = ultrasonic_measure_cm(&sensor, MAX_DISTANCE_CM, &distance);
        if (res != ESP_OK)
        {
            printf("Error: ");
            switch (res)
            {
                case ESP_ERR_ULTRASONIC_PING:
                    printf("Cannot ping (device is in invalid state)\n");
                    break;
                case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
                    printf("Ping timeout (no device found)\n");
                    break;
                case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
                    printf("Echo timeout (i.e. distance too big)\n");
                    break;
                default:
                    printf("%d\n", res);
            }
        }
        else
            //printf("Distance: %d cm\n", distance);

        reference.input = reference.input * (1 - FILTER_K) + distance*FILTER_K;
        pid->pid_compute();

		if((uint32_t)reference.output > SERVO_MAX_DEGREE)
			reference.output = SERVO_MAX_DEGREE;
		//printf("Angle of rotation: %f\n", pid_output);
		angle = servo_per_degree_init((uint32_t)reference.output);
		//printf("pulse width: %dus\n", angle);
		mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle);

        xQueueSend( xQueue, &distance, xBlockTime );

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

/**
 *
 */
void blink(void *param) {

    gpio_pad_select_gpio(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    //gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

	while (true) {
		xEventGroupWaitBits(mqtt_event_group, BIT1, false, true, portMAX_DELAY);
		uint32_t DataToReceive = 0;
		xQueueReceive( xQueue, &DataToReceive, xBlockTime );
		char str[10];
		sprintf(str, "%d", DataToReceive);
		//int msg_id = esp_mqtt_client_publish(client, "/topic/publish/esp2py", str, 0, 0, 0);
	  /* Blink off (output low) */
		//printf("Turning off the LED\n");
	        //gpio_set_level(BLINK_GPIO, 0);
	        vTaskDelay(1000 / portTICK_PERIOD_MS);
	        /* Blink on (output high) */
		//printf("Turning on the LED\n");
	        //gpio_set_level(BLINK_GPIO, 1);
	        vTaskDelay(1000 / portTICK_PERIOD_MS);
	}

}


/**
 *
 */
void app_main(void)
{
	xQueue = xQueueCreate( 1, sizeof( uint32_t ) );

	flash_init();
    wifi_init();
    mqtt_app_start(setup_pid_parameters);

    xTaskCreate(&task_initI2C, "mpu_task", 2048, NULL, 5, NULL);
        vTaskDelay(500/portTICK_PERIOD_MS);
        xTaskCreate(&task_display, "disp_task", 8192, NULL, 5, NULL);

	xTaskCreate(ultrasonic_control, "ultrasonic_control", configMINIMAL_STACK_SIZE * 3, NULL, 24, NULL);

    xTaskCreate(blink, "blink", configMINIMAL_STACK_SIZE*5, NULL, 24, NULL);
}
