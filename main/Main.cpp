/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <include/MqttService.h>
#include <include/PersistService.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_system.h"
#include "include/ServoService.h"
#include "include/UltrasonicService.h"
#include <ds18b20.h>

#define PITCH_TOPIC "/topic/publish/esp2py"
#define ULTRASONIC_TOPIC "/topic/publish/ultrasonic"
#define TEMPERATURE_TOPIC "/topic/publish/temperature"

QueueHandle_t xQueue = NULL;
QueueHandle_t uQueue = NULL;
EventGroupHandle_t mqtt_event_group;


extern "C" {
	void app_main(void);
}

extern void task_initI2C(void*);
extern void task_display(void*);
extern void setup_pid_parameters(char *data);
extern void pid_init(void);
extern void pid_control(void*);


/**
 *
 */
void blink(void *param) {

    gpio_pad_select_gpio(GPIO_NUM_5);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(GPIO_NUM_5, GPIO_MODE_OUTPUT);
    float angle = 0.0;
    uint32_t distance = 0;
    char angle_str[10], distance_str[10];

	while (true) {
		angle = 0.0;
		distance = 0;
		xEventGroupWaitBits(mqtt_event_group, BIT1, false, true, portMAX_DELAY);
		xQueueReceive( xQueue, &angle, pdMS_TO_TICKS( 100 ) );
		sprintf(angle_str, "%f", angle);
		mqtt_client_publish(PITCH_TOPIC, angle_str);

		xQueueReceive( uQueue, &distance, pdMS_TO_TICKS( 100 ) );
		sprintf(distance_str, "%d", distance);
		mqtt_client_publish(ULTRASONIC_TOPIC, distance_str);
	  /* Blink off (output low) */
	        gpio_set_level(GPIO_NUM_5, 0);
	        vTaskDelay(500 / portTICK_PERIOD_MS);
	        /* Blink on (output high) */
	        gpio_set_level(GPIO_NUM_5, 1);
	        vTaskDelay(500 / portTICK_PERIOD_MS);
	}

}

void ds18b20_task(void *pvParameters){
	ds18b20_init(14);
	char str[10];
  while (true) {
	 float temp = ds18b20_get_temp();
	 sprintf(str, "%f", temp);
	 mqtt_client_publish(TEMPERATURE_TOPIC, str);
    printf("Temperature: %0.1f\n", temp);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}


/**
 *
 */
void app_main(void)
{
	xQueue = xQueueCreate( 2, sizeof( uint32_t ) );
	uQueue = xQueueCreate( 1, sizeof( uint32_t ) );
	mqtt_event_group = xEventGroupCreate();

	flash_init();
	servo_init();
	pid_init();
    wifi_init();
    mqtt_app_start(setup_pid_parameters, mqtt_event_group);

    xTaskCreate(&task_initI2C, "mpu_task", 2048, NULL, 5, NULL);
    vTaskDelay(500/portTICK_PERIOD_MS);
    xTaskCreate(&task_display, "disp_task", 8192, ( void * )xQueue, 5, NULL);
    xTaskCreate(pid_control, "pid_control", 8192, ( void * )xQueue, 5, NULL);

	xTaskCreate(ultrasonic_control, "ultrasonic_control", configMINIMAL_STACK_SIZE * 3, ( void * )uQueue, 5, NULL);

    xTaskCreate(blink, "blink", configMINIMAL_STACK_SIZE*5, NULL, 5, NULL);

    xTaskCreate(ds18b20_task, "ds18b20_task", configMINIMAL_STACK_SIZE*5, NULL, 5, NULL);
}
