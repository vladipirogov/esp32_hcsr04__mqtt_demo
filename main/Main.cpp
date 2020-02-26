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
#include "cJSON.h"
#include "include/QueueService.h"

#define TOPIC "/topic/publish/esp2py"

QueueHandle_t gQueue = NULL;
EventGroupHandle_t mqtt_event_group;


extern "C" {
	void app_main(void);
}

extern void task_initI2C(void*);
extern void task_display(void*);
extern void setup_pid_parameters(char *data);


/**
 *
 */
void blink(void *param) {

    gpio_pad_select_gpio(GPIO_NUM_5);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(GPIO_NUM_5, GPIO_MODE_OUTPUT);

	while (true) {
	  /* Blink off (output low) */
	        gpio_set_level(GPIO_NUM_5, 0);
	        vTaskDelay(500 / portTICK_PERIOD_MS);
	        /* Blink on (output high) */
	        gpio_set_level(GPIO_NUM_5, 1);
	        vTaskDelay(500 / portTICK_PERIOD_MS);
	}

}

void ds18b20_task(void *param) {
	ds18b20_init(14);
	char x_data[10];
	portBASE_TYPE xStatus;
	Tdata data;

  while (true) {
		float temp = ds18b20_get_temp();
		sprintf(x_data, "%f", temp);
		data.type = TEMPERATURE;
		data.value = x_data;
		xStatus = xQueueSend((QueueHandle_t )gQueue, (void* )&data, 0);
		if (xStatus != pdPASS) {
			printf("Could not send to the queue from ds18b20.\r\n");
		}
	 printf("Temperature: %0.1f\n", temp);
	 vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}


void send_mqtt_task(void *param) {
	portBASE_TYPE gStatus;
	Tdata g_data;
	char *buffer;

	while (true) {
		xEventGroupWaitBits(mqtt_event_group, BIT1, false, true, portMAX_DELAY);
		gStatus = xQueueReceive(gQueue, &g_data, pdMS_TO_TICKS( 1000 ));
		if (gStatus == pdPASS) {
			cJSON *root = cJSON_CreateObject();
			cJSON *type = cJSON_CreateString(g_data.type);
			cJSON *value = cJSON_CreateString(g_data.value);
			cJSON_AddItemToObject(root, "type", type);
			cJSON_AddItemToObject(root, "value", value);
			buffer = cJSON_Print(root);
			mqtt_client_publish(TOPIC, buffer);
			free(buffer);
			cJSON_Delete(root);
		} else {
			printf("Could not receive from the queue.\r\n");
		}

	}
}

/**
 *
 */
void app_main(void)
{
	gQueue = xQueueCreate( 3, sizeof( Tdata ) );
	mqtt_event_group = xEventGroupCreate();

	flash_init();
    wifi_init();
    mqtt_app_start(setup_pid_parameters, mqtt_event_group);

    xTaskCreate(&task_initI2C, "mpu_task", 2048, NULL, 5, NULL);
    vTaskDelay(500/portTICK_PERIOD_MS);
    xTaskCreate(&task_display, "disp_task", 8192, ( void * )gQueue, 5, NULL);
	xTaskCreate(ultrasonic_control, "ultrasonic_control", configMINIMAL_STACK_SIZE * 3, ( void * )gQueue, 5, NULL);
    xTaskCreate(blink, "blink", configMINIMAL_STACK_SIZE*5, NULL, 5, NULL);
    xTaskCreate(ds18b20_task, "ds18b20_task", configMINIMAL_STACK_SIZE*5, NULL, 5, NULL);
    xTaskCreate(send_mqtt_task, "send_mqtt_task", configMINIMAL_STACK_SIZE*5, NULL, 10, NULL);
}
