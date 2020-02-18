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

	while (true) {
		xEventGroupWaitBits(mqtt_event_group, BIT1, false, true, portMAX_DELAY);
		float data = 0.0;
		xQueueReceive( xQueue, &data, pdMS_TO_TICKS( 200 ) );
		char str[10];
		sprintf(str, "%f", data);
		int msg_id = mqtt_client_publish("/topic/publish/esp2py", str);
	  /* Blink off (output low) */
	        gpio_set_level(GPIO_NUM_5, 0);
	        vTaskDelay(1000 / portTICK_PERIOD_MS);
	        /* Blink on (output high) */
	        gpio_set_level(GPIO_NUM_5, 1);
	        vTaskDelay(1000 / portTICK_PERIOD_MS);
	}

}


/**
 *
 */
void app_main(void)
{
	flash_init();
	servo_init();
	pid_init();
    wifi_init();
    mqtt_app_start(setup_pid_parameters);

    xTaskCreate(&task_initI2C, "mpu_task", 2048, NULL, 5, NULL);
    vTaskDelay(500/portTICK_PERIOD_MS);
    xTaskCreate(&task_display, "disp_task", 8192, NULL, 5, NULL);
    xTaskCreate(pid_control, "pid_control", 8192, NULL, 5, NULL);

	//xTaskCreate(ultrasonic_control, "ultrasonic_control", configMINIMAL_STACK_SIZE * 3, NULL, 24, NULL);

    xTaskCreate(blink, "blink", configMINIMAL_STACK_SIZE*5, NULL, 24, NULL);
}
