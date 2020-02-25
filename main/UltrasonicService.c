#include <esp_log.h>
#include <esp_err.h>
#include "esp_system.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "sdkconfig.h"
#include <ultrasonic.h>
#include "freertos/queue.h"
#include "include/QueueService.h"

#define MAX_DISTANCE_CM 500 // 5m max

void ultrasonic_control(void *xQueue)
{
    ultrasonic_sensor_t sensor;
        sensor.trigger_pin = GPIO_NUM_17;
		sensor.echo_pin = GPIO_NUM_16;

    ultrasonic_init(&sensor);
    uint32_t distance = 0;
	char x_data[sizeof(int32_t)];
	portBASE_TYPE xStatus;
	Tdata data;

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
        else {
            //printf("Distance: %d cm\n", distance);
			itoa(distance, x_data, 10);
			data.type = ULTRASONIC;
			data.value = x_data;
            xStatus = xQueueSend( (QueueHandle_t)xQueue, (void *)&data, 0);
			if (xStatus != pdPASS) {
				printf("Could not send to the queue from ultrasonic.\r\n");
			}
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
