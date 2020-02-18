#include <esp_log.h>
#include <esp_err.h>
#include "esp_system.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "sdkconfig.h"
#include <ultrasonic.h>

#define MAX_DISTANCE_CM 500 // 5m max

void ultrasonic_control(void *data)
{
    ultrasonic_sensor_t sensor;
        sensor.trigger_pin = GPIO_NUM_17;
		sensor.echo_pin = GPIO_NUM_16;

    ultrasonic_init(&sensor);
    uint32_t distance = 0;

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
            printf("Distance: %d cm\n", distance);

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
