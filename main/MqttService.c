/*
 * mqtt_handle.c
 *
 *  Created on: Feb 13, 2020
 *      Author: Vladyslav_Pyrohov
 */
#include <include/MqttService.h>
#include "esp_err.h"

#define CONFIG_WIFI_SSID "XiaomyAP"
#define CONFIG_WIFI_PASSWORD ""

static char *TAG = "MQTTS_SAMPLE";
static void(*setup_pid_parameters)(char *);
static EventGroupHandle_t wifi_event_group;
static esp_mqtt_client_handle_t client = NULL;
static EventGroupHandle_t mqtt_event_group = NULL;

void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    switch (event_id) {
        case WIFI_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            esp_wifi_connect();
            xEventGroupClearBits(wifi_event_group, BIT0);
            break;
        default:
            break;
    }
    return;
}

/**
 *
 */
void ip_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    switch (event_id) {
        case IP_EVENT_STA_GOT_IP:
            xEventGroupSetBits(wifi_event_group, BIT0);

            break;
        default:
            break;
    }
    return;
}

/**
 *
 */
void wifi_init(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_LOGI(TAG, "start the WIFI SSID:[%s] password:[%s]", CONFIG_WIFI_SSID, "******");
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "Waiting for wifi");
    xEventGroupWaitBits(wifi_event_group, BIT0, false, true, portMAX_DELAY);
}

/**
 *
 */
esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    // your_context_t *context = event.context;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
        	 xEventGroupSetBits(mqtt_event_group, BIT1);
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            msg_id = esp_mqtt_client_subscribe(client, "/topic/subscribe/py2esp", 0);
            ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            msg_id = esp_mqtt_client_publish(client, "/topic/subscribe/py2esp", "data", 0, 0, 0);
            ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);
            //setup_pid_parameters(event->data);
            setup_pid_parameters(event->data);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
        default:
            break;
    }
    return ESP_OK;
}

/**
 *
 */
void mqtt_app_start(void(*setup_parameters)(char *), EventGroupHandle_t event_group)
{
	setup_pid_parameters = setup_parameters;
	mqtt_event_group = event_group;
    const esp_mqtt_client_config_t mqtt_cfg = {
        .uri = "mqtt://mqtt.eclipse.org:1883",    // for mqtt over tcp
        .event_handle =  mqtt_event_handler,
    };

    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(client);
}

int mqtt_client_publish(const char *topic, const char *data) {
	return esp_mqtt_client_publish(client, topic, data, 0, 0, 0);
}

