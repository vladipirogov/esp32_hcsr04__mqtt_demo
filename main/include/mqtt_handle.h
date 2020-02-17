/*
 * mqtt_handle.h
 *
 *  Created on: Feb 13, 2020
 *      Author: Vladyslav_Pyrohov
 */

#ifndef MAIN_INCLUDE_MQTT_HANDLE_H_
#define MAIN_INCLUDE_MQTT_HANDLE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_wifi.h"
#include "mqtt_client.h"
#include "freertos/event_groups.h"
#include "esp_log.h"

EventGroupHandle_t mqtt_event_group;

void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data);

void ip_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data);

void wifi_init(void);

esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event);

void mqtt_app_start(void(*setup_parameters)(char *));

#ifdef __cplusplus
}
#endif

#endif /* MAIN_INCLUDE_MQTT_HANDLE_H_ */
