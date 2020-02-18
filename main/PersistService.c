/*
 * flash.c
 *
 *  Created on: Feb 14, 2020
 *      Author: Vladyslav_Pyrohov
 */

#include <include/PersistService.h>
#include "esp_system.h"

static nvs_handle_t handle;

void flash_init(void) {
	// Initialize NVS
	    esp_err_t err = nvs_flash_init();
	    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
	        // NVS partition was truncated and needs to be erased
	        // Retry nvs_flash_init
	        ESP_ERROR_CHECK(nvs_flash_erase());
	        err = nvs_flash_init();
	    }
	    ESP_ERROR_CHECK( err );
	    // Open
		printf("\n");
		printf("Opening Non-Volatile Storage (NVS) handle... ");
	    err = nvs_open("storage", NVS_READWRITE, &handle);
		if (err != ESP_OK) {
			printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
		} else {
			printf("Done\n");
		}
}

/**
 *
 */
void read_param_from_flash(const char* key, uint64_t* out_value) {
	// Read
	printf("Reading from NVS ... ");
	esp_err_t err = nvs_get_u64(handle, key, out_value);
	switch (err) {
		case ESP_OK:
			printf("Done\n");
			break;
		case ESP_ERR_NVS_NOT_FOUND:
			printf("The value is not initialized yet!\n");
			break;
		default :
			printf("Error (%s) reading!\n", esp_err_to_name(err));
	}
}

/**
 *
 */
void write_param_to_flash(const char* key, uint64_t value) {
	esp_err_t err = nvs_set_u64(handle, key, value);
	printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
	// Commit written value.
	printf("Committing updates in NVS ... ");
	err = nvs_commit(handle);
	printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
}

