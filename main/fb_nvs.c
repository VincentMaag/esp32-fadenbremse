
#include <stdint.h>
#include <stdbool.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "sdkconfig.h"

#include "fb_projdefs.h"
#include "fb_nvs.h"
#include "fb_esp_time.h"

static const char *TAG = "fb_nvs";

// store 32bits to nvs
bool set_flash_uint32(uint32_t data, const char *label)
{
	//ESP_LOGW(TAG, "before nvs_open, HEAP min free: %d = free: %d\n", esp_get_minimum_free_heap_size(), esp_get_free_heap_size());
	nvs_handle my_handle;
	// open handler
	esp_err_t err = nvs_open(label, NVS_READWRITE, &my_handle);
	// check if nvs opened successfully
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "Error (%s) opening NVS handle\n", esp_err_to_name(err));
	}
	else
	{
		// write to flash
		err = nvs_set_u32(my_handle, label, data);
		if (err != ESP_OK)
		{
			ESP_LOGE(TAG, "Error (%s) writing to NVS\n", esp_err_to_name(err));
		}
		err = nvs_commit(my_handle);
		if (err != ESP_OK)
		{
			ESP_LOGE(TAG, "Error (%s) committing to NVS\n", esp_err_to_name(err));
		}
		else
		{
			// close handle and free allocated memory
			nvs_close(my_handle);
			//ESP_LOGW(TAG, "after nvs_close, HEAP min free: %d = free: %d\n", esp_get_minimum_free_heap_size(), esp_get_free_heap_size());
			// return successful writing of flash
			return true;
		}
	}
	// return false if something went wrong (we should not get to this point right here...)
	// close handle and return unsuccsessful writing of data
	nvs_close(my_handle);
	return false;
}
// read 32bits from nvs
bool get_flash_uint32(uint32_t *data, const char *label)
{
	nvs_handle my_handle;
	// open handler
	esp_err_t err = nvs_open(label, NVS_READWRITE, &my_handle);
	// check if nvs opened successfully
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "Error (%s) opening NVS handle\n", esp_err_to_name(err));
		return false;
	}
	else
	{
		// read from flash
		err = nvs_get_u32(my_handle, label, data);
		// handle common errors
		switch (err)
		{
		case ESP_OK:
			// close handle and free allocated memory
			nvs_close(my_handle);
			// return successful reading of flash
			return true;
		case ESP_ERR_NVS_NOT_FOUND:
			ESP_LOGI(TAG, "The value \"%s\" is not initialized yet", label);
			break;
		default:
			ESP_LOGE(TAG, "Error (%s) reading from NVS\n", esp_err_to_name(err));
			break;
		}
	}
	// return false if something went wrong (we should not get to this point right here...)
	return false;
}
// store 64bits to nvs
bool set_flash_uint64(uint32_t data, const char *label)
{
	nvs_handle my_handle;
	// open handler
	esp_err_t err = nvs_open(label, NVS_READWRITE, &my_handle);
	// check if nvs opened successfully
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "Error (%s) opening NVS handle\n", esp_err_to_name(err));
	}
	else
	{
		// write to flash
		err = nvs_set_u64(my_handle, label, data);
		if (err != ESP_OK)
		{
			ESP_LOGE(TAG, "Error (%s) writing to NVS\n", esp_err_to_name(err));
		}
		err = nvs_commit(my_handle);
		if (err != ESP_OK)
		{
			ESP_LOGE(TAG, "Error (%s) committing to NVS\n", esp_err_to_name(err));
		}
		else
		{
			// close handle and free allocated memory
			nvs_close(my_handle);
			// return successful writing of flash
			return true;
		}
	}
	// return false if something went wrong (we should not get to this point right here...)
	// close handle and return unsuccsessful writing of data
	nvs_close(my_handle);
	return false;
}
// read 64 bits from nvs
bool get_flash_uint64(uint32_t *data, const char *label)
{
	nvs_handle my_handle;
	// open handler
	esp_err_t err = nvs_open(label, NVS_READWRITE, &my_handle);
	// check if nvs opened successfully
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "Error (%s) opening NVS handle\n", esp_err_to_name(err));
		return false;
	}
	else
	{
		// read from flash
		err = nvs_get_u64(my_handle, label, data);
		// handle common errors
		switch (err)
		{
		case ESP_OK:
			// close handle and free allocated memory
			nvs_close(my_handle);
			// return successful reading of flash
			return true;
		case ESP_ERR_NVS_NOT_FOUND:
			ESP_LOGI(TAG, "The value \"%s\" is not initialized yet", label);
			break;
		default:
			ESP_LOGE(TAG, "Error (%s) reading from NVS\n", esp_err_to_name(err));
			break;
		}
	}
	// return false if something went wrong (we should not get to this point right here...)
	return false;
}
// read string from nvs
bool get_flash_str(char *str, const char *label)
{
	nvs_handle my_handle;
	size_t required_size;
	// open handler
	esp_err_t err = nvs_open(label, NVS_READWRITE, &my_handle);
	// check if nvs opened successfully
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "Error (%s) opening NVS handle\n", esp_err_to_name(err));
		return false;
	}
	else
	{
		// read from flash
		// first, get required length (by setting "out_value" to 0, see nvs_get_str())
		err = nvs_get_str(my_handle, label, 0, &required_size);
		// then, try and read string if no errors. if errors, then handle them below
		if(err == ESP_OK){
			err = nvs_get_str(my_handle, label, str, &required_size);
		}
		// handle common errors
		switch (err)
		{
		case ESP_OK:
			// close handle and free allocated memory
			nvs_close(my_handle);
			// return successful reading of flash
			return true;
		case ESP_ERR_NVS_NOT_FOUND:
			ESP_LOGI(TAG, "The value \"%s\" is not initialized yet", label);
			break;
		case ESP_ERR_NVS_INVALID_LENGTH:
			ESP_LOGE(TAG, "(%s) Invalid string length: %d", esp_err_to_name(err),required_size);
			break;	
		default:
			ESP_LOGE(TAG, "Error (%s) reading from NVS\n", esp_err_to_name(err));
			break;
		}
	}
	// return false if something went wrong (we should not get to this point right here...)
	return false;
}
// set string from nvs
bool set_flash_str(char *str, const char *label)
{
	nvs_handle my_handle;
	// open handler
	esp_err_t err = nvs_open(label, NVS_READWRITE, &my_handle);
	// check if nvs opened successfully
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "Error (%s) opening NVS handle\n", esp_err_to_name(err));
	}
	else
	{
		// write to flash
		err = nvs_set_str(my_handle, label, str);
		if (err != ESP_OK)
		{
			ESP_LOGE(TAG, "Error (%s) writing to NVS\n", esp_err_to_name(err));
		}
		err = nvs_commit(my_handle);
		if (err != ESP_OK)
		{
			ESP_LOGE(TAG, "Error (%s) committing to NVS\n", esp_err_to_name(err));
		}
		else
		{
			// close handle and free allocated memory
			nvs_close(my_handle);
			// return successful writing of flash
			return true;
		}
	}
	// return false if something went wrong (we should not get to this point right here...)
	// close handle and return unsuccsessful writing of data
	nvs_close(my_handle);
	return false;
}

// initialize nvs data (do this in main())
void init_nvs_data()
{
	// load nvs data into nvs-object:
	// ================================================================
	// CONTROL MODE
	if (get_flash_uint32(&nvs_data[0].nvs_control_mode, "mode0"))
	{
		current_control_mode[0] = (uint8_t)(nvs_data[0].nvs_control_mode);
	}
	else
	{
		current_control_mode[0] = CONTROL_MODE_MEAN;
		nvs_data[0].nvs_control_mode = (uint32_t)current_control_mode[0];
		ESP_LOGI(TAG, "Initializing flash: mode0");
		set_flash_uint32(nvs_data[0].nvs_control_mode, "mode0");
	};
	//
	if (get_flash_uint32(&nvs_data[1].nvs_control_mode, "mode1"))
	{
		current_control_mode[1] = (uint8_t)(nvs_data[1].nvs_control_mode);
	}
	else
	{
		current_control_mode[1] = CONTROL_MODE_MEAN;
		nvs_data[1].nvs_control_mode = (uint32_t)current_control_mode[1];
		ESP_LOGI(TAG, "Initializing flash: mode1");
		set_flash_uint32(nvs_data[1].nvs_control_mode, "mode1");
	};
	// check boundaries. If something ist wrong, choose mean mode
	if (current_control_mode[0] > 1)
	{
		current_control_mode[0] = CONTROL_MODE_MEAN;
	}
	if (current_control_mode[1] > 1)
	{
		current_control_mode[1] = CONTROL_MODE_MEAN;
	}
	// ================================================================
	// Setpoint Auto
	if (get_flash_uint32(&nvs_data[0].nvs_Setpoint_Auto, "auto0"))
	{
		wifi_fb[0].setpoint_mean_control = (float)nvs_data[0].nvs_Setpoint_Auto;
	}
	else
	{
		wifi_fb[0].setpoint_mean_control = 5;
		nvs_data[0].nvs_Setpoint_Auto = (uint32_t)wifi_fb[0].setpoint_mean_control;
		ESP_LOGI(TAG, "Initializing flash: auto0");
		set_flash_uint32(nvs_data[0].nvs_Setpoint_Auto, "auto0");
	};
	//
	if (get_flash_uint32(&nvs_data[1].nvs_Setpoint_Auto, "auto1"))
	{
		wifi_fb[1].setpoint_mean_control = (float)nvs_data[1].nvs_Setpoint_Auto;
	}
	else
	{
		wifi_fb[1].setpoint_mean_control = 5;
		nvs_data[1].nvs_Setpoint_Auto = (uint32_t)wifi_fb[1].setpoint_mean_control;
		ESP_LOGI(TAG, "Initializing flash: auto1");
		set_flash_uint32(nvs_data[1].nvs_Setpoint_Auto, "auto1");
	};
	// check boundaries
	if (wifi_fb[0].setpoint_mean_control > 50 || wifi_fb[0].setpoint_mean_control < 0)
	{
		wifi_fb[0].setpoint_mean_control = 5;
	}
	if (wifi_fb[1].setpoint_mean_control > 50 || wifi_fb[1].setpoint_mean_control < 0)
	{
		wifi_fb[1].setpoint_mean_control = 5;
	}
	// ================================================================
	// Setpoint Manual
	if (get_flash_uint32(&nvs_data[0].nvs_Setpoint_Manual, "manual0"))
	{
		wifi_fb[0].setpoint_hdrive = (float)nvs_data[0].nvs_Setpoint_Manual;
	}
	else
	{
		wifi_fb[0].setpoint_hdrive = 50;
		nvs_data[0].nvs_Setpoint_Manual = (uint32_t)wifi_fb[0].setpoint_hdrive;
		ESP_LOGI(TAG, "Initializing flash: manual0");
		set_flash_uint32(nvs_data[0].nvs_Setpoint_Manual, "manual0");
	};
	//
	if (get_flash_uint32(&nvs_data[1].nvs_Setpoint_Manual, "manual1"))
	{
		wifi_fb[1].setpoint_hdrive = (float)nvs_data[1].nvs_Setpoint_Manual;
	}
	else
	{
		wifi_fb[1].setpoint_hdrive = 50;
		nvs_data[1].nvs_Setpoint_Manual = (uint32_t)wifi_fb[1].setpoint_hdrive;
		ESP_LOGI(TAG, "Initializing flash: manual1");
		set_flash_uint32(nvs_data[1].nvs_Setpoint_Manual, "manual1");
	};
	// check boundaries
	if (wifi_fb[0].setpoint_hdrive > 100 || wifi_fb[0].setpoint_hdrive < 0)
	{
		wifi_fb[0].setpoint_hdrive = 50;
	}
	if (wifi_fb[1].setpoint_hdrive > 100 || wifi_fb[1].setpoint_hdrive < 0)
	{
		wifi_fb[1].setpoint_hdrive = 50;
	}
	// ================================================================
	// Monitoring Mode & IP Adress
	if (get_flash_uint32(&nvs_data[0].nvs_monitoring_mode, "monitMode"))
	{
		wifi_fb[0].monitoring_mode = (uint8_t)nvs_data[0].nvs_monitoring_mode;
	}
	else
	{
		wifi_fb[0].monitoring_mode = 1;
		nvs_data[0].nvs_monitoring_mode = (uint32_t)wifi_fb[0].monitoring_mode;
		ESP_LOGI(TAG, "Initializing flash: monitMode");
		set_flash_uint32(nvs_data[0].nvs_monitoring_mode, "monitMode");
	};
	if (wifi_fb[0].monitoring_mode > 1)
	{
		wifi_fb[0].monitoring_mode = 1;
	}
	//
	if (get_flash_uint32(&nvs_data[0].nvs_ip, "ip"))
	{
		wifi_fb[0].ip.addr = (u32_t)nvs_data[0].nvs_ip;
	}
	else
	{
		wifi_fb[0].ip.addr = ipaddr_addr("192.168.0.250");
		nvs_data[0].nvs_ip = (uint32_t)wifi_fb[0].ip.addr;
		ESP_LOGI(TAG, "Initializing flash: ip");
		set_flash_uint32(nvs_data[0].nvs_ip, "ip");
	};
	//
	// pointer to display ip adress
	u8_t *ip_display = (void *)&wifi_fb[0].ip.addr;
	// fallback ip if unplausible IP adress. if first bundle is 0, something is wrong
	if (*ip_display == 0)
	{
		wifi_fb[0].ip.addr = ipaddr_addr("192.168.0.250");
	}
	// ================================================================
	// Angle Offset
	if (get_flash_uint32(&nvs_data[0].nvs_angleOffset, "angleOffset"))
	{
		wifi_fb[0].angleOffset = (uint16_t)nvs_data[0].nvs_angleOffset;
	}
	else
	{
		wifi_fb[0].angleOffset = 0;
		nvs_data[0].nvs_angleOffset = (uint32_t)wifi_fb[0].angleOffset;
		ESP_LOGI(TAG, "Initializing flash: angleOffset");
		set_flash_uint32(nvs_data[0].nvs_angleOffset, "angleOffset");
	};
	//
	if (wifi_fb[0].angleOffset < 0 || wifi_fb[0].angleOffset > 360)
	{
		wifi_fb[0].angleOffset = 0;
	}
	// ================================================================
	// Opening/Closing Times hdrive
	if (get_flash_uint32(&nvs_data[0].nvs_hdrive_closing_time, "closingTime"))
	{
		wifi_fb[0].hdrive_closing_time = (uint16_t)nvs_data[0].nvs_hdrive_closing_time;
	}
	else
	{
		wifi_fb[0].hdrive_closing_time = 290;
		nvs_data[0].nvs_hdrive_closing_time = (uint32_t)wifi_fb[0].hdrive_closing_time;
		ESP_LOGI(TAG, "Initializing flash: closingTime");
		set_flash_uint32(nvs_data[0].nvs_hdrive_closing_time, "closingTime");
	};
	//
	if (get_flash_uint32(&nvs_data[0].nvs_hdrive_opening_time, "openingTime"))
	{
		wifi_fb[0].hdrive_opening_time = (uint16_t)nvs_data[0].nvs_hdrive_opening_time;
	}
	else
	{
		wifi_fb[0].hdrive_opening_time = 120;
		nvs_data[0].nvs_hdrive_opening_time = (uint32_t)wifi_fb[0].hdrive_opening_time;
		ESP_LOGI(TAG, "Initializing flash: openingTime");
		set_flash_uint32(nvs_data[0].nvs_hdrive_opening_time, "openingTime");
	};
	//
	if (wifi_fb[0].hdrive_closing_time == 0)
	{
		wifi_fb[0].hdrive_closing_time = 290;
	}
	if (wifi_fb[0].hdrive_opening_time == 0)
	{
		wifi_fb[0].hdrive_opening_time = 120;
	}
	// ================================================================
	// min/max calibration values for encoder
	if (get_flash_uint32(&nvs_data[0].nvs_enc_min, "enc_min"))
	{
		loom.enc_min = (int16_t)nvs_data[0].nvs_enc_min;
	}
	else
	{
		loom.enc_min = 0;
		nvs_data[0].nvs_enc_min = (uint32_t)loom.enc_min;
		ESP_LOGI(TAG, "Initializing flash: enc_min");
		set_flash_uint32(nvs_data[0].nvs_enc_min, "enc_min");
	};

	if (get_flash_uint32(&nvs_data[0].nvs_enc_max, "enc_max"))
	{
		loom.enc_max = (int16_t)nvs_data[0].nvs_enc_max;
	}
	else
	{
		loom.enc_max = 4095;
		nvs_data[0].nvs_enc_max = (uint32_t)loom.enc_max;
		ESP_LOGI(TAG, "Initializing flash: enc_max");
		set_flash_uint32(nvs_data[0].nvs_enc_max, "enc_max");
	};
	//
	if (loom.enc_max < loom.enc_min || loom.enc_min < 0)
	{
		loom.enc_min = 0;
		loom.enc_max = 4095;
	}
	// ================================================================
	// ssid
	if (get_flash_str(nvs_data[0].nvs_ssid, "ssid"))
	{
		strncpy(wifi_fb[0].ssid, nvs_data[0].nvs_ssid, strlen(nvs_data[0].nvs_ssid) + 1); // +1 for \0
	}
	else
	{
		strcpy(wifi_fb[0].ssid, "myssid");
		strncpy(nvs_data[0].nvs_ssid, wifi_fb[0].ssid, strlen(wifi_fb[0].ssid) + 1); // +1 for \0
		ESP_LOGI(TAG, "Initializing flash: ssid");
		set_flash_str(wifi_fb[0].ssid, "ssid");
	};
	//
	if (wifi_fb[0].ssid[0] == '\0')
	{
		strcpy(wifi_fb[0].ssid, "myssid");
	}
	// ================================================================
	// Chart Y-Lim
	if (get_flash_uint32(&nvs_data[0].nvs_chartYLimit, "chartYLimit"))
	{
		wifi_fb[0].chartYLimit = (uint8_t)nvs_data[0].nvs_chartYLimit;
	}
	else
	{
		wifi_fb[0].chartYLimit = 20;
		nvs_data[0].nvs_chartYLimit = (uint32_t)wifi_fb[0].chartYLimit;
		ESP_LOGI(TAG, "Initializing flash: chartYLimit");
		set_flash_uint32(nvs_data[0].nvs_chartYLimit, "chartYLimit");
	};
	//
	if (wifi_fb[0].chartYLimit < 0 || wifi_fb[0].chartYLimit > 500)
	{
		wifi_fb[0].chartYLimit = 20;
	}
	// ================================================================
	// Time Offset
	if (get_flash_uint64(&nvs_data[0].nvs_timeOffset, "timeOffset"))
	{
		esp_time_init_offset((time_t)nvs_data[0].nvs_timeOffset);
	}
	else
	{
		esp_time_init_offset(131);
		nvs_data[0].nvs_timeOffset = (uint64_t)131;
		ESP_LOGI(TAG, "Initializing flash: timeOffset");
		set_flash_uint64(nvs_data[0].nvs_timeOffset, "timeOffset");
	};
	//

	// ================================================================
	// Display nvs data at startup in console:
	//
	ESP_LOGW(TAG, "Control Modes -- 0: %d -- 1: %d", current_control_mode[0], current_control_mode[1]);
	ESP_LOGW(TAG, "Auto Setpoints -- 0: %.2f -- 1: %.2f", wifi_fb[0].setpoint_mean_control, wifi_fb[1].setpoint_mean_control);
	ESP_LOGW(TAG, "Manual Setpoints -- 0: %.2f -- 1: %.2f", wifi_fb[0].setpoint_hdrive, wifi_fb[1].setpoint_hdrive);
	ESP_LOGW(TAG, "Monitoring Mode -- %i -- IP: %d.%d.%d.%d", wifi_fb[0].monitoring_mode, ip_display[0], ip_display[1], ip_display[2], ip_display[3]);
	ESP_LOGW(TAG, "Angle Offset -- %i -- ", wifi_fb[0].angleOffset);
	ESP_LOGW(TAG, "Opening Time -- %i -- Closing Time -- %i -- ", wifi_fb[0].hdrive_opening_time, wifi_fb[0].hdrive_closing_time);
	ESP_LOGW(TAG, "enc_min -- %i -- enc_max -- %i -- ", loom.enc_min, loom.enc_max);
	ESP_LOGW(TAG, "ssid: -- %s -- password -- mypassword -- ", wifi_fb[0].ssid);
	ESP_LOGW(TAG, "chartYLimit: -- %i -- ", wifi_fb[0].chartYLimit);
	ESP_LOGW(TAG, "timeOffset: -- %ld -- ", esp_time_get_current_offset());

	nvs_stats_t nvs_stats;
	nvs_get_stats(NULL, &nvs_stats);
	printf("Count: UsedEntries = (%d), FreeEntries = (%d), AllEntries = (%d)\n",
		   nvs_stats.used_entries, nvs_stats.free_entries, nvs_stats.total_entries);
}

void nvs_task(void *arg)
{

	while (1)
	{
		// store data to nvs only if variables have changed value
		// some variables may have other conditions

		if (nvs_data[0].nvs_control_mode != (uint32_t)current_control_mode[0])
		{
			nvs_data[0].nvs_control_mode = (uint32_t)current_control_mode[0];
			set_flash_uint32(nvs_data[0].nvs_control_mode, "mode0");
		}
		if (nvs_data[1].nvs_control_mode != (uint32_t)current_control_mode[1])
		{
			nvs_data[1].nvs_control_mode = (uint32_t)current_control_mode[1];
			set_flash_uint32(nvs_data[1].nvs_control_mode, "mode1");
		}
		if (nvs_data[0].nvs_Setpoint_Auto != (uint32_t)wifi_fb[0].setpoint_mean_control)
		{
			nvs_data[0].nvs_Setpoint_Auto = (uint32_t)wifi_fb[0].setpoint_mean_control;
			set_flash_uint32(nvs_data[0].nvs_Setpoint_Auto, "auto0");
		}
		if (nvs_data[1].nvs_Setpoint_Auto != (uint32_t)wifi_fb[1].setpoint_mean_control)
		{
			nvs_data[1].nvs_Setpoint_Auto = (uint32_t)wifi_fb[1].setpoint_mean_control;
			set_flash_uint32(nvs_data[1].nvs_Setpoint_Auto, "auto1");
		}
		if (nvs_data[0].nvs_Setpoint_Manual != (uint32_t)wifi_fb[0].setpoint_hdrive)
		{
			nvs_data[0].nvs_Setpoint_Manual = (uint32_t)wifi_fb[0].setpoint_hdrive;
			set_flash_uint32(nvs_data[0].nvs_Setpoint_Manual, "manual0");
		}
		if (nvs_data[1].nvs_Setpoint_Manual != (uint32_t)wifi_fb[1].setpoint_hdrive)
		{
			nvs_data[1].nvs_Setpoint_Manual = (uint32_t)wifi_fb[1].setpoint_hdrive;
			set_flash_uint32(nvs_data[1].nvs_Setpoint_Manual, "manual1");
		}
		if (nvs_data[0].nvs_monitoring_mode != (uint32_t)wifi_fb[0].monitoring_mode)
		{
			nvs_data[0].nvs_monitoring_mode = (uint32_t)wifi_fb[0].monitoring_mode;
			set_flash_uint32(nvs_data[0].nvs_monitoring_mode, "monitMode");
		}
		if (nvs_data[0].nvs_ip != (uint32_t)wifi_fb[0].ip.addr)
		{
			nvs_data[0].nvs_ip = (uint32_t)wifi_fb[0].ip.addr;
			set_flash_uint32(nvs_data[0].nvs_ip, "ip");
		}
		if (nvs_data[0].nvs_angleOffset != (uint32_t)wifi_fb[0].angleOffset)
		{
			nvs_data[0].nvs_angleOffset = (uint32_t)wifi_fb[0].angleOffset;
			set_flash_uint32(nvs_data[0].nvs_angleOffset, "angleOffset");
		}
		if (nvs_data[0].nvs_hdrive_closing_time != (uint32_t)wifi_fb[0].hdrive_closing_time)
		{
			nvs_data[0].nvs_hdrive_closing_time = (uint32_t)wifi_fb[0].hdrive_closing_time;
			set_flash_uint32(nvs_data[0].nvs_hdrive_closing_time, "closingTime");
		}
		if (nvs_data[0].nvs_hdrive_opening_time != (uint32_t)wifi_fb[0].hdrive_opening_time)
		{
			nvs_data[0].nvs_hdrive_opening_time = (uint32_t)wifi_fb[0].hdrive_opening_time;
			set_flash_uint32(nvs_data[0].nvs_hdrive_opening_time, "openingTime");
		}
		if (nvs_data[0].nvs_enc_min != (uint32_t)loom.enc_min)
		{
			nvs_data[0].nvs_enc_min = (uint32_t)loom.enc_min;
			set_flash_uint32(nvs_data[0].nvs_enc_min, "enc_min");
		}
		if (nvs_data[0].nvs_enc_max != (uint32_t)loom.enc_max)
		{
			nvs_data[0].nvs_enc_max = (uint32_t)loom.enc_max;
			set_flash_uint32(nvs_data[0].nvs_enc_max, "enc_max");
		}
		if (strcmp(nvs_data[0].nvs_ssid, wifi_fb[0].ssid))
		{
			strncpy(nvs_data[0].nvs_ssid, wifi_fb[0].ssid, strlen(wifi_fb[0].ssid) + 1); // +1 for \0
			set_flash_str(wifi_fb[0].ssid, "ssid");
		}
		if (nvs_data[0].nvs_chartYLimit != (uint32_t)wifi_fb[0].chartYLimit)
		{
			nvs_data[0].nvs_chartYLimit = (uint32_t)wifi_fb[0].chartYLimit;
			set_flash_uint32(nvs_data[0].nvs_chartYLimit, "chartYLimit");
		}
		if ((abs(esp_time_get_next_needed_offset() - (time_t)nvs_data[0].nvs_timeOffset) > (time_t)21600) || check_clear_event_bit(wifi_event_group, BIT_NEW_TIME_OFFSET_RECEIVED)) // 6*3600 store time offset only if diff > 6h
		{
			ESP_LOGW(TAG, "Writing time offset to flash: %ld", esp_time_get_next_needed_offset());
			nvs_data[0].nvs_timeOffset = (uint64_t)esp_time_get_next_needed_offset();
			set_flash_uint64(nvs_data[0].nvs_timeOffset, "timeOffset");
		}

		// timing not essential. check to store every few seconds
		vTaskDelay((2000 / portTICK_PERIOD_MS));
	}
}
