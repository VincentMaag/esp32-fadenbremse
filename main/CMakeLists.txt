set(COMPONENT_SRCS "main.c"
	      	"fb_data_acq.c"
                "fb_hdrive.c"
                "fb_mean_control.c"
                "fb_nvs.c"
                "fb_wifi.c"
                "fb_blinker.c"
                "fb_color_sensor.c"
		"C:/esp32/GitHub/esp32-fadenbremse/components/websocket/websocket.c"
		"C:/esp32/GitHub/esp32-fadenbremse/components/websocket/websocket_server.c"
		"fb_sd_card.c"
		"fb_esp_time.c"
                )

set(COMPONENT_ADD_INCLUDEDIRS ".")

register_component()
