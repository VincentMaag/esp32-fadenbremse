/*
	...

*/
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "lwip/ip4_addr.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "tcpip_adapter.h"
#include "driver/gpio.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "fb_wifi.h"
#include "fb_projdefs.h"
#include "fb_blinker.h"
#include "fb_hdrive.h"
#include "fb_sd_card.h"
#include "fb_esp_time.h"

#include "http_parser.h"

#define EXAMPLE_ESP_WIFI_SSID CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_MAX_STA_CONN CONFIG_MAX_STA_CONN

static const char *TAG = "fb_wifi";

static QueueHandle_t client_queue;
const static int client_queue_size = 10;

TaskHandle_t server_handle_task_handle;

#define GPIO_WIFI_ENABLE GPIO_NUM_14 // GPIO_NUM_18 //
#define GPIO_INPUT_PIN_SEL ((1ULL << GPIO_WIFI_ENABLE))

bool bGlobalConnectionStatus = false;
// wifi mode
int iWifiMode = 0; // ev in wifi task verschieben, braucht es nicht hier draussen

// =====================================================================
// stuff from wifi_softAP...
//
/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;
// don't really know what these do here
// think this one should actually be in main.c, however
// wifi_init_softap needs this function, so not the wiser yet...
static esp_err_t event_handler(void *ctx, system_event_t *event)
{
	switch (event->event_id)
	{
	case SYSTEM_EVENT_AP_STACONNECTED:
		ESP_LOGI(TAG, "station:" MACSTR " join, AID=%d",
				 MAC2STR(event->event_info.sta_connected.mac),
				 event->event_info.sta_connected.aid);
		ESP_LOGI(TAG, "AP STA CONNECTED");
		bGlobalConnectionStatus = true;
		blinker_set_mode(BLINKER_MEDIUM_DOUBLE);
		break;
	case SYSTEM_EVENT_AP_STADISCONNECTED:
		ESP_LOGI(TAG, "station:" MACSTR "leave, AID=%d",
				 MAC2STR(event->event_info.sta_disconnected.mac),
				 event->event_info.sta_disconnected.aid);
		ESP_LOGI(TAG, "AP STA DISCONNECTED");
		bGlobalConnectionStatus = false;
		blinker_set_mode(BLINKER_SLOW);
		break;
	case SYSTEM_EVENT_STA_CONNECTED:
		ESP_LOGI(TAG, "STA CONNECTED");
		bGlobalConnectionStatus = true;
		blinker_set_mode(BLINKER_MEDIUM_DOUBLE);
		break;
	case SYSTEM_EVENT_STA_DISCONNECTED:
		ESP_LOGI(TAG, "STA DISCONNECTED");
		bGlobalConnectionStatus = false;
		blinker_set_mode(BLINKER_SLOW);
		break;

	default:
		break;
	}
	return ESP_OK;
}
// =====================================================================
// function to connect to router if in STA mode
void wifi_try_connect_sta()
{

	for (int i = 0; i < 20; i++)
	{
		ESP_LOGE(TAG, "make sure wifi is disonnected");
		ESP_ERROR_CHECK(esp_wifi_disconnect());
		ESP_LOGE(TAG, "disconnecting...");
		// vTaskDelay((5000 / portTICK_PERIOD_MS));
		ESP_LOGE(TAG, "trying to connect to router");
		ESP_ERROR_CHECK(esp_wifi_connect());
		ESP_LOGE(TAG, "connecting... ");
		vTaskDelay((5000 / portTICK_PERIOD_MS));
		if (bGlobalConnectionStatus)
		{
			ESP_LOGE(TAG, "event handler signaling successful connection");
			break;
		}
		else if (i == 19)
		{
			ESP_LOGE(TAG, "timout connection to router! 20 unsuccessful connection attempts");
		}
		else
		{
			ESP_LOGE(TAG, "retry");
			;
		}
	}
}
// =====================================================================
// function to initialize wifi in sta mode
void wifi_init_sta()
{
	// create event group (not sure if needed actually...)
	s_wifi_event_group = xEventGroupCreate();
	// init global connection status with false
	bGlobalConnectionStatus = false;
	// wifi power
	esp_wifi_set_max_tx_power((uint8_t)60);
	// init adapter
	tcpip_adapter_init();
	// stop dhcp client
	ESP_ERROR_CHECK(tcpip_adapter_dhcpc_stop(TCPIP_ADAPTER_IF_STA));
	// initialize event handling
	ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
	// set default wifi configuration
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	// configure tcpip adapter out of remanent memory
	tcpip_adapter_ip_info_t IpInfo;
	IpInfo.ip.addr = wifi_fb[0].ip.addr;
	IpInfo.gw.addr = ipaddr_addr("192.168.0.1");		// 192.168.0.1 default gateway
	IpInfo.netmask.addr = ipaddr_addr("255.255.255.0"); // 255.255.255.0 default net mask
	ESP_ERROR_CHECK(tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_STA, &IpInfo));
	// configure sta-params
	wifi_config_t sta_config = {
		.sta = {
			.ssid = "SaatiPreSeries",
			.password = "mypassword",
			.bssid_set = false,
		}};
	// do not think next is needed...
	if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0)
	{
		sta_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
	}
	// set sta mode & config
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &sta_config));
	// start wifi (maybe not needed, because done in main of wifi)
	ESP_ERROR_CHECK(esp_wifi_start());
}
// =====================================================================
// function to initialize wifi in ap mode
void wifi_init_ap()
{
	// create event group (not sure if needed actually...)
	s_wifi_event_group = xEventGroupCreate();
	// wifi power
	esp_wifi_set_max_tx_power((uint8_t)60);
	// init adapter
	tcpip_adapter_init();
	// stop dhcp server
	ESP_ERROR_CHECK(tcpip_adapter_dhcps_stop(TCPIP_ADAPTER_IF_AP));
	// initialize event handling
	ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL)); // not sure if this is necessary
	// set default wifi configuration
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	// configure tcpip adapter out of remanent memory
	tcpip_adapter_ip_info_t IpInfo;
	IpInfo.ip.addr = wifi_fb[0].ip.addr;
	IpInfo.gw.addr = ipaddr_addr("192.168.0.1");		// 192.168.0.1 default gateway
	IpInfo.netmask.addr = ipaddr_addr("255.255.255.0"); // 255.255.255.0 default net mask
	ESP_ERROR_CHECK(tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_AP, &IpInfo));
	// start dhcp server
	ESP_ERROR_CHECK(tcpip_adapter_dhcps_start(TCPIP_ADAPTER_IF_AP));
	ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
	// configure default ap-params
	wifi_config_t ap_config = {
		.ap = {
			.ssid = "SaatiPreSeries",
			//.ssid = (wifi_fb[0].ssid),
			// .ssid_len = strlen(wifi_fb[0].ssid),
			.ssid_len = strlen("SaatiPreSeries"),
			.password = "mypassword",
			.max_connection = EXAMPLE_MAX_STA_CONN,
			.authmode = WIFI_AUTH_WPA_WPA2_PSK},
	};

	// set desired ssid (and password) if valid
	if (strlen(wifi_fb[0].ssid) > 0)
	{
		strncpy((char *)ap_config.ap.ssid, wifi_fb[0].ssid, strlen(wifi_fb[0].ssid) + 1);
		ap_config.ap.ssid_len = strlen(wifi_fb[0].ssid);
	}
	// set ap mode & config
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &ap_config));
	// log wifi configuration
	u8_t *ip_display = (void *)&IpInfo.ip.addr;
	ESP_LOGW(TAG, "Configured ESP with SSID: %s and PASSWORD: %s, IP: %d.%d.%d.%d", ap_config.ap.ssid, ap_config.ap.password, ip_display[0], ip_display[1], ip_display[2], ip_display[3]);
	// do not start wifi in ap mode (user event!)
	// ESP_ERROR_CHECK(esp_wifi_start());
}
// =============================================================================================================
// function to handle webserver
static void http_serve(struct netconn *conn)
{
	const static char *TAG2 = "http_server";
	const static char HTML_HEADER[] = "HTTP/1.1 200 OK\nContent-type: text/html\n\n";
	// const static char ERROR_HEADER[] = "HTTP/1.1 404 Not Found\nContent-type: text/html\n\n";
	const static char JS_HEADER[] = "HTTP/1.1 200 OK\nContent-type: text/javascript\n\n";
	const static char CSS_HEADER[] = "HTTP/1.1 200 OK\nContent-type: text/css\n\n";
	// const static char PNG_HEADER[] = "HTTP/1.1 200 OK\nContent-type: image/png\n\n";
	// const static char ICO_HEADER[] = "HTTP/1.1 200 OK\nContent-type: image/x-icon\n\n";
	// const static char PDF_HEADER[] = "HTTP/1.1 200 OK\nContent-type: application/pdf\n\n";
	// const static char EVENT_HEADER[] = "HTTP/1.1 200 OK\nContent-Type: text/event-stream\nCache-Control: no-cache\nretry: 3000\n\n";
	struct netbuf *inbuf;
	static char *buf;
	static uint16_t buflen;
	static err_t err;

	// char outbuf[1000];

	// default page
	extern const uint8_t root_html_start[] asm("_binary_index_html_start");
	extern const uint8_t root_html_end[] asm("_binary_index_html_end");
	const uint32_t root_html_len = root_html_end - root_html_start;

	// index.js
	extern const uint8_t test_js_start[] asm("_binary_index_js_start");
	extern const uint8_t test_js_end[] asm("_binary_index_js_end");
	const uint32_t test_js_len = test_js_end - test_js_start;

	// Chart.js
	extern const uint8_t Chart_js_start[] asm("_binary_Chart_js_start");
	extern const uint8_t Chart_js_end[] asm("_binary_Chart_js_end");
	const uint32_t Chart_js_len = Chart_js_end - Chart_js_start;

	// index.css
	extern const uint8_t index_css_start[] asm("_binary_index_css_start");
	extern const uint8_t index_css_end[] asm("_binary_index_css_end");
	const uint32_t index_css_len = index_css_end - index_css_start;

	netconn_set_recvtimeout(conn, 1000); // allow a connection timeout of 1 second
	err = netconn_recv(conn, &inbuf);

	if (err == ERR_OK)
	{
		netbuf_data(inbuf, (void **)&buf, &buflen);
		if (buf)
		{

			// default page
			if (strstr(buf, "GET / ") && !strstr(buf, "Upgrade: websocket"))
			{
				ESP_LOGI(TAG2, "Sending /");
				netconn_write(conn, HTML_HEADER, sizeof(HTML_HEADER) - 1, NETCONN_NOCOPY);
				netconn_write(conn, root_html_start, root_html_len, NETCONN_NOCOPY);
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);
			}
			// default page websocket
			else if (strstr(buf, "GET / ") && strstr(buf, "Upgrade: websocket"))
			{
				ESP_LOGI(TAG2, "Requesting websocket on /");
				ws_server_add_client(conn, buf, buflen, "/", websocket_callback);
				netbuf_delete(inbuf);
			}
			//
			else if (strstr(buf, "GET /index.js "))
			{
				ESP_LOGI(TAG2, "Sending /index.js");
				netconn_write(conn, JS_HEADER, sizeof(JS_HEADER) - 1, NETCONN_NOCOPY);
				netconn_write(conn, test_js_start, test_js_len, NETCONN_NOCOPY);
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);
			}
			else if (strstr(buf, "GET /Chart.js "))
			{
				ESP_LOGI(TAG2, "Sending /Chart.js");
				netconn_write(conn, JS_HEADER, sizeof(JS_HEADER) - 1, NETCONN_NOCOPY);
				netconn_write(conn, Chart_js_start, Chart_js_len, NETCONN_NOCOPY);
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);
				ESP_LOGI(TAG2, "Sent Chart.js");
			}
			else if (strstr(buf, "GET /index.css "))
			{
				ESP_LOGI(TAG2, "Sending /index.css");
				netconn_write(conn, CSS_HEADER, sizeof(CSS_HEADER) - 1, NETCONN_NOCOPY);
				netconn_write(conn, index_css_start, index_css_len, NETCONN_NOCOPY);
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);
			}
			// =========================================================================
			// testing stuff
			else if (strstr(buf, "GET /test"))
			{
				ESP_LOGW(TAG2, "Requesting CONSOLE TEST! /");
				//
				netbuf_delete(inbuf);
			}
			// =========================================================================
			// hdrives disable
			else if (strstr(buf, "POST /disableHdrives"))
			{
				ESP_LOGW(TAG2, "Received Request to disable hdrives");
				// tell via event that hdrive must be disabled
				xEventGroupSetBits(wifi_event_group, BIT_REQ_DISABLE_HDRIVE);
				// respond with header (?)
				netconn_write(conn, HTML_HEADER, sizeof(HTML_HEADER) - 1, NETCONN_NOCOPY);
				//
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);
			}
			// =========================================================================
			// hdrives re enable
			else if (strstr(buf, "POST /enableHdrives"))
			{
				ESP_LOGW(TAG2, "Received Request to enable hdrives");
				// tell via event that hdrive must be enabled
				xEventGroupSetBits(wifi_event_group, BIT_REQ_ENABLE_HDRIVE);
				// respond with header (?)
				netconn_write(conn, HTML_HEADER, sizeof(HTML_HEADER) - 1, NETCONN_NOCOPY);
				//
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);
			}
			// =========================================================================
			// data
			else if (strstr(buf, "GET /getData"))
			{
				// create ints out of floats to send single bytes (1 byte <= 255 float value)

				uint8_t idata00 = current_control_mode[0];

				// ESP_LOGE(TAG2,"bit0 sent: %i\n",idata00);
				//
				int idata0 = (int)(100 * mean_control[0].display_mean);
				int idata1 = (int)(100 * mean_control[1].display_mean);
				int idata2 = (int)(hdrive[0].currentSetpointClosedPercent);
				int idata3 = (int)(hdrive[1].currentSetpointClosedPercent);
				// information for hmi-init
				int idata4 = (int)(wifi_fb[0].setpoint_mean_control);
				int idata5 = (int)(wifi_fb[1].setpoint_mean_control);
				int idata6 = (int)(wifi_fb[0].setpoint_hdrive);
				int idata7 = (int)(wifi_fb[1].setpoint_hdrive);
				// ESP_LOGI(TAG, "wifi sending: manual setpoint is %.2f",wifi_fb[0].setpoint_hdrive);
				// monitoring mode
				uint8_t idata8 = wifi_fb[0].monitoring_mode;
				// actual Angle Offset
				int idata9 = (int)(loom.angleOffset);
				// actual Angle
				int idata10 = (int)(loom.actualAngle);
				// calibration
				uint8_t idata11 = wifi_fb[0].calibrate_enc;
				// actual opening/closing times
				int idata12 = (int)hdrive[0].hdrive_opening_time;
				int idata13 = (int)hdrive[0].hdrive_closing_time;
				// ip adress
				// u8_t *ip_display = (void*)&wifi_fb[0].ip.addr; //--> done directly further down
				// diagnostic srtuff
				EventBits_t diagnose_bits = xEventGroupGetBits(diagnose_event_group);
				// printf("%d\n",(int)diagnose_bits);

				// create buffer
				char outbuf_mean[100];
				// 0&1 byte = control mode (second byte not used yet)
				outbuf_mean[0] = ((char *)&idata00)[0];
				// 2&3 byte = mean 1
				outbuf_mean[2] = ((char *)&idata0)[0];
				outbuf_mean[3] = ((char *)&idata0)[1];
				// 4&5 byte = mean 2
				outbuf_mean[4] = ((char *)&idata1)[0];
				outbuf_mean[5] = ((char *)&idata1)[1];
				// 6 byte = %-setpoint 1, i.e actual closed position
				outbuf_mean[6] = ((char *)&idata2)[0];
				// 7 byte = %-setpoint 2
				outbuf_mean[7] = ((char *)&idata3)[0];
				// 8 byte = auto setpoint 1
				outbuf_mean[8] = ((char *)&idata4)[0];
				// 9 byte = auto setpoint 2
				outbuf_mean[9] = ((char *)&idata5)[0];
				// 10 byte = manual setpoint 1
				outbuf_mean[10] = ((char *)&idata6)[0];
				// 11 byte = manual setpoint 2
				outbuf_mean[11] = ((char *)&idata7)[0];
				// 12 byte = monitoring mode
				outbuf_mean[12] = ((char *)&idata8)[0];
				// 13&14 byte = actual angle offset
				outbuf_mean[13] = ((char *)&idata9)[0];
				outbuf_mean[14] = ((char *)&idata9)[1];
				// 15&16 byte = actual angle
				outbuf_mean[15] = ((char *)&idata10)[0];
				outbuf_mean[16] = ((char *)&idata10)[1];
				// 17 byte = calibration on/off
				outbuf_mean[17] = ((char *)&idata11)[0];
				// 18&19 byte = opening time
				outbuf_mean[18] = ((char *)&idata12)[0];
				outbuf_mean[19] = ((char *)&idata12)[1];
				// 20&21 byte = closing time
				outbuf_mean[20] = ((char *)&idata13)[0];
				outbuf_mean[21] = ((char *)&idata13)[1];
				// 22-25 byte = ip adress
				outbuf_mean[22] = ((char *)&wifi_fb[0].ip.addr)[0];
				outbuf_mean[23] = ((char *)&wifi_fb[0].ip.addr)[1];
				outbuf_mean[24] = ((char *)&wifi_fb[0].ip.addr)[2];
				outbuf_mean[25] = ((char *)&wifi_fb[0].ip.addr)[3];
				// 26&27 = calibration value high
				outbuf_mean[26] = ((char *)&loom.enc_max)[0];
				outbuf_mean[27] = ((char *)&loom.enc_max)[1];
				// 28&29 = calibration value low
				outbuf_mean[28] = ((char *)&loom.enc_min)[0];
				outbuf_mean[29] = ((char *)&loom.enc_min)[1];
				// bytes 30-44 are 15 bytes for ssid (we set max length 15 char)
				// if ssid is longer than 15 char then rest is cut off
				for (int m = 0; m < 15; m++)
				{
					outbuf_mean[30 + m] = wifi_fb[0].ssid[m];
				}
				// continue from byte 60 --> dataAq status
				outbuf_mean[60] = ((char *)&loom.machine_running)[0];
				// next byte is Y-Axis limit
				outbuf_mean[61] = ((char *)&wifi_fb[0].chartYLimit)[0];
				// next byte is diagnostic bits --> limit to 8 bits
				outbuf_mean[62] = ((char *)&diagnose_bits)[0];

				//
				netconn_write(conn, outbuf_mean, sizeof(outbuf_mean), NETCONN_NOCOPY);
				//
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);

				// err_t   netconn_recv(struct netconn *conn, struct netbuf **new_buf);
			}
			// =========================================================================
			// =========================================================================
			// adc 0
			else if (strstr(buf, "GET /getArray"))
			{
				// ESP_LOGE(TAG2,"Requesting array \n");
				int i;
				int idata1;
				int idata2;
				// create buffer, 4*idx + 1 (two Sensors, each value = 2 bytes, + valid channel)
				if ((dataAcq[0].idx > 0) && (dataAcq[0].idx < 1000)) // make sure idx is not 0 and not very large
				{
					char outbuf_btsr[4 * dataAcq[0].idx + 1]; // this can easily lead to stack overflows...
					// first buffer value is channel select
					outbuf_btsr[0] = ((char *)&dataAcq[0].valid_channel)[0];

					// fill buffer with Sensor data. Idx = amount of valid measurements
					// convert float datapoint to int and multiply by 100
					// this leads to max Force of 2^16 / 100 = 655 cN (or 327 cN if interpreted as signed int)
					for (i = 0; i < dataAcq[0].idx; i++)
					{
						// multiply float by 100, convert to uint16, we get 2 bytes
						idata1 = (uint16_t)(dataAcq[0].valid_data[i] * 100);
						// convert to char data and copy both bytes into buffer. 2*i"+1" because first buffer value is channel!
						outbuf_btsr[2 * i + 1] = ((char *)&idata1)[0];
						outbuf_btsr[2 * i + 2] = ((char *)&idata1)[1];
						// do the same manipulation for sensor 2 data
						idata2 = (uint16_t)(dataAcq[1].valid_data[i] * 100);
						// convert & copy again, shift buffer index by idx-value
						outbuf_btsr[2 * i + 1 + 2 * dataAcq[0].idx] = ((char *)&idata2)[0];
						outbuf_btsr[2 * i + 2 + 2 * dataAcq[0].idx] = ((char *)&idata2)[1];

						// !!!!!!!!!!!!!!!!!!
						// massive hack 25.06.21
						// set very first number = valid channel nr
						// int hack = dataAcq[0].valid_channel;
						// outbuf_btsr[0] = ((char *)&hack)[0];
					}
					// printf("[0]: %i",);
					netconn_write(conn, outbuf_btsr, sizeof(outbuf_btsr), NETCONN_NOCOPY);
				}
				else
				{
					char outbuf_btsr[3] = {0, 0, 0};
					netconn_write(conn, outbuf_btsr, sizeof(outbuf_btsr), NETCONN_NOCOPY);
				}
				//
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);

				// err_t   netconn_recv(struct netconn *conn, struct netbuf **new_buf);
			}
			// =========================================================================
			// Setpoint closed hdrive 1
			else if (strstr(buf, "POST /sendhdrive1setpoint"))
			{
				ESP_LOGW(TAG2, "Receiving Data");

				// define how long the request string is
				int length = (int)sizeof("POST /sendhdrive1setpoint");
				// fill int value according to where we know the first char number starts
				// extracted data: first 3 bytes is Manual setpoint [%], next two is Auto setpoint [cN]
				int extractedData = ((buf[length - 1] - '0') * 100 + (buf[length] - '0') * 10 + (buf[length + 1] - '0'));
				// second number
				int extractedData2 = ((buf[length + 2] - '0') * 10 + (buf[length + 3] - '0'));
				// Log
				ESP_LOGW(TAG2, "[0]: Manual Setpoint: %i, Auto Setpoint: %i", extractedData,extractedData2);
				// printf("String: %s\nlength: %i\nbuf: %c\nbufExtract: %i\n",buf,length,buf[length-1],test);
				// printf("%s\n",buf);
				// printf("first number: %i second number: %i\n", extractedData, extractedData2);
				//
				// copy setoint and signal event
				wifi_fb[0].setpoint_hdrive = (float)extractedData;
				wifi_fb[0].setpoint_mean_control = (float)extractedData2;
				xEventGroupSetBits(hdrive_event_group_array[0], BIT_CHANGE_SETPOINT_HDRIVE);

				// respond with header
				netconn_write(conn, HTML_HEADER, sizeof(HTML_HEADER) - 1, NETCONN_NOCOPY);
				//
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);

				// err_t   netconn_recv(struct netconn *conn, struct netbuf **new_buf);
			}
			// =========================================================================
			// Setpoint closed hdrive 2
			else if (strstr(buf, "POST /sendhdrive2setpoint"))
			{
				ESP_LOGW(TAG2, "Receiving Data");

				// define how long the request string is
				int length = (int)sizeof("POST /sendhdrive2setpoint");
				// fill int value according to where we know the first char number starts
				// extracted data: first 3 bytes is Manual setpoint [%], next two is Auto setpoint [cN]
				int extractedData = ((buf[length - 1] - '0') * 100 + (buf[length] - '0') * 10 + (buf[length + 1] - '0'));
				// second number
				int extractedData2 = ((buf[length + 2] - '0') * 10 + (buf[length + 3] - '0'));
				// Log
				ESP_LOGW(TAG2, "[1]: Manual Setpoint: %i, Auto Setpoint: %i", extractedData,extractedData2);
				// printf("String: %s\nlength: %i\nbuf: %c\nbufExtract: %i\n",buf,length,buf[length-1],test);
				//
				//  copy setoint and signal event
				wifi_fb[1].setpoint_hdrive = (float)extractedData;
				wifi_fb[1].setpoint_mean_control = (float)extractedData2;
				xEventGroupSetBits(hdrive_event_group_array[1], BIT_CHANGE_SETPOINT_HDRIVE);

				// respond with header
				netconn_write(conn, HTML_HEADER, sizeof(HTML_HEADER) - 1, NETCONN_NOCOPY);
				//
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);

				// err_t   netconn_recv(struct netconn *conn, struct netbuf **new_buf);
			}
			// =========================================================================
			// Offset
			else if (strstr(buf, "POST /resetAngleOffset"))
			{
				ESP_LOGW(TAG2, "Receiving Data");

				// define how long the request string is
				int length = (int)sizeof("POST /resetAngleOffset");
				// fill int value according to where we know the first char number starts
				// extracted data: first 3 bytes is offset
				int extractedData = ((buf[length - 1] - '0') * 100 + (buf[length] - '0') * 10 + (buf[length + 1] - '0'));
				// Log
				ESP_LOGW(TAG2, "New Offset Angle: %i", extractedData);
				//
				// copy offset
				wifi_fb[0].angleOffset = (float)extractedData;
				// signal event
				xEventGroupSetBits(wifi_event_group, BIT_SET_ANGLE_OFFSET);

				// respond with header
				netconn_write(conn, HTML_HEADER, sizeof(HTML_HEADER) - 1, NETCONN_NOCOPY);
				//
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);

				// err_t   netconn_recv(struct netconn *conn, struct netbuf **new_buf);
			}
			// =========================================================================
			// Offset
			else if (strstr(buf, "POST /setTiming"))
			{
				ESP_LOGW(TAG2, "Receiving Data");

				// define how long the request string is
				int length = (int)sizeof("POST /setTiming");
				// fill int value according to where we know the first char number starts
				// first 3 bytes are opening time
				int extractedData = ((buf[length - 1] - '0') * 100 + (buf[length] - '0') * 10 + (buf[length + 1] - '0'));
				// next three bytes are closing time
				int extractedData2 = ((buf[length + 2] - '0') * 100 + (buf[length + 3] - '0') * 10 + (buf[length + 4] - '0'));
				// Log
				ESP_LOGW(TAG2, "Received Opening/Closing times: %i, %i\n", extractedData, extractedData2);
				//
				// copy offset
				wifi_fb[0].hdrive_opening_time = (uint16_t)extractedData;
				wifi_fb[0].hdrive_closing_time = (uint16_t)extractedData2;

				// signal event
				xEventGroupSetBits(wifi_event_group, BIT_SET_OPENING_CLOSING_TIME);

				// respond with header
				netconn_write(conn, HTML_HEADER, sizeof(HTML_HEADER) - 1, NETCONN_NOCOPY);
				//
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);

				// err_t   netconn_recv(struct netconn *conn, struct netbuf **new_buf);
			}
			// =========================================================================
			// IP Adress
			else if (strstr(buf, "POST /setIP"))
			{
				ESP_LOGW(TAG2, "Receiving Data");

				// do data handling it a bit differently for ip adress:

				// length of buffer (inbuf) with data @ end
				int len = (int)inbuf->p->len;
				// pick out last two chars (which is the length of ip adress)
				int ipLength = (buf[len - 2] - '0') * 10 + (buf[len - 1] - '0');
				// define temporary char array with length of ip adress, plus 1 for 0-terminator
				char tmpCharArray[ipLength + 1];
				// cut out ip adress from buffer and copy into tempArray
				strncpy(tmpCharArray, &buf[len - ipLength - 2], (size_t)ipLength);
				// assign null-terminator so that array is seen as a string
				tmpCharArray[ipLength] = '\0';

				// ESP_LOGE(TAG2, "inbuf: %s\n", buf);
				// ESP_LOGE(TAG2, "len: %i\n", len);
				// ESP_LOGE(TAG2, "ipLength: %i\n", ipLength);
				// ESP_LOGE(TAG2, "myArray: %s\n",tmpCharArray);

				// copy ip adress to wifi_fb
				wifi_fb[0].ip.addr = ipaddr_addr(tmpCharArray);
				// signal via event
				xEventGroupSetBits(wifi_event_group, BIT_SET_IP_ADRESS);
				// output ip adress
				u8_t *ip_display = (void *)&wifi_fb[0].ip.addr;
				ESP_LOGW(TAG, "New IP-Adress: %d.%d.%d.%d", ip_display[0], ip_display[1], ip_display[2], ip_display[3]);

				// respond with header
				netconn_write(conn, HTML_HEADER, sizeof(HTML_HEADER) - 1, NETCONN_NOCOPY);
				//
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);

				// err_t   netconn_recv(struct netconn *conn, struct netbuf **new_buf);
			}
			// =========================================================================
			// SSID
			else if (strstr(buf, "POST /setSsid"))
			{
				ESP_LOGW(TAG2, "Receiving Data");

				// do data handling like ip adress:

				// length of buffer (inbuf) with data @ end
				int len = (int)inbuf->p->len;
				// pick out last two chars (which is the length of ssid)
				int ssidLength = (buf[len - 2] - '0') * 10 + (buf[len - 1] - '0');
				// define temporary char array with length of ssid, plus 1 for 0-terminator
				char tmpCharArray[ssidLength + 1];
				// cut out ssid from buffer and copy into tempArray
				strncpy(tmpCharArray, &buf[len - ssidLength - 2], (size_t)ssidLength);
				// assign null-terminator so that array is seen as a string
				tmpCharArray[ssidLength] = '\0'; // -> think this is actually "undefined behaviour"...

				// ESP_LOGE(TAG2, "inbuf: %s\n", buf);
				// ESP_LOGE(TAG2, "len: %i\n", len);
				ESP_LOGW(TAG2, "ssidLength: %i", ssidLength);
				ESP_LOGW(TAG2, "myArray: %s", tmpCharArray);

				// copy ssid to wifi_fb
				strncpy(wifi_fb[0].ssid, tmpCharArray, ssidLength + 1); // +1 for \0

				// signal via event
				xEventGroupSetBits(wifi_event_group, BIT_SET_IP_ADRESS);
				// output ssid adress
				ESP_LOGE(TAG2, "copied ssid: %s", wifi_fb[0].ssid);

				// respond with header
				netconn_write(conn, HTML_HEADER, sizeof(HTML_HEADER) - 1, NETCONN_NOCOPY);
				//
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);

				// err_t   netconn_recv(struct netconn *conn, struct netbuf **new_buf);
			}
			// =========================================================================
			// hdrives request change to Manual Mode
			else if (strstr(buf, "POST /controlMode0"))
			{
				ESP_LOGW(TAG2, "Received request to change control Mode to 0 (manual)");
				// tell via event that control Mode shpuld change
				xEventGroupSetBits(wifi_event_group, BIT_REQ_CHANGE_TO_MODE_0);
				// respond with header
				netconn_write(conn, HTML_HEADER, sizeof(HTML_HEADER) - 1, NETCONN_NOCOPY);
				//
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);
			}
			// =========================================================================
			// hdrives request change to Manual Mode
			else if (strstr(buf, "POST /controlMode1"))
			{
				ESP_LOGW(TAG2, "Received request to change control Mode to 1 (auto/mean)");
				// tell via event that control Mode should change
				xEventGroupSetBits(wifi_event_group, BIT_REQ_CHANGE_TO_MODE_1);
				// respond with header
				netconn_write(conn, HTML_HEADER, sizeof(HTML_HEADER) - 1, NETCONN_NOCOPY);
				//
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);
			}
			// =========================================================================
			// request change to Monitoring Mode
			else if (strstr(buf, "POST /monitoringMode0"))
			{
				ESP_LOGW(TAG2, "Received request to change monitoring Mode to 0");
				// write monitoring mode directly into wifi struct, to be stored in nvs
				wifi_fb[0].monitoring_mode = 0;
				// respond with header
				netconn_write(conn, HTML_HEADER, sizeof(HTML_HEADER) - 1, NETCONN_NOCOPY);
				//
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);
			}
			// =========================================================================
			// request change to Monitoring Mode
			else if (strstr(buf, "POST /monitoringMode1"))
			{
				ESP_LOGW(TAG2, "Received request to change monitoring Mode to 1");
				// tell via event that monitoring Mode should change
				wifi_fb[0].monitoring_mode = 1;
				// respond with header
				netconn_write(conn, HTML_HEADER, sizeof(HTML_HEADER) - 1, NETCONN_NOCOPY);
				//
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);
			}
			// =========================================================================
			// start calibration
			else if (strstr(buf, "POST /calibrateEncoder0"))
			{
				ESP_LOGW(TAG2, "Received request to stop calibration of encoder");
				// tell via event that calibration should start
				wifi_fb[0].calibrate_enc = false;
				// xEventGroupSetBits(wifi_event_group,BIT_STOP_CALIB);
				//  respond with header
				netconn_write(conn, HTML_HEADER, sizeof(HTML_HEADER) - 1, NETCONN_NOCOPY);
				//
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);
			}
			// =========================================================================
			// start calibration
			else if (strstr(buf, "POST /calibrateEncoder1"))
			{
				ESP_LOGW(TAG2, "Received request to start calibration of encoder");
				// tell via event that calibration should start
				wifi_fb[0].calibrate_enc = true;
				// xEventGroupSetBits(wifi_event_group,BIT_START_CALIB);
				//  respond with header
				netconn_write(conn, HTML_HEADER, sizeof(HTML_HEADER) - 1, NETCONN_NOCOPY);
				//
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);
			}
			// =========================================================================
			// set chart Y Limit
			else if (strstr(buf, "POST /chartYLimit"))
			{
				ESP_LOGW(TAG2, "Receiving Data");
				// get length of Poststring
				int length = (int)sizeof("POST /chartYLimit");
				// first 3 bytes is chartYLimit
				int extractedData = ((buf[length - 1] - '0') * 100 + (buf[length] - '0') * 10 + (buf[length + 1] - '0'));
				// copy to global struct
				wifi_fb[0].chartYLimit = (uint8_t)extractedData;
				//
				ESP_LOGW(TAG2, "New Chart Y Limit: %i", extractedData);
				//
				// respond with header
				netconn_write(conn, HTML_HEADER, sizeof(HTML_HEADER) - 1, NETCONN_NOCOPY);
				//
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);
			}
			// =========================================================================
			// synch time
			else if (strstr(buf, "POST /synchTimeStamp"))
			{
				ESP_LOGW(TAG2, "Receiving Data");
				// printf("%s\n",buf);
				// get length of Poststring
				int length = (int)sizeof("POST /synchTimeStamp");
				// first 2 bytes is the day
				int extractedDay = ((buf[length - 1] - '0') * 10 + (buf[length] - '0'));
				// next 2 bytes is the Month
				int extractedMonth = ((buf[length + 1] - '0') * 10 + (buf[length + 2] - '0'));
				// next 4 bytes is the Year
				int extractedYear = ((buf[length + 3] - '0') * 1000 + (buf[length + 4] - '0') * 100 + (buf[length + 5] - '0') * 10 + (buf[length + 6] - '0'));
				// next 2 bytes is the hour
				int extractedHour = ((buf[length + 7] - '0') * 10 + (buf[length + 8] - '0'));
				// next 2 bytes is the minute
				int extractedMinute = ((buf[length + 9] - '0') * 10 + (buf[length + 10] - '0'));
				// next 2 bytes is the Second
				int extractedSecond = ((buf[length + 11] - '0') * 10 + (buf[length + 12] - '0'));
				//
				ESP_LOGW(TAG2, "received Date & Time: %i.%i.%i, %i:%i:%i", extractedDay, extractedMonth, extractedYear, extractedHour, extractedMinute, extractedSecond);
				//
				// synchronize time offset
				time_t receivedOffset = esp_time_calculate_offset(extractedYear, extractedMonth, extractedDay, extractedHour, extractedMinute, extractedSecond);
				ESP_LOGW(TAG2, "calculated offset: %ld", receivedOffset);
				esp_time_synch_offset_error(receivedOffset);
				// refresh time at this point
				esp_time_synch_offset();
				esp_time_estimate();
				// signal nvs to store current offset!
				xEventGroupSetBits(wifi_event_group, BIT_NEW_TIME_OFFSET_RECEIVED);

				// respond with header
				netconn_write(conn, HTML_HEADER, sizeof(HTML_HEADER) - 1, NETCONN_NOCOPY);
				//
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);
			}
			// =========================================================================
			// Reset Diagnose BitsDiagnose
			else if (strstr(buf, "POST /sendDiagInfo"))
			{
				ESP_LOGW(TAG2, "Receiving Data");
				ESP_LOGW(TAG2, "Resetting diagnostic bits");
				// get length of Poststring
				int length = (int)sizeof("POST /sendDiagInfo");
				// to start, just reset all diagnostic bits
				xEventGroupClearBits(diagnose_event_group, 0xF);
				//
				// respond with header
				netconn_write(conn, HTML_HEADER, sizeof(HTML_HEADER) - 1, NETCONN_NOCOPY);
				//
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);
			}
			// =========================================================================
			// IOS test
			else if (strstr(buf, "GET /iosTest"))
			{
				ESP_LOGE(TAG2, "Requesting ios Test data \n");

				int i;
				int idata;
				int idata2;

				// testdata
				// int test_idx = 254;
				// float test_array[254];
				// for(i=0;i<test_idx;i++){
				//	test_array[i] = i;
				//}

				// fill buffer with Sensor 1 data
				// for (i = 0; i < dataAcq[1].idx; i++)
				// {
				// 	// convert float datapoint to int, because max 50cN, we stay under 255 (1 byte),
				// 	// this means we use only first byte of every idata (int) value.
				// 	//
				// 	// convert one float value to int, 2 bytes, <255
				// 	idata = (int)dataAcq[0].valid_data[i];
				// 	// convert to char data and copy first byte into buffer
				// 	// outbuf[i] = ((char *)&idata)[0];
				// 	// do the same for sensor 2 data
				// 	idata2 = (int)dataAcq[1].valid_data[i];
				// 	// outbuf[i + dataAcq[1].idx] = ((char *)&idata2)[0];
				// }

				// fill buffer with Sensor 2 data
				// for(i=dataAcq[0].idx;i<(2*dataAcq[0].idx);i++){
				//	idata = (int)dataAcq[0].valid_data[i];
				//	outbuf[i] = ((char*)&idata)[0];
				//}

				char outbuf_ios[10];
				outbuf_ios[0] = "0";
				// outbuf_ios[1] = "0";

				netconn_write(conn, outbuf_ios, sizeof(outbuf_ios), NETCONN_NOCOPY);
				// netconn_write(conn, outbuf, 4, NETCONN_NOCOPY);

				//
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);

				// err_t   netconn_recv(struct netconn *conn, struct netbuf **new_buf);
			}
			//
			else
			{
				ESP_LOGI(TAG2, "Unknown request");
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);
			}
		}
		else
		{
			ESP_LOGI(TAG2, "Unknown request (empty?...)");
			netconn_close(conn);
			netconn_delete(conn);
			netbuf_delete(inbuf);
		}
	}
	else
	{ // if err!=ERR_OK
		ESP_LOGI(TAG2, "error on read, closing connection");
		netconn_close(conn);
		netconn_delete(conn);
		netbuf_delete(inbuf);
	}
}
// =============================================================================================================
// handles clients when they first connect. passes to a queue
static void server_task(void *pvParameters)
{
	const static char *TAG2 = "server_task";
	struct netconn *conn, *newconn;
	static err_t err;
	client_queue = xQueueCreate(client_queue_size, sizeof(struct netconn *));

	conn = netconn_new(NETCONN_TCP);
	netconn_bind(conn, NULL, 80);
	netconn_listen(conn);
	ESP_LOGI(TAG2, "server listening");
	do
	{
		err = netconn_accept(conn, &newconn);
		// ESP_LOGI(TAG2,"new client");
		if (err == ERR_OK)
		{
			// ESP_LOGI(TAG2, "newconn->socket: %d", newconn->socket);
			xQueueSendToBack(client_queue, &newconn, portMAX_DELAY);
			// http_serve(newconn);
		}
		vTaskDelay(pdMS_TO_TICKS(1));
	} while (err == ERR_OK);
	netconn_close(conn);
	netconn_delete(conn);
	ESP_LOGE(TAG2, "task ending, rebooting board");
	esp_restart();
}
// receives clients from queue, handles them
static void server_handle_task(void *pvParameters)
{
	const static char *TAG2 = "server_handle_task";
	struct netconn *conn;
	ESP_LOGI(TAG2, "task starting");
	for (;;)
	{
		xQueueReceive(client_queue, &conn, portMAX_DELAY);
		if (!conn)
			continue;
		http_serve(conn);
	}
	vTaskDelete(NULL);
}
// handles websocket events
void websocket_callback(uint8_t num, WEBSOCKET_TYPE_t type, char *msg, uint64_t len)
{
	const static char *TAG2 = "websocket_callback";
	//	int value;

	switch (type)
	{
	case WEBSOCKET_CONNECT:
		ESP_LOGI(TAG2, "client %i connected!", num);
		break;
	case WEBSOCKET_DISCONNECT_EXTERNAL:
		ESP_LOGI(TAG2, "client %i sent a disconnect message", num);
		//		led_duty(0);
		break;
	case WEBSOCKET_DISCONNECT_INTERNAL:
		ESP_LOGI(TAG2, "client %i was disconnected", num);
		break;
	case WEBSOCKET_DISCONNECT_ERROR:
		ESP_LOGI(TAG2, "client %i was disconnected due to an error", num);
		//		led_duty(0);
		break;
	case WEBSOCKET_TEXT:
		//		if(len) {
		//			switch(msg[0]) {
		//			case 'L':
		//				if(sscanf(msg,"L%i",&value)) {
		//					ESP_LOGI(TAG2,"LED value: %i",value);
		//					led_duty(value);
		//					ws_server_send_text_all_from_callback(msg,len); // broadcast it!
		//				}
		//			}
		//		}
		break;
	case WEBSOCKET_BIN:
		ESP_LOGI(TAG2, "client %i sent binary message of size %i:\n%s", num, (uint32_t)len, msg);
		break;
	case WEBSOCKET_PING:
		ESP_LOGI(TAG2, "client %i pinged us with message of size %i:\n%s", num, (uint32_t)len, msg);
		break;
	case WEBSOCKET_PONG:
		ESP_LOGI(TAG2, "client %i responded to the ping", num);
		break;
	}
}
// =============================================================================================================
// fuction to initialize all gpio's
// =============================================================================================================
void wifi_init_gpio()
{
	// configure DigIns
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
	io_conf.pull_down_en = 1;
	io_conf.pull_up_en = 0;
	gpio_config(&io_conf);
}
// =============================================================================================================
// main wifi task
void wifi_task(void *arg)
{
	// init GPIO and vars
	wifi_init_gpio();
	int ap_step = 0; // wifi in ap is initialized without wifi_start()!
	int ap_restart_counter = 0;
	iWifiMode = WIFI_MODE_AP;
	// initialize Wifi according to mode
	switch (iWifiMode)
	{
	case WIFI_MODE_STA:
		// in sta mode, wifi gets started in init
		wifi_init_sta();
		break;
	case WIFI_MODE_AP:
		// in ap mode, wifi does not get started yet
		wifi_init_ap();
	default:
		break;
	}
	if (iWifiMode == 1)
	{
		ESP_LOGW(TAG, "WIFI INITIALIZED IN STA MODE (STATION)");
	}
	else if (iWifiMode == 2)
	{
		ESP_LOGW(TAG, "WIFI INITIALIZED IN AP MODE (ACCESSPOINT)");
	}
	// start webserver, no matter which mode
	ws_server_stop();
	ws_server_start();
	// create needed tasks, no matter which mode
	xTaskCreatePinnedToCore(&server_task, "server_task", 3000, NULL, 5, NULL, 0);
	xTaskCreatePinnedToCore(&server_handle_task, "server_handle_task", 8192, NULL, 5, &server_handle_task_handle, 0);
	// wifi loop
	while (1)
	{
		// check if ip adress has changed
		if (check_clear_event_bit(wifi_event_group, BIT_SET_IP_ADRESS))
		{
			ESP_LOGE(TAG, "change of IP Adress, SSID or Password requested");
			// wait another 3 seconds before restart (so that remanent variables are stored)
			vTaskDelay((1000 / portTICK_PERIOD_MS));
			ESP_LOGE(TAG, "Turning off Wifi");
			esp_wifi_stop();
			vTaskDelay((3000 / portTICK_PERIOD_MS));
			ESP_LOGE(TAG, "Restarting ESP");
			esp_restart();
		};
		// user events according to mode
		switch (iWifiMode)
		{
		case WIFI_MODE_STA:
			// in sta mode, try to connect after startup, or reconnect if disconnected
			if (bGlobalConnectionStatus == false)
			{
				ESP_LOGE(TAG, "trying to connect/reconnect");
				wifi_try_connect_sta();
			}
			break;
		case WIFI_MODE_AP:
			// in ap mode, start/stop wifi according to user. Restart wifi if not connected for a while
			switch (ap_step)
			{
			// ap is deactivated
			case 0:
				ESP_LOGW(TAG, "wifi is in mode ap and currently deactivated");
				// if user event to activate:
				if (gpio_get_level(GPIO_WIFI_ENABLE))
				{
					ESP_LOGW(TAG, "activating wifi in AP Mode");
					// restart wifi
					// ESP_LOGW(TAG, "starting up wifi");
					esp_wifi_start();
					// wait a second
					// vTaskDelay((1000 / portTICK_PERIOD_MS));
					// ESP_LOGW(TAG, "starting webserver");
					// ws_server_start();
					ap_step = 1;
				}
				break;
			// ap is activated
			case 1:
				// if user event to deactivate:
				if (!gpio_get_level(GPIO_WIFI_ENABLE))
				{
					ESP_LOGW(TAG, "deactivating wifi in AP Mode");
					// ESP_LOGW(TAG, "stopping webserver");
					// ws_server_stop();
					// wait a second
					// vTaskDelay((1000 / portTICK_PERIOD_MS));
					// stop wifi
					// ESP_LOGW(TAG, "stopping wifi");
					esp_wifi_stop();
					ap_step = 0;
				};
				// if AP is activated, but no connection for xx minutes, restart wifi
				if (bGlobalConnectionStatus == false)
				{
					ap_restart_counter++;
					if (ap_restart_counter > 600) // restart every 10 minutes
					{
						ap_restart_counter = 0;
						ESP_LOGW(TAG, "No clients heard for quite a while. Restarting Wifi!");
						// ws_server_stop();
						// wait a second
						// vTaskDelay((1000 / portTICK_PERIOD_MS));
						// stop wifi
						// ESP_LOGW(TAG, "stopping wifi");
						esp_wifi_stop();
						ap_step = 0;
					}
				}
				else
				{
					ap_restart_counter = 0;
				}
				break;
			default:
				break;
			}
		default:
			break;
		}

		// ESP_LOGI(TAG, "8 Bit min free: %d = free: %d", heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT), heap_caps_get_free_size(MALLOC_CAP_8BIT));
		// ESP_LOGI(TAG, "32 Bit min free: %d = free: %d", heap_caps_get_minimum_free_size(MALLOC_CAP_32BIT), heap_caps_get_free_size(MALLOC_CAP_32BIT));
		// ESP_LOGI(TAG, "INTERNAL min free: %d = free: %d", heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL), heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
		// ESP_LOGI(TAG, "DMA min free: %d = free: %d", heap_caps_get_minimum_free_size(MALLOC_CAP_DMA), heap_caps_get_free_size(MALLOC_CAP_DMA));
		// ESP_LOGI(TAG, "HEAP min free: %d = free: %d\n", esp_get_minimum_free_heap_size(), esp_get_free_heap_size());

		// heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM);

		// printf("wifi: %i\n",uxTaskGetStackHighWaterMark(fb_wifi_task_handle));
		// printf("hdrive: %i\n",uxTaskGetStackHighWaterMark(hdrive_task_handle));
		// printf("dataAcq: %i\n",uxTaskGetStackHighWaterMark(dataAcq_task_handle));
		// printf("mean: %i\n",uxTaskGetStackHighWaterMark(mean_control_task_handle));
		// printf("nvs: %i\n",uxTaskGetStackHighWaterMark(nvs_task_handle));
		// printf("server_handle: %i\n",uxTaskGetStackHighWaterMark(server_handle_task_handle));

		// printf("nvs: %i\n",esp_get_free_heap_size());
		// printf("nvs: %i\n",esp_get_minimum_free_heap_size());
		// ESP_LOGE(TAG, "=== 1s? ===");

		// ESP_LOGE(TAG, "GPIO: %d",gpio_get_level(GPIO_WIFI_ENABLE));

		//..
		vTaskDelay((1000 / portTICK_PERIOD_MS));
	}
}
