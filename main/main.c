/*  FADENBREMSE
PCB V1.1, November 2021, V. Maag
Build upon Firmware for PCB V1.0 (PreSeries)

General ToDo's for V2.0

- Use global filter functions in data_acq, mean_control etc.
    --> change all filtering to: yk = xk b0/a0 + xk-1 b1/a0 ... etc
    --> we had all kinds of filters, try to stick to one method from now on
    --> move filters to my maag-lib, add notch etc.
    --> leave implemented ones of course, only switch once I have tested and verified new ones are solid
- kill color sensor stuff in mean_control
- use own function get_clear_event_bit
- Gloabl HDRIVE-MAX-ANGLE, MIN-ANGLE (did this once before, but lost progress because not consistent in versioning!!!)
- get machine angle with .1° precision (currently, only 1° precision)
- wifi: sendhdrivesetpoint, use 1 single post-command and not 2!
- put all includes in projectdefs

- use event group hdrive vor current_control_mode?

- Configuration in HMI to switch Sensor direction!

- HMI: if connection timeout for 10s, after connection has been established, then maybe don't send anymore, close connections
and say "please refresh page"!!!
  --> we actually count up until 15 times connection has been blocked. We then release blocking and try to getArray/stuff again.
  I think we should count how many times we try and connect (so a counter inside this counting of 15 times). If, lets say, 
  5-6 times no success, we block all kind of communication and ask in HMI to refresh page

  - Also, maybe first: only send stuff (updating parameters etc.) if in fact global connection status is green. This way we dont send stuff
  into no-mans-land!

  - jumping (rauschen) from 0->360->0 seems to leed to speed > 0 --> machine running... probably a bug in calculating
  velocity (wrapped velocity)

  - hmi: wait at load for 1-2s (init, rest for some time until starting requests)

  - wifi: allow only one single connection, i.e. allowed stations in ap mode: 1!

  - core 0 panick: test some stuff with wifi (GET-Commands). Maybe we do some silly shit when idx is 1, or 0, or whatever
  - i think the error may be some pointer looking at something that isn't there anymore...

  - hmi: maybe send only every second datapoint, as to save throughput and make life easier for esp

  - make startup faster if esp32 crashes --> setup tasks faster, don't wait 3 seconds to enable hdrives...

*/

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "driver/periph_ctrl.h"
#include "driver/timer.h"

#include "driver/uart.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "driver/rmt.h"

#include "esp_wifi.h"
#include "lwip/ip4_addr.h"
#include "tcpip_adapter.h"

#include "fb_projdefs.h"
#include "fb_wifi.h"
#include "fb_hdrive.h"
#include "fb_data_acq.h"
#include "fb_mean_control.h"
#include "fb_nvs.h"
#include "fb_blinker.h"
#include "fb_color_sensor.h"
#include "fb_sd_card.h"
#include "fb_esp_time.h"

static const char *TAG = "main";

// =========================================================================
// Global functions to handle event bits easier
// =========================================================================
// checks eventgroup for eventbit, clears eventbit if set. returns true
// if eventbit was set & cleared
bool check_clear_event_bit(EventGroupHandle_t eventgroup, EventBits_t eventbit)
{
  //
  if ((xEventGroupGetBits(eventgroup) & eventbit) == eventbit)
  {
    xEventGroupClearBits(eventgroup, eventbit);
    return true;
  }
  else
  {
    return false;
  }
}
// checks eventgroup for eventbit returns true if eventbit was set
bool check_event_bit(EventGroupHandle_t eventgroup, EventBits_t eventbit)
{
  //
  if ((xEventGroupGetBits(eventgroup) & eventbit) == eventbit)
  {
    return true;
  }
  else
  {
    return false;
  }
}
// sets eventbit of event group (just used for easy/consistent nameing...)
void set_event_bit(EventGroupHandle_t eventgroup, EventBits_t eventbit)
{
  xEventGroupSetBits(eventgroup, eventbit);
}
// clears eventbit of event group (just used for easy/consistent nameing...)
void clear_event_bit(EventGroupHandle_t eventgroup, EventBits_t eventbit)
{
  xEventGroupClearBits(eventgroup, eventbit);
}
// =========================================================================
// Global digital filter functions
// =========================================================================
// filter pt1 using zoh method
void filter_pt1_zoh(filter_pt1_zoh_t *data)
{
  // filter value
  data->output = data->input * data->coeff_A + data->input_old * data->coeff_B;
  data->input_old = data->output;
}
// dt1 with tustin method
// y_k+1 = x_k+1 * b1/a1 + x_k * b0/a1 - y_k * a0/a1
void filter_dt1(filter_dt1_t *data)
{
  // pass current value (x_k+1)
  data->x[1] = data->input;
  // calculate current output (y_k+1)
  data->y[1] = (data->x[1] * data->b[1] / data->a[1]) + (data->x[0] * data->b[0] / data->a[1]) - (data->y[0] * data->a[0] / data->a[1]);
  // timeshift
  data->y[0] = data->y[1];
  data->x[0] = data->x[1];
  // copy output value
  data->output = data->y[1];
}
// dt1 with tustin method, input wrapped around 360°
// y_k+1 = x_k+1 * b1/a1 + x_k * b0/a1 - y_k * a0/a1
void filter_dt1_360(filter_dt1_t *data)
{
  // in can be shown that discrete TF = s/T1s+1 is:
  //  y_k+1 = x_k+1 * b1/a1 + x_k * b0/a1 - y_k * a0/a1
  // we see that in the case of a DT1, b0 = -b1. So it follows:
  // y_k+1 = x_k+1 * b1/a1 - x_k * b1/a1 - y_k * a0/a1, further:
  // y_k+1 = (x_k+1 - x_k) * b1/a1 - y_k * a0/a1
  // ... so we actually only need the difference (x_k+1 - x_k) between last two values
  // This means we can wrap difference around 360°

  // pass current value (x_k+1)
  data->x[1] = data->input;
  data->test = data->x[0];

  // calculate difference. If dif < -180, we wrapp angle
  float dif = (data->x[1] - data->x[0]);
  float dif_wrap;
  if (dif < -180)
  {
    dif_wrap = dif + 360.0;
  }
  else
  {
    dif_wrap = dif;
  }
  // as dif_wrap = (x_k+1 - x_k), we write:
  // y_k+1 = (x_k+1 - x_k) * b1/a1 - y_k * a0/a1
  data->y[1] = (dif_wrap * data->b[1] / data->a[1]) - (data->y[0] * data->a[0] / data->a[1]);
  // data->y[1] = (data->x[1] * data->b[1] / data->a[1]) + (data->x[0] * data->b[0] / data->a[1]) - (data->y[0] * data->a[0] / data->a[1]);
  // timeshift
  data->y[0] = data->y[1];
  data->x[0] = data->x[1];
  // copy output value
  data->output = data->y[1];
}

void app_main()
{
  // =====================================================================
  // Initialize NVS
  // =====================================================================
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
  //
  // =====================================================================
  // init global variables
  // =====================================================================
  // dataAcq
  dataAcq_event_group = xEventGroupCreate();
  for (int i = 0; i < DATA_ACQ_NUM_OF_CHANNELS; i++)
  {
    dataAcq[i].idx = 251;
    dataAcq[i].act_value = 0;
    dataAcq[i].act_value_filtered = 0;
    dataAcq[i].mean = 0;
    dataAcq[i].integral_approx = 0;
    dataAcq[i].valid_channel = 0;
    for (int ii = 0; ii < DATA_ACQ_MAX_SAMPLES_PER_CYCLE; ii++)
    {
      dataAcq[i].data[ii] = 0;
      dataAcq[i].valid_data[ii] = 13.54;
    }
    for (int nn = 0; nn < DATA_ACQ_NUM_OF_FILTER_COEFFICIANTS; nn++)
    {
      dataAcq[i].act_value_filtered_old_array[nn] = 0;
    }
  }
  hdrive_event_group_array[0] = xEventGroupCreate();
  hdrive_event_group_array[1] = xEventGroupCreate();
  //
  // wifi
  for (int i = 0; i < DATA_ACQ_NUM_OF_CHANNELS; i++)
  {
    wifi_fb[i].setpoint_hdrive = 51;
    wifi_fb[i].setpoint_mean_control = 16;
    wifi_fb[i].monitoring_mode = 0;
    wifi_fb[i].ip.addr = ipaddr_addr("192.168.0.254");
    //wifi_fb[i].ssid = "myssid";
    strncpy(wifi_fb[i].ssid, "myssid", strlen("myssid") + 1); // +1 for \0
    wifi_fb[i].angleOffset = 0;
    wifi_fb[i].calibrate_enc = false;
    wifi_fb[i].hdrive_closing_time = 250;
    wifi_fb[i].hdrive_opening_time = 70;
    wifi_fb[i].chartYLimit = 36;
  }
  //ESP_LOGE(TAG, "___________________ WROTE STRING: %s", wifi_fb[0].ssid);
  wifi_event_group = xEventGroupCreate();
  //
  // mean control
  for (int i = 0; i < DATA_ACQ_NUM_OF_CHANNELS; i++)
  {
    mean_control[i].actual_single_mean_value = 15;
    mean_control[i].actual_mean_value = 15;
    mean_control[i].actual_mean_value_old = 15;
    mean_control[i].control_integral = 160;
    mean_control[i].mean_control_output = 170; // hdrive will jump to this angle at first startup!
    mean_control[i].mean_setpoint = 15;
    mean_control[i].display_mean = 15;
    mean_control[i].display_mean_old = 15;
  }
  current_control_mode[0] = CONTROL_MODE_MANUAL;
  current_control_mode[1] = CONTROL_MODE_MANUAL;
  //
  // global loom object
  loom.actualAngle = 0;
  loom.angleOffset = 0;
  loom.ip.addr = wifi_fb[0].ip.addr;
  loom.monitoring_mode = wifi_fb[0].monitoring_mode;
  loom.enc_min = 500;
  loom.enc_max = 3000;
  loom.machine_running = false;
  //
  // global timer
  global_timer_group = TIMER_GROUP_1;
  global_timer_idx = TIMER_1;
  timer_config_t config;
  config.divider = 16;
  config.counter_dir = TIMER_COUNT_UP;
  config.counter_en = TIMER_START;
  config.alarm_en = TIMER_ALARM_DIS;
  config.intr_type = TIMER_INTR_LEVEL;
  config.auto_reload = false;
  timer_init(global_timer_group, global_timer_idx, &config);
  timer_start(global_timer_group, global_timer_idx);
  //
  // diagnose functions
  diagnose_event_group = xEventGroupCreate();
  xEventGroupSetBits(diagnose_event_group,BIT_DIAGNOSE_0_ESP_RESTARTED); // at startup, set first event bit --> signal a restart!
  // 
  // =====================================================================
  // initialize nvs data (remanent variables)
  // =====================================================================
  init_nvs_data();
  //
  // =====================================================================
  // create tasks
  // =====================================================================

  // CORE 0
  ESP_LOGW(TAG, "Creating tasks on core 0");
  
  xTaskCreatePinnedToCore(nvs_task, "nvs_task", 2048, (void *)0, configMAX_PRIORITIES - 20, &nvs_task_handle, 0);

  vTaskDelay(2500);
  xTaskCreatePinnedToCore(wifi_task, "wifi_task", 4096, (void *)0, configMAX_PRIORITIES, &fb_wifi_task_handle, 0);

  vTaskDelay(2500);
  xTaskCreatePinnedToCore(sd_card_task, "sd_card_task", 8192, (void *)0, configMAX_PRIORITIES - 1, &sd_card_task_handle, 0);

  vTaskDelay(2500);
  xTaskCreatePinnedToCore(blinker_task, "blinker_task", 2048, (void *)0, configMAX_PRIORITIES - 2, &blinker_task_handle, 0);

  // CORE 1
  ESP_LOGW(TAG, "creating tasks on core 1");

  vTaskDelay(2500);
  xTaskCreatePinnedToCore(dataAcq_task, "dataAcq_task", 4096, (void *)0, configMAX_PRIORITIES - 1, &dataAcq_task_handle, 1);

  vTaskDelay(2500);
  xTaskCreatePinnedToCore(hdrive_task, "hdrive_task", 4096, (void *)0, configMAX_PRIORITIES - 2, &hdrive_task_handle, 1);

  vTaskDelay(2500);
  xTaskCreatePinnedToCore(mean_control_task, "mean_control_task", 4096, (void *)0, configMAX_PRIORITIES - 3, &mean_control_task_handle, 1);

  vTaskDelay(2500);
  xTaskCreatePinnedToCore(esp_time_task, "esp_time_task", 2048, (void *)0, configMAX_PRIORITIES - 4, &esp_time_task_handle, 1);

  // obsolete
  // xTaskCreatePinnedToCore(color_sensor_task, "color_sensor_task", 4096, (void *)0, 1, &color_sensor_task_handle, 1);
}
