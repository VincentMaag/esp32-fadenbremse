/*
    ...

*/
#ifndef __FB_PROJDEFS_H__
#define __FB_PROJDEFS_H__
//
#include "freertos/event_groups.h"
// =================================================================================================
// Global defs
// =================================================================================================
#define DATA_ACQ_MAX_SAMPLES_PER_CYCLE  (500)
#define DATA_ACQ_NUM_OF_CHANNELS        (2)
#define DATA_ACQ_NUM_OF_FILTER_COEFFICIANTS (4)
// =================================================================================================
// Global variable ("object") for data acquisition
// =================================================================================================
typedef struct{
    float data[DATA_ACQ_MAX_SAMPLES_PER_CYCLE];  // data of one full cycle [cN]
    float valid_data[DATA_ACQ_MAX_SAMPLES_PER_CYCLE]; // valid full data of one cycle
    int idx;                // amount of datapoints in valid dataset
    float mean;             // last valid single mean value [cN]
    float act_value;        // current actual sensor value [cN]
    float act_value_filtered; // current value filtered
    float act_value_filtered_old_array[DATA_ACQ_NUM_OF_FILTER_COEFFICIANTS]; // array of last actual (filtered) values
    float integral_approx;  // [cNs] approximated integral of one full measurement (i.e. 254 or 127 dataPoints)
    int valid_channel;      // only index 0 is relevant --> dataAcq[0].valid_channel is 0 for channel 0, 1 for channel 1
} dataAcq_t;
//
dataAcq_t dataAcq[DATA_ACQ_NUM_OF_CHANNELS];
//
// event group for data Acquisition. initialized in main()
EventGroupHandle_t dataAcq_event_group; 
// define event bits
#define BIT_0_HALLS             (1 << 0) // Bits if HallS was triggered (isr)
//#define BIT_VALID_MEASUREMENT   (1 << 1) // Bit if new valid measurement has been completed --> bit is set in hdrive event group
#define BIT_DATA_ACQ_IS_ACTIVE  (1 << 2) // Bit if dataAcq is acquiring data
#define BIT_DATA_ACQ_IS_IDLE    (1 << 3) // Bit if dataAcq is idle
// =================================================================================================
// Global variable ("object") for wifi
// =================================================================================================
#include "lwip/ip4_addr.h"
typedef struct{
    float setpoint_hdrive;                  // setpoint for MANUAL hdrive position from wifi, [%]
    float setpoint_mean_control;            // setpoint for AUTO mean control from wifi, [cN]
    uint8_t monitoring_mode;                // demanded display mode for montitoring (index 0!): 0=only monitoring, 1=monitoring & brakes & control
    ip4_addr_t ip;                          // demanded ip adress (wifi[0]=ip, wifi[1]=submask --> optional, not implemented)
    char ssid[32];                          // demanded SSID
    uint16_t angleOffset;                   // demanded angle offset from wifi (index 0!)
    bool calibrate_enc;                     // start/stop calibration of encoder
    uint16_t hdrive_opening_time;           // demanded time that hdrive should open (machine-angle) [°]
    uint16_t hdrive_closing_time;           // demanded time that hdrive should close (machine-angle) [°]
    uint8_t chartYLimit;                    // Y-Axis limit in chart on webpage. max 256 cN
} wifi_fb_t;
//
wifi_fb_t wifi_fb[2];
//
EventGroupHandle_t wifi_event_group;
#define BIT_REQ_CHANGE_TO_MODE_0     (1 << 0)    // Bit 0 to request change to control Mode 0 (Manual)
#define BIT_REQ_CHANGE_TO_MODE_1     (1 << 1)    // Bit 1 to request change to control Mode 1 (Auto)
#define BIT_REQ_DISABLE_HDRIVE       (1 << 2)    // Bit 2 to request disable hdrive
#define BIT_REQ_ENABLE_HDRIVE        (1 << 3)    // Bit 3 to request enable hdrive
#define BIT_SET_ANGLE_OFFSET         (1 << 4)    // Bit 4 to request new machine angle offset
#define BIT_START_CALIB              (1 << 5)    // Bit 5 to request start encoder calibration
#define BIT_STOP_CALIB               (1 << 6)    // Bit 6 to request stop encoder calibration
#define BIT_SET_OPENING_CLOSING_TIME (1 << 7)    // Bit 7 to request changing opening & closing time of brakes
#define BIT_SET_IP_ADRESS            (1 << 8)    // Bit 8 to request changing IP Adress of ESP
#define BIT_NEW_TIME_OFFSET_RECEIVED (1 << 9)    // Bit 9 to signal update of time offset

//
#define WIFI_SSID		"myssid"
#define WIFI_PASSWORD	""
// 
//
// =================================================================================================
// Global variable ("object") for hdrive
// =================================================================================================
// struct for hdrive object, with enum etc.
typedef enum{
    HD_DISABLED,
    HD_ENABLE,
    HD_INITIALIZE,
    HD_CLOSING,
    HD_CLOSED,
    HD_OPENING,
    HD_OPEN,
    HD_ERROR,
    HD_REQ_DISABLED     // from hmi requested disable
} hdrive_step_enum_t;
typedef struct{
    float setpointOpen;     // setpoint for open position, in angle [°]
    float setpointClosed;   // setpoint for closed position, in angle [°]
    float actualPosition;
    hdrive_step_enum_t step;
    int trigger_gpio_ch;
    float currentSetpointClosedPercent;    // closed setpoint of hdrive in percent for wifi [%]
    uint16_t hdrive_opening_time;           // time that hdrive should open (machine-angle) [°]
    uint16_t hdrive_closing_time;           // time that hdrive should close (machine-angle) [°]
}hdrive_t;
//
hdrive_t hdrive[2];
//
// event group for each hdrive. initialized in main()
EventGroupHandle_t hdrive_event_group_array[2];
// define event bits for hdrive
//#define BIT_REQ_DISABLE_HDRIVE       (1 << 0)     // Bit 0 to request disable hdrive
#define BIT_CHANGE_SETPOINT_HDRIVE   (1 << 1)       // Bit 1 to request change in hdrive position
#define BIT_CONTROL_MODE_MANUAL       (1 << 2)      // Bit 2 to signal control Mode is Manual
#define BIT_CONTROL_MODE_MEAN         (1 << 3)      // Bit 3 to signal control Mode is mean value control
#define BIT_VALID_MEASUREMENT        (1 << 4)       // Bit 4 if new valid measurement has been completed
#define BIT_HDRIVE_READY_TO_ENABLE   (1 << 5)       // Bit 5 hdrive ready to be enabled
#define BIT_HDRIVE_ALLOWED_TO_ENABLE (1 << 6)       // Bit 6 hdrive allowed to be enabled
#define BIT_HDRIVE_RUNNING            (1 << 7)      // Bit 7 hdrive is running (getting constant triggers)
#define BIT_HDRIVE_CHANGE_AUTO_SETPOINT (1 << 8)    // Bit 8 to signal new auto setpoint was calculated by mean control
//
// =================================================================================================
// Global variable ("object") for mean control
// =================================================================================================
typedef struct{
    float actual_single_mean_value_old; 
    float actual_mean_value_old;
    float actual_single_mean_value; // actual single mean tension value @ 348-352° [cN]
    float actual_mean_value;        // actual filtered mean tension for mean control [cN]
    float actual_mean_a[2];
    float actual_mean_b[2];
    float actual_mean_x[2];
    float actual_mean_y[2];
    float mean_setpoint;
    float mean_setpoint_filtered;
    double a[3]; // array of a coefficiants for setpoint filtering
    double b[3]; // array of b coefficiants for setpoint filtering
    double y[3]; // array of y for setpoint filtering
    double x[3]; // array of x for setpoint filtering
    float mean_control_output;      // [cN]
    float control_integral;         // [cN]
    float display_mean;
    float display_mean_old;
} mean_control_t;
//
mean_control_t mean_control[2];
//
#define CONTROL_MODE_MANUAL     (0)
#define CONTROL_MODE_MEAN       (1)
// current_control_mode could probably be put into hdive_t...
uint8_t current_control_mode[2];   // current wanted control Mode of hdrives (0=manual, 1=mean)
//
// =================================================================================================
// Global variable ("object") for nvs storage
// =================================================================================================
typedef struct {
    uint32_t nvs_Setpoint_Manual;
    uint32_t nvs_Setpoint_Auto;
    uint32_t nvs_control_mode;
    uint32_t nvs_monitoring_mode;                
    uint32_t nvs_ip; 
    uint32_t nvs_angleOffset;
    uint32_t nvs_hdrive_opening_time;           // demanded time that hdrive should open (machine-angle) [°]
    uint32_t nvs_hdrive_closing_time;           // demanded time that hdrive should close (machine-angle) [°]
    int16_t  nvs_enc_min;                       // calibration value for machine angle calculation  
    int16_t  nvs_enc_max;                       // calibration value for machine angle calculation  
    char nvs_ssid[32];                          // ssid of wifi
    uint32_t nvs_chartYLimit;                    // Y-Limit of Chart in browser
    uint64_t nvs_timeOffset;                    // stored time offset
}nvs_data_t;
//
nvs_data_t nvs_data[2];
//
// =================================================================================================
// Global variable ("object") for projectile loom
// =================================================================================================
typedef struct {
    uint8_t monitoring_mode;    // set monitoring mode (maybe not even needed --> wifi_fb[0])
    ip4_addr_t ip;              // set ip adress, NOT USED
    float actualAngle;          // actual machine angle [°]
    uint16_t angleOffset;       // actual angle offset [°]
    int16_t enc_min;            // calibration value for machine angle calculation  
    int16_t enc_max;            // calibration value for machine angle calculation
    bool machine_running;       // status if projectile loom is running or not (true/false)
    float differential;         // simple (positive!) differential of machine angle that is in plausible range
}loom_t;
//
loom_t loom;


// =================================================================================================
// Other global variables (timers, etc.)
// =================================================================================================
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
// global timer
timer_group_t global_timer_group;
timer_idx_t global_timer_idx;

//

// =================================================================================================
// globale Task handlers
// =================================================================================================
TaskHandle_t fb_wifi_task_handle;
TaskHandle_t hdrive_task_handle;
TaskHandle_t dataAcq_task_handle;
TaskHandle_t mean_control_task_handle;
TaskHandle_t nvs_task_handle;
TaskHandle_t blinker_task_handle;
TaskHandle_t color_sensor_task_handle;
TaskHandle_t esp_time_task_handle;
TaskHandle_t sd_card_task_handle;

// =================================================================================================
// digital filter "object" type
// =================================================================================================
typedef struct {
    float input;        // input value of filter
    float output;       // output value of filter
    float coeff_A;      // zoh coefficiant A
    float coeff_B;      // zoh coefficiant B
    float input_old     // memory value
}filter_pt1_zoh_t;

typedef struct {
    float input;        // input value of filter
    float output;       // output value of filter
    float a[2];         // a coefficiants
    float b[2];         // b coefficiants
    float x[2];         // x vector
    float y[2];         // y vector
    float test;
}filter_dt1_t;



// =================================================================================================
// global diagnose bits
// =================================================================================================
EventGroupHandle_t diagnose_event_group;
#define BIT_DIAGNOSE_0_ESP_RESTARTED        (1 << 0)    // 
#define BIT_DIAGNOSE_1_HDRIVE_ENABLED       (1 << 1)    // 
#define BIT_DIAGNOSE_2_SD_CARD_NOT_MOUNTED  (1 << 2)    // 
#define BIT_DIAGNOSE_2_SD_CARD_NO_VALID_FILENAME  (1 << 3)    // 



// =================================================================================================
// global (support) functions
// =================================================================================================
// checks and clears bit from event group
bool check_clear_event_bit(EventGroupHandle_t eventgroup, EventBits_t eventbit); 
// checks bit from event group
bool check_event_bit(EventGroupHandle_t eventgroup, EventBits_t eventbit); 
// sets bit of event group
void set_event_bit(EventGroupHandle_t eventgroup, EventBits_t eventbit);
// clears bit of event group
void clear_event_bit(EventGroupHandle_t eventgroup, EventBits_t eventbit);
//
void filter_pt1_zoh(filter_pt1_zoh_t* data); // filters data of object type filter_pt1_zoh_t
void filter_dt1(filter_dt1_t *data);
void filter_dt1_360(filter_dt1_t *data);



#endif /* __FB_PROJDEFS_H__ */