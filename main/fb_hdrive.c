/*
    ...

*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/ledc.h"

#include "driver/ledc.h"
#include "esp_err.h"

#include "driver/periph_ctrl.h"
#include "driver/timer.h"

#include "fb_projdefs.h"
#include "fb_hdrive.h"

//
static const char *TAG = "fb_hdrive";
//
// define digital Inputs
#define GPIO_CHANNEL_SEL_INPUT GPIO_NUM_35 // GPIO_NUM_32 //GPIO_NUM_35    // hdrive channel select --> with pull-down!!!
#define GPIO_INPUT_PIN_SEL ((1ULL << GPIO_CHANNEL_SEL_INPUT))

// define digital Outputs
#define GPIO_ENABLE_OUTPUT GPIO_NUM_16 // GPIO_NUM_15 //GPIO_NUM_16
#define GPIO_OUTPUT_PIN_SEL ((1ULL << GPIO_ENABLE_OUTPUT))
// define PWM Outputs
#define GPIO_PWM_OUTPUT_CH0 GPIO_NUM_18 // GPIO_NUM_27 //GPIO_NUM_18
#define GPIO_PWM_OUTPUT_CH1 GPIO_NUM_17 // GPIO_NUM_14 //GPIO_NUM_17
#define PWM_NUM_OUTPUT_CHANNELS (2)
// pwm input & output resolution
#define PWM_OUTPUT_RESOLUTION LEDC_TIMER_12_BIT
#define PWM_INPUT_RESOLUTION LEDC_TIMER_12_BIT
// number of pwm input channels
#define GPIO_PWM_INPUT_CH0 GPIO_NUM_19 // GPIO_NUM_4  //GPIO_NUM_19
#define GPIO_PWM_INPUT_CH1 GPIO_NUM_5  // GPIO_NUM_16 //GPIO_NUM_5
#define PWM_NUM_INPUT_CHANNELS (2)
#define PWM_INPUT_FREQ (75.3)
// define hdrive channels
#define HDRIVE_CH0 (0)
#define HDRIVE_CH1 (1)
#define HDRIVE_NUM_OF_CHANNELS (2)
// Hardware timer clock divider
#define TIMER_DIVIDER (16)
#define TIMER_TICK_FREQ (80000000 / 16)
// Global timer group & index for pwm input reading. Timer 1, index 0
timer_group_t timer_group = TIMER_GROUP_1;
timer_idx_t timer_idx = TIMER_0;
// Global led channel "object"
ledc_channel_config_t ledc_channel[PWM_NUM_OUTPUT_CHANNELS];
// Global variables for interrupt handling
uint64_t isrTicksAtRising[PWM_NUM_INPUT_CHANNELS];
uint64_t isrTicksAtFalling[PWM_NUM_INPUT_CHANNELS];
uint64_t isrTickDifference[PWM_NUM_INPUT_CHANNELS];
// array to select gpio number from hdrive channel number (for example in isr)
int gpio_ch_select[HDRIVE_NUM_OF_CHANNELS] = {GPIO_PWM_INPUT_CH0, GPIO_PWM_INPUT_CH1};
//
uint8_t trigChangeMem[2] = {0, 0}; // memory used for trigger change detection (think depricated)

// ========================================================================================
// === PARAMETERS =========================================================================
// ========================================================================================
// measured angles for minimum and maximum force of hdrives
float minimumForceAngle = 170;
float maximumForceAngle = 150;
float absoluteOpenAngle = 185; // 200; //185;
//
float hdriveAngleHysteresis = 10; // [°], hysteresis for opening/Closing angles (prevents bouncing if enocder jitters)
//
uint8_t delayUntilOpen = 10; // [ms] to delay. 10ms leads to around 14° delay @ 254ms/360°
uint8_t delayUntilClose = 2; // 4; // [ms] to delay
//
//
//
//
// ========================================================================================
// fuction to initialize all gpio's, specifically for hdrive
void init_gpio_hdrive()
{
    // configure DigIns
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.pull_down_en = 0; // 1 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    // configure DigOut(s) and initialize "low"
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    // io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    gpio_set_level(GPIO_ENABLE_OUTPUT, 0);
    // prepare and configure pwm ouputs.
    // first, prepare individual configuration of each LED (pwm) channel,
    // we use timer 0, channels 0-3
    for (int i = 0; i < PWM_NUM_OUTPUT_CHANNELS; i++)
    {
        // ledc_channel[i].channel    = LEDC_CHANNEL_0;
        ledc_channel[i].duty = 0;
        // ledc_channel[i].gpio_num   = GPIO_PWM_OUTPUT_CH0;
        ledc_channel[i].speed_mode = LEDC_HIGH_SPEED_MODE;
        ledc_channel[i].timer_sel = LEDC_TIMER_0;
    };
    // configure the ones we actualy use (2 from 4)
    ledc_channel[0].channel = LEDC_CHANNEL_0;
    ledc_channel[0].gpio_num = GPIO_PWM_OUTPUT_CH0;
    ledc_channel[1].channel = LEDC_CHANNEL_1;
    ledc_channel[1].gpio_num = GPIO_PWM_OUTPUT_CH1;
    // next, configure the LED timer itself (timer 0) and initialize
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = PWM_OUTPUT_RESOLUTION,
        .freq_hz = 5000,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0};
    ledc_timer_config(&ledc_timer);
    // now, initialize all configured led pwm channels
    int ch;
    for (ch = 0; ch < PWM_NUM_OUTPUT_CHANNELS; ch++)
    {
        ledc_channel_config(&ledc_channel[ch]);
    }
    // configure free running timer (timer 1 idx 0) and initialize
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_START;
    config.alarm_en = TIMER_ALARM_DIS;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = false;
    timer_init(timer_group, timer_idx, &config);
    timer_start(timer_group, timer_idx);
    // configure and initialize isr
    // first, config gpios
    gpio_set_intr_type(GPIO_PWM_INPUT_CH0, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(GPIO_PWM_INPUT_CH1, GPIO_INTR_ANYEDGE);
    // next, install isr service, with esp_intr_flag_defult = 0
    gpio_install_isr_service(0);
    // finally, hook isr to individual gpios
    gpio_isr_handler_add(GPIO_PWM_INPUT_CH0, gpio_isr_handler_tickTimeBetweenEdges, (void *)HDRIVE_CH0);
    gpio_isr_handler_add(GPIO_PWM_INPUT_CH1, gpio_isr_handler_tickTimeBetweenEdges, (void *)HDRIVE_CH1);
    // initialize tick vectors
    for (ch = 0; ch < PWM_NUM_INPUT_CHANNELS; ch++)
    {
        isrTicksAtRising[ch] = 0;
        isrTicksAtFalling[ch] = 0;
        isrTickDifference[ch] = 0;
    }
}
// ========================================================================================
// function to calculate duty cycle output (setpoint hdrive) out of SET angle in degrees
int hdrive_degree_2_duty(float degree)
{
    float a = 0.018, b = -2.65; // these values leed to: 150°-200° = 0.05-0.95 dc
    float duty = (degree * a + b);
    // make sure duty cycle is inbetween 5% and 95%
    if (duty < 0.05)
    {
        duty = 0.05;
    }
    else if (duty > 0.95)
    {
        duty = 0.95;
    }
    float dutyRes = PWM_OUTPUT_RESOLUTION;
    return (int)(duty * (pow(2, dutyRes) - 1)); // calc duty in x-bit resolution
}
// ========================================================================================
// function to calculate angle out of a scaled percentage input (from wifi) from 0-100%
float percent_2_angle(float percent)
{
    float a = (maximumForceAngle - minimumForceAngle) / 100.0, b = minimumForceAngle;
    float angle = (percent * a + b);
    // make sure angle is inbetween 150° and 170°
    if (angle < maximumForceAngle)
    {
        angle = maximumForceAngle;
    }
    else if (angle > minimumForceAngle)
    {
        angle = minimumForceAngle;
    }
    return angle;
}
// ========================================================================================
// function to calculate scaled percent out of angle input (for wifi) from 150-170%
float angle_2_percent(float angle)
{
    float a = 100.0 / (maximumForceAngle - minimumForceAngle), b = -100.0 * minimumForceAngle / (maximumForceAngle - minimumForceAngle);
    float percent = (angle * a + b);
    // make sure percent is valid
    if (percent < 0)
    {
        percent = 0;
    }
    else if (percent > 100)
    {
        percent = 100;
    }
    return percent;
}
// ========================================================================================
// function to SET position (angle in degrees) of individual hdrive channels
void hdrive_set_position(int hdrive_channel, float hdrive_setAngle)
{
    uint32_t duty = (uint32_t)hdrive_degree_2_duty(hdrive_setAngle);
    ledc_set_duty(ledc_channel[hdrive_channel].speed_mode, ledc_channel[hdrive_channel].channel, duty);
    ledc_update_duty(ledc_channel[hdrive_channel].speed_mode, ledc_channel[hdrive_channel].channel);
}
// ========================================================================================
// function to GET position (angle in degrees) of individual hdrive channels
// returns hdrive position in [degrees]
float hdrive_get_position(int hdrive_channel)
{
    float timeDiff = ((float)isrTickDifference[hdrive_channel]) / ((float)TIMER_TICK_FREQ);
    float pwmFreq = (float)PWM_INPUT_FREQ;
    float duty = timeDiff * pwmFreq; // dc in [%/100]
    // float a = -360/0.975, b = 360;   // values from MT (Juli 2020) --> we had different motor angles in MT! (see Doku)
    float a = 368.5, b = -7.0;            // new values, hdrive June 2021 --> hdrive angle is tha same as controller angle
    float angle = 360.0 - (a * duty + b); // hdrive switched direction...
    if (angle > 360)
    {
        angle = 360;
    }
    else if (angle < 0)
    {
        angle = 0;
    }
    return angle;
    // return duty;
}
// ========================================================================================
// interrupt service routine to measure ticks between edges
// This version does not measure time between rising edges!
static void IRAM_ATTR gpio_isr_handler_tickTimeBetweenEdges(void *hdrive_ch)
{
    // check edge of gpio, because interrupt is thrown on both edges
    int ch = (int)hdrive_ch;
    if (gpio_get_level(gpio_ch_select[ch]))
    {
        timer_get_counter_value(timer_group, timer_idx, &isrTicksAtRising[ch]);
    }
    else
    {
        timer_get_counter_value(timer_group, timer_idx, &isrTicksAtFalling[ch]);
        isrTickDifference[ch] = isrTicksAtFalling[ch] - isrTicksAtRising[ch];
    }
}
// ========================================================================================
// function to get hdrive channel select
int hdrive_get_channel_select()
{
    // gpio is 0 for channel 1, 1 for channel 2
    // so channel[0] means level 0, channel[1] means level 1
    int ch = gpio_get_level(GPIO_CHANNEL_SEL_INPUT);
    if (ch < 0)
    {
        ch = 0;
    }
    else if (ch > 1)
    {
        ch = 1;
    }
    return ch;
}
// ========================================================================================
// function to get hdrive currentSetpointClosedPercent. ch = channel select
float hdrive_get_currentSetpointClosedPercent(int ch)
{
    return hdrive[ch].currentSetpointClosedPercent;
}
// ========================================================================================
// =================== HDRIVE TASK ========================================================
// ========================================================================================
void hdrive_task(void *arg)
{
    // =====================================================
    // local variables
    // variable to store last tick time for exakt cycle time. Updates every cycle
    TickType_t previousWakeTime = xTaskGetTickCount();
    TickType_t cycleFrequency = 1000;                 // actual cycle frequency [Hz]
    float cycleTime = 1000.0 / (float)cycleFrequency; // actual cycle time [ms] off hdrive loop
    int cycleCount = 0;                               // count hdrive swicth cycles
    uint64_t actual_counter_value = 0;                // counter value for testing/debugging
    uint64_t before_counter_value = 0;                // counter value for testing/debugging
    uint64_t after_counter_value = 0;                 // counter value for testing/debugging
    int difference_hdrive = 0;
    uint8_t testing0 = 0;
    uint8_t testing1 = 0;
    int step_counter[HDRIVE_NUM_OF_CHANNELS] = {0, 0};
    //
    uint8_t delayCounter[2] = {0, 0};
    //
    int testCounter = 0;
    //
    // uint8_t delayStepWhileClosedCounter[2] = {0, 0};
    //
    int hdriveRunningTimeoutCounter[HDRIVE_NUM_OF_CHANNELS] = {0, 0}; // timeout for detecting idle channel
    bool hdriveTriggerOld[HDRIVE_NUM_OF_CHANNELS] = {false, false};   // old (timeshift) trigger for hdrives (true=open, false=close)
    bool hdriveTrigger[HDRIVE_NUM_OF_CHANNELS] = {false, false};      // trigger for hdrives (true=open, false=close)
    bool hdriveChannel = false;                                       // channel select (true = hdrive[1], false = hdrive [0])
    bool hdriveChannelInv = true;                                     // inversed channel select
    bool hdriveChannelSel[2] = {true, false};
    // ====================================================
    // Initialize stuff
    init_gpio_hdrive();
    //...
    // init global hdrive objects (and remanent nvs data)
    hdrive[0].setpointClosed = percent_2_angle(wifi_fb[0].setpoint_hdrive);
    hdrive[0].setpointOpen = absoluteOpenAngle;
    hdrive[0].step = HD_DISABLED;
    hdrive[0].actualPosition = hdrive_get_position(0);
    // hdrive[0].trigger_gpio_ch= GPIO_TRIGGER1_INPUT;
    hdrive[0].currentSetpointClosedPercent = 0;
    hdrive[0].hdrive_closing_time = wifi_fb[0].hdrive_closing_time;
    hdrive[0].hdrive_opening_time = wifi_fb[0].hdrive_opening_time;
    //
    hdrive[1].setpointClosed = percent_2_angle(wifi_fb[1].setpoint_hdrive);
    hdrive[1].setpointOpen = absoluteOpenAngle;
    hdrive[1].step = HD_DISABLED;
    hdrive[1].actualPosition = hdrive_get_position(1);
    // hdrive[1].trigger_gpio_ch= GPIO_TRIGGER2_INPUT;
    hdrive[1].currentSetpointClosedPercent = 0;
    hdrive[1].hdrive_closing_time = wifi_fb[0].hdrive_closing_time;
    hdrive[1].hdrive_opening_time = wifi_fb[0].hdrive_opening_time;
    // Disable hdrives
    gpio_set_level(GPIO_ENABLE_OUTPUT, 0);
    // initialize control mode to manual, check last control mode
    if (current_control_mode[0])
    {
        xEventGroupSetBits(hdrive_event_group_array[0], BIT_CONTROL_MODE_MEAN);
    }
    else
    {
        xEventGroupSetBits(hdrive_event_group_array[0], BIT_CONTROL_MODE_MANUAL);
    }
    if (current_control_mode[1])
    {
        xEventGroupSetBits(hdrive_event_group_array[1], BIT_CONTROL_MODE_MEAN);
    }
    else
    {
        xEventGroupSetBits(hdrive_event_group_array[1], BIT_CONTROL_MODE_MANUAL);
    }
    // =============================================================================================================================
    // WHILE LOOP
    // =============================================================================================================================
    while (1)
    {
        // =========================================================================================================================
        // === Get Position and Triggers
        // =========================================================================================================================
        // Get hdrive positions
        hdrive[0].actualPosition = hdrive_get_position(0);
        hdrive[1].actualPosition = hdrive_get_position(1);
        //
        // catch change of opening/closing times
        if (check_clear_event_bit(wifi_event_group, BIT_SET_OPENING_CLOSING_TIME))
        {
            hdrive[0].hdrive_closing_time = wifi_fb[0].hdrive_closing_time;
            hdrive[0].hdrive_opening_time = wifi_fb[0].hdrive_opening_time;
            hdrive[1].hdrive_closing_time = wifi_fb[0].hdrive_closing_time;
            hdrive[1].hdrive_opening_time = wifi_fb[0].hdrive_opening_time;
        }
        // get channel select (digIn) --> change channel if needed
        // --> logic 1 means channel 2 is active
        // hdriveChannelSel[x] is true if corresponding channel has to do something
        hdriveChannelSel[0] = !gpio_get_level(GPIO_CHANNEL_SEL_INPUT); // here we could also use !hdrive_get_channel_select()
        hdriveChannelSel[1] = !hdriveChannelSel[0];
        //
        // =========================================================================================================================
        // === ENABLE / DISABLE
        // =========================================================================================================================
        // watch for disable command from browser/wifi.
        if (check_clear_event_bit(wifi_event_group, BIT_REQ_DISABLE_HDRIVE))
        {
            // move state machines to requested disable
            hdrive[0].step = HD_REQ_DISABLED;
            hdrive[1].step = HD_REQ_DISABLED;
            // disable hdrives
            ESP_LOGW(TAG, "Manual disable of hdrives");
            gpio_set_level(GPIO_ENABLE_OUTPUT, 0);
        }
        // wait for request of re-enabling hdrives. Only if hdrives are in fact disabled!
        else if (check_clear_event_bit(wifi_event_group, BIT_REQ_ENABLE_HDRIVE))
        {
            // check if hdrives are in disabled state, and if so, let them try to enable again
            if ((hdrive[0].step == HD_REQ_DISABLED) && (hdrive[1].step == HD_REQ_DISABLED))
            {
                ESP_LOGW(TAG, "Manual enable of hdrives");
                hdrive[0].step = HD_ENABLE;
                hdrive[1].step = HD_ENABLE;
            }
            else
            {
                ESP_LOGE(TAG, "hdrives are not both in requested disabled state. doing nothing...");
            }
        }
        // wait for both hdrives to be ready to enable. If so, allow them to be enabled in state machine and enable via gpio
        else if (check_event_bit(hdrive_event_group_array[0], BIT_HDRIVE_READY_TO_ENABLE) && check_event_bit(hdrive_event_group_array[1], BIT_HDRIVE_READY_TO_ENABLE))
        {
            ESP_LOGW(TAG, "Master saw two ready drives. setting bits to enable");
            xEventGroupSetBits(hdrive_event_group_array[0], BIT_HDRIVE_ALLOWED_TO_ENABLE);
            xEventGroupSetBits(hdrive_event_group_array[1], BIT_HDRIVE_ALLOWED_TO_ENABLE);
            gpio_set_level(GPIO_ENABLE_OUTPUT, 1); //
        }
        // =========================================================================================================================
        // === CONTROL MODE, SETPOINT and start of For-Loop ========================================================================
        // =========================================================================================================================
        for (int hdrive_ch = 0; hdrive_ch < HDRIVE_NUM_OF_CHANNELS; hdrive_ch++)
        {
            // check wich control Mode is active (if any)
            // if manual mode:
            if (check_event_bit(hdrive_event_group_array[hdrive_ch], BIT_CONTROL_MODE_MANUAL))
            {
                // if manual mode and new setpoint...
                if (check_clear_event_bit(hdrive_event_group_array[hdrive_ch], BIT_CHANGE_SETPOINT_HDRIVE))
                {
                    ESP_LOGW(TAG, "[%i] New manual SetpointClosed position received", hdrive_ch);
                    // set closed setpoint to wifi setpoint
                    hdrive[hdrive_ch].setpointClosed = percent_2_angle(wifi_fb[hdrive_ch].setpoint_hdrive);
                }
            }
            // if mean mode:
            if (check_event_bit(hdrive_event_group_array[hdrive_ch], BIT_CONTROL_MODE_MEAN))
            {
                if (check_clear_event_bit(hdrive_event_group_array[hdrive_ch], BIT_HDRIVE_CHANGE_AUTO_SETPOINT))
                {
                    // ESP_LOGW(TAG, "[%i] New auto/mean SetpointClosed position received", hdrive_ch);
                    // if mean mode, setpoint = control output:
                    hdrive[hdrive_ch].setpointClosed = mean_control[hdrive_ch].mean_control_output;
                }
            }
            // after setpoint closed chosen, check absolute boundries
            if (hdrive[hdrive_ch].setpointClosed < 150)
            {
                hdrive[hdrive_ch].setpointClosed = 150;
            }
            else if (hdrive[hdrive_ch].setpointClosed > 200)
            {
                hdrive[hdrive_ch].setpointClosed = 200;
            }
            // and copy current setpoint closed for wifi
            hdrive[hdrive_ch].currentSetpointClosedPercent = angle_2_percent(hdrive[hdrive_ch].setpointClosed);
            //
            // ==========================================================================================================================
            // === STATE MACHINE ========================================================================================================
            // =========================================================================================================================
            switch (hdrive[hdrive_ch].step)
            {
            // ==========================================================================================
            // DISABLED
            // ==========================================================================================
            case HD_DISABLED:
                step_counter[hdrive_ch]++;
                if (step_counter[hdrive_ch] >= 4000)
                {
                    printf("[%i] Trying to enable hdrive. Checking if position is in-range...\n", hdrive_ch);
                    step_counter[hdrive_ch] = 0;
                    hdrive[hdrive_ch].step = HD_ENABLE;
                }
                else if (step_counter[hdrive_ch] == 3000)
                {
                    printf("[%i] Try to enable hdrive in 1...\n", hdrive_ch);
                }
                else if (step_counter[hdrive_ch] == 2000)
                {
                    printf("[%i] Try to enable hdrive in 2...\n", hdrive_ch);
                }
                else if (step_counter[hdrive_ch] == 1000)
                {
                    printf("[%i] Try to enable hdrive in 3...\n", hdrive_ch);
                }
                //
                hdrive_set_position(hdrive_ch, hdrive[hdrive_ch].setpointOpen); // if not enabled, just set position to be open
                //
                break;
            // ==========================================================================================
            // ENABLE
            // ==========================================================================================
            case HD_ENABLE:
                // check if actual position is in range
                // if((hdrive[hdrive_ch].actualPosition>=145) && (hdrive[hdrive_ch].actualPosition<=290)){
                if ((hdrive[hdrive_ch].actualPosition >= 0) && (hdrive[hdrive_ch].actualPosition <= 360)) // set a position in any case
                {
                    // hdrive_set_position(hdrive_ch, hdrive[hdrive_ch].actualPosition);
                    //  if actual position cannot be read, just set setpoint open as first position
                    hdrive_set_position(hdrive_ch, hdrive[hdrive_ch].setpointOpen);
                    // hdrive_set_position(hdrive_ch, minimumForceAngle); // set minimal closed angle
                    // signal readyness
                    xEventGroupSetBits(hdrive_event_group_array[hdrive_ch], BIT_HDRIVE_READY_TO_ENABLE);
                    // wait for master to allow enabling
                    if (check_event_bit(hdrive_event_group_array[hdrive_ch], BIT_HDRIVE_ALLOWED_TO_ENABLE))
                    {
                        // clear relevant eventbits
                        xEventGroupClearBits(hdrive_event_group_array[hdrive_ch], BIT_HDRIVE_READY_TO_ENABLE | BIT_HDRIVE_ALLOWED_TO_ENABLE);
                        hdrive[hdrive_ch].step = HD_INITIALIZE;
                    }
                }
                else
                {
                    // if not in range, clear relevant bits, go back to disabled and try again
                    ESP_LOGE(TAG, "[%i] hdrive position NOT in-range: %.2f. retrying...", hdrive_ch, hdrive[hdrive_ch].actualPosition);
                    xEventGroupClearBits(hdrive_event_group_array[hdrive_ch], BIT_HDRIVE_READY_TO_ENABLE | BIT_HDRIVE_ALLOWED_TO_ENABLE);
                    hdrive[hdrive_ch].step = HD_DISABLED;
                }
                break;
            // ==========================================================================================
            // INITIALIZE
            // ==========================================================================================
            case HD_INITIALIZE:
                // wait in this state
                step_counter[hdrive_ch]++;
                if (step_counter[hdrive_ch] <= 1)
                {
                    printf("[%i] hdrive enabled, waiting for 1 second...\n", hdrive_ch);
                }
                else if (step_counter[hdrive_ch] >= 500)
                {
                    printf("[%i] trying to open brake...\n", hdrive_ch);
                    // hdrive_set_position(hdrive_ch, hdrive[hdrive_ch].setpointClosed);
                    hdrive_set_position(hdrive_ch, hdrive[hdrive_ch].setpointOpen);
                    // hdrive_set_position(hdrive_ch, minimumForceAngle); // 05.08.21 at startup, close to minimal force angle
                    step_counter[hdrive_ch] = 0;
                    xEventGroupClearBits(hdrive_event_group_array[hdrive_ch], BIT_HDRIVE_RUNNING);
                    hdriveRunningTimeoutCounter[hdrive_ch] = 0;
                    //
                    // hdrive[hdrive_ch].step = HD_CLOSING;
                    hdrive[hdrive_ch].step = HD_OPENING;
                }
                break;
            // ==========================================================================================
            // CLOSING
            // ==========================================================================================
            case HD_CLOSING: // --> if we do not measure actual position, this state is obsolete...
                // check if brake has closed:
                if (abs(hdrive[hdrive_ch].setpointClosed - hdrive[hdrive_ch].actualPosition) <= 360) // if actual position is not measured, we use 360°
                {
                    // printf("[%i] hdrive has successfully closed...\n",hdrive_ch);
                    delayCounter[hdrive_ch] = 0;
                    hdrive[hdrive_ch].step = HD_CLOSED;
                }
                else
                {
                    // hdrive_set_position(hdrive_ch, hdrive[hdrive_ch].setpointClosed);
                }

                break;
            // ==========================================================================================
            // CLOSED
            // ==========================================================================================
            case HD_CLOSED:
                // wait for trigger to open. If loom not running (only in Auto mode!), open immediately
                if ((!loom.machine_running && (current_control_mode[hdrive_ch] == CONTROL_MODE_MEAN)) ||
                    (hdriveChannelSel[hdrive_ch] && (loom.actualAngle >= hdrive[hdrive_ch].hdrive_opening_time) && (loom.actualAngle < (hdrive[hdrive_ch].hdrive_closing_time - hdriveAngleHysteresis))))
                {

                    if (delayCounter[hdrive_ch] >= (delayUntilOpen / cycleTime))
                    {
                        delayCounter[hdrive_ch] = 0;
                        hdrive_set_position(hdrive_ch, hdrive[hdrive_ch].setpointOpen);
                        // signal non-idleness
                        hdriveRunningTimeoutCounter[hdrive_ch] = 0;
                        xEventGroupSetBits(hdrive_event_group_array[hdrive_ch], BIT_HDRIVE_RUNNING);
                        hdrive[hdrive_ch].step = HD_OPENING;
                    }
                    else
                    {
                        delayCounter[hdrive_ch]++;
                    }
                }
                else
                {
                    // reset delay counter
                    delayCounter[hdrive_ch] = 0;
                    // detect idleness
                    if (hdriveRunningTimeoutCounter[hdrive_ch] < (int)(2000.0 / cycleTime))
                    {
                        hdriveRunningTimeoutCounter[hdrive_ch]++;
                    }
                    else
                    {
                        xEventGroupClearBits(hdrive_event_group_array[hdrive_ch], BIT_HDRIVE_RUNNING);
                    }
                    // detect if we are in manual mode, if so then change setpoint closed constantly
                    if (current_control_mode[hdrive_ch] == CONTROL_MODE_MANUAL)
                    {
                        hdrive_set_position(hdrive_ch, hdrive[hdrive_ch].setpointClosed);
                    }
                }

                break;
            // ==========================================================================================
            // OPENING
            // ==========================================================================================
            case HD_OPENING: // --> if we do not measure actual position, this state is obsolete...
                // check if brake has opened:
                if (abs(hdrive[hdrive_ch].setpointOpen - hdrive[hdrive_ch].actualPosition) <= 360)
                {
                    // printf("[%i] hdrive has successfully opened...\n",hdrive_ch);
                    delayCounter[hdrive_ch] = 0;
                    hdrive[hdrive_ch].step = HD_OPEN;
                }
                else
                {
                    // hdrive_set_position(hdrive_ch, hdrive[hdrive_ch].setpointOpen);
                }
                break;
            // ==========================================================================================
            // OPEN
            // ==========================================================================================
            case HD_OPEN:
                // wait for trigger to close. In Manual Mode, ignore machine running status!
                if (
                    (loom.machine_running || (current_control_mode[hdrive_ch] == CONTROL_MODE_MANUAL)) &&
                    hdriveChannelSel[hdrive_ch] &&
                    ((loom.actualAngle >= hdrive[hdrive_ch].hdrive_closing_time) || (loom.actualAngle < (hdrive[hdrive_ch].hdrive_opening_time - hdriveAngleHysteresis)))

                )
                {
                    if (delayCounter[hdrive_ch] >= (delayUntilClose / cycleTime))
                    {
                        delayCounter[hdrive_ch] = 0;
                        hdrive_set_position(hdrive_ch, hdrive[hdrive_ch].setpointClosed);
                        // signal non-idleness
                        hdriveRunningTimeoutCounter[hdrive_ch] = 0;
                        xEventGroupSetBits(hdrive_event_group_array[hdrive_ch], BIT_HDRIVE_RUNNING);
                        hdrive[hdrive_ch].step = HD_CLOSING;
                    }
                    else
                    {
                        delayCounter[hdrive_ch]++;
                    }
                }
                else
                {
                    // reset delay counter
                    delayCounter[hdrive_ch] = 0;
                    // detect idleness
                    if (hdriveRunningTimeoutCounter[hdrive_ch] < (int)(2000.0 / cycleTime))
                    {
                        hdriveRunningTimeoutCounter[hdrive_ch]++;
                    }
                    else
                    {
                        xEventGroupClearBits(hdrive_event_group_array[hdrive_ch], BIT_HDRIVE_RUNNING);
                    }
                    // TESTING 21.10.2021
                    // hdrive_set_position(hdrive_ch, hdrive[hdrive_ch].setpointClosed); // set setpoint closed so that we can change position during IB
                }
                // else, if certain timeout --> kriechgang z.B.

                break;
            // ==========================================================================================
            // REQ_DISABLED
            // ==========================================================================================
            case HD_REQ_DISABLED:
                // wait for re-enabling
                // this state is only left if Master re-enables drives through forcing step of state machine
            // ==========================================================================================
            // DEFAULT
            // ==========================================================================================
            default:
                //...
                break;
            } // end switch
        }     // end for

        // ==========================================================================================
        // print stuff every x cycles, i.e. every 2*x ms
        if (cycleCount >= 1000)
        {
            // printf("free running timer value [s]: %.5f\n",ftestvar1);
            // printf("= AcPos: %.2f = Step: %i = eventGroupBits: %i, logic: %i\n",hdrive[0].actualPosition,hdrive[0].step,xEventGroupGetBits(hdrive_event_group_array[0]),(xEventGroupGetBits(hdrive_event_group_array[0]) & BIT_REQ_ENABLE_HDRIVE));
            // printf("= AcPos0: %.2f = AcPos1: %.2f =\n",hdrive[0].actualPosition,hdrive[1].actualPosition);
            // printf("= 0: %i = 1: %i =\n",gpio_get_level(GPIO_TRIGGER1_INPUT),gpio_get_level(GPIO_TRIGGER2_INPUT));
            //  printf("= step 0: %i = step 1: %i =\n", hdrive[0].step, hdrive[1].step);
            // printf("[0]: setpoint %.1f, feedback: %.2f\n",hdrive[0].setpointClosed,hdrive[0].currentSetpointClosedPercent);
            // printf("[1]: step: %i, setpointClosed %.1f, feedback: %.2f\n",hdrive[1].step, hdrive[1].setpointClosed,hdrive[1].actualPosition);

            // printf("[hd1] -%i-\n",difference_hdrive);
            // printf("trig1: %i\n",gpio_get_level(GPIO_TRIGGER1_INPUT));

            // testing0 = (xEventGroupGetBits(hdrive_event_group_array[0]) & BIT_HDRIVE_RUNNING);
            // testing1 = (xEventGroupGetBits(hdrive_event_group_array[1]) & BIT_HDRIVE_RUNNING);

            // IB printfs ===============
            // printf("= AcPos0: %.2f = AcPos1: %.2f =\n",hdrive[0].actualPosition,hdrive[1].actualPosition);
            // printf("= 0: %i = 1: %i =\n",gpio_get_level(GPIO_TRIGGER1_INPUT),gpio_get_level(GPIO_TRIGGER2_INPUT));
            // ==========================

            // printf("TRIGGER = 0: %i = 1: %i =\n", hdriveTrigger[0], hdriveTrigger[1]);
            // printf("[0]: %i === [1]: === %i === %i\n",testing0,testing1,hdriveRunningTimeoutCounter[0]);

            // if(testCounter>=5000){
            //     testCounter = 0;
            //     hdrive[0].setpointClosed = 150;
            //     printf("DISABLE/ENABLE\n");
            // }

            // if(testCounter>3){

            //     hdrive[0].setpointClosed = percent_2_angle(wifi_fb[0].setpoint_hdrive);
            // }
            // testCounter++;

            // DIAGNOSE CHECKS
            // if gpio = 0, then disabled, if 1, then enabled
            if (gpio_get_level(GPIO_ENABLE_OUTPUT))
            {
                xEventGroupSetBits(diagnose_event_group, BIT_DIAGNOSE_1_HDRIVE_ENABLED);
            }
            else
            {
                xEventGroupClearBits(diagnose_event_group, BIT_DIAGNOSE_1_HDRIVE_ENABLED);
            }

            cycleCount = 0;
        }
        // ====================================================
        // delay until next cycle, increment cycle counter
        cycleCount++;
        vTaskDelayUntil(&previousWakeTime, (configTICK_RATE_HZ / cycleFrequency));
        // vTaskDelay(1);
        //  ====================================================
    }
}
