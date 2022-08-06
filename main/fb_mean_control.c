/*
    ToDo: use global filter functions and -objects
    // filter variables
    filter_pt1_zoh_t filter_object_actual_angle;
    filter_object_actual_angle.input = 0;
    filter_object_actual_angle.input_old = 0;
    filter_object_actual_angle.output = 0;
    filter_object_actual_angle.coeff_B = exp(-cycleTime / 500.0);
    filter_object_actual_angle.coeff_A = 1 - filter_object_actual_angle.coeff_B;

    - take out, on top of state machine, "switch controle mode" --> check and clear event bits in state machine,
    not at two point in the code!!!
    - use our functions check_clear_bits
    - for hdrive setup: only change setpointclosed in mean mode if new control value has benn written: hdrive_event_group_array(bit_new_auto_setpoint)



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
#include "fb_mean_control.h"
#include "fb_color_sensor.h"

//
static const char *TAG = "fb_mean_control";
//
// === PARAMETERS ======================================================
float mean_control_KP = -16;        // KP for mean control [°/cN]
float mean_control_Tn = 190;        // Tn for mean control [Schuss]
float filter_slow_T1 = 1000;        // T1 for mean value filter [Schuss]
float filter_fast_T1 = 20;          // T1 for mean value filter [Schuss]
float filter_T = 1;                 // sample time of mean value filter [Schuss]
float mean_control_satHigh = 170.0; // high saturation of mean control
float mean_control_satLow = 150.0;  // low saturation of mean control
int saturation_threshhold = 30;     // amount of cycles controller may be in saturation until change of filter dynamics
float saturation_hysteresis = 2;    // Hysteresis for coming out of high/low saturation of controller

// === END PARAMETERS ==================================================
//
typedef enum
{
    MC_IDLE,
    MC_MANUAL,
    MC_MEAN
} mean_control_enum_t;
mean_control_enum_t step_mean_control[2]; // one step for time beeing; both drives always have same mode
//
// =========================================================
// function to filter display value to send via wifi
// ZOH, only for display, not in control loop
// works with address of control object and filter time constant, no return values
void mean_control_filter_displayData(mean_control_t *pMean_control, float coeff_A, float coeff_B)
{
    // filter value
    (*pMean_control).display_mean = (*pMean_control).actual_single_mean_value * coeff_A + (*pMean_control).display_mean_old * coeff_B;
    // old = actual
    (*pMean_control).display_mean_old = (*pMean_control).display_mean;
}
// =========================================================
// function to filter mean for control
// ZOH
// works with address of control object and filter time constant, no return values
void mean_control_filter_mean(mean_control_t *pMean_control, float coeff_A, float coeff_B)
{
    // filter value
    (*pMean_control).actual_mean_value = (*pMean_control).actual_single_mean_value * coeff_A + (*pMean_control).actual_mean_value_old * coeff_B;
    // old = actual
    (*pMean_control).actual_mean_value_old = (*pMean_control).actual_mean_value;
}
// =========================================================
// function to filter mean for control
// tustin
// works with address of control object and filter time constant, no return values
// achtung; koeffizienten a und b sind eigentlich nicht die "normalen" koeffs. -> vgl. Doku von Masterarbeit!
void mean_control_filter_mean_2(mean_control_t *pMean_control)
{
    //
    // current mean value
    (*pMean_control).actual_mean_x[1] = (*pMean_control).actual_single_mean_value;
    // filter algo
    (*pMean_control).actual_mean_y[1] = (*pMean_control).actual_mean_y[0] * (*pMean_control).actual_mean_a[0] +
                                        (*pMean_control).actual_mean_x[1] * (*pMean_control).actual_mean_b[1] + (*pMean_control).actual_mean_x[0] * (*pMean_control).actual_mean_b[0];
    // timeshift
    (*pMean_control).actual_mean_y[0] = (*pMean_control).actual_mean_y[1];
    (*pMean_control).actual_mean_x[0] = (*pMean_control).actual_mean_x[1];
    // output
    (*pMean_control).actual_mean_value = (*pMean_control).actual_mean_y[1] / (*pMean_control).actual_mean_a[1];
}
// =========================================================
// function to filter Setpoint value for control
// tustin
// works with address of control object, no return values. This function works with a,b coefficiants (arrays)
void mean_control_filter_setpoint_2(mean_control_t *pMean_control)
{
    //
    // y_k+2 * a2 = y_k+1 * a1 + y_k * a0 + x_k+1 * b1 + x_k * b0
    // entering filter, current setpoint:
    (*pMean_control).x[2] = (double)((*pMean_control).mean_setpoint);
    // filter algo:
    (*pMean_control).y[2] = (*pMean_control).y[1] * (*pMean_control).a[1] + (*pMean_control).y[0] * (*pMean_control).a[0] +
                            (*pMean_control).x[2] * (*pMean_control).b[2] + (*pMean_control).x[1] * (*pMean_control).b[1] + (*pMean_control).x[0] * (*pMean_control).b[0];
    // timeshift
    (*pMean_control).y[0] = (*pMean_control).y[1];
    (*pMean_control).y[1] = (*pMean_control).y[2];
    (*pMean_control).x[0] = (*pMean_control).x[1];
    (*pMean_control).x[1] = (*pMean_control).x[2];
    // output:
    (*pMean_control).mean_setpoint_filtered = (float)((*pMean_control).y[2] / (*pMean_control).a[2]);
}
// =========================================================
// function to control mean value
void mean_control_controller(mean_control_t *pMean_control, float Kp, float Tn, float saturationHigh, float saturationLow)
{
    // calculate control error
    float control_error = (*pMean_control).mean_setpoint_filtered - (*pMean_control).actual_mean_value;
    // proportional part Kp*e
    float p_part = Kp * control_error;
    // integral part I(old) + h*Kp/Tn*e (here h is 1, i.e. every shot)
    float i_part = (*pMean_control).control_integral + (Kp / Tn * control_error);
    // temporary control value before limits:
    float control_value_temp = p_part + i_part;
    // check limits and clamp if necessary --> if out of bounds, don't update Integral
    if (control_value_temp > saturationHigh)
    {
        (*pMean_control).mean_control_output = saturationHigh;
    }
    else if (control_value_temp < saturationLow)
    {
        (*pMean_control).mean_control_output = saturationLow;
    }
    else
    {
        // if in limits, use temp value and do timeshift of integral
        (*pMean_control).mean_control_output = control_value_temp;
        (*pMean_control).control_integral = i_part;
    }
}

// =========================================================
// mean control task
// =========================================================
void mean_control_task(void *arg)
{
    // =====================================================
    // local variables
    // variable to store last tick time for exakt cycle time. Updates every cycle
    TickType_t previousWakeTime = xTaskGetTickCount();
    TickType_t pauseTime = 200;
    TickType_t cycleFrequency = 5;     // actual cycle frequency [Hz]
    uint64_t actual_counter_value = 0; // counter value for testing/debugging
    uint64_t before_counter_value = 0; // counter value for testing/debugging
    uint64_t after_counter_value = 0;  // counter value for testing/debugging
    int difference = 0;
    int cycleCount = 0; // count switch cycles
    //
    uint8_t ifilter_cycle_counter[2] = {0, 0};
    //
    bool switchBit[2] = {false, false};    // bit to signal switching filter coefficiants during control if controller is in saturation
    bool switchBitOld[2] = {false, false}; // timeshifted switchbit
    int switchBitCounter[2] = {0, 0};
    // =====================================================
    // init filter coefficiants (ZOH) for display value only (not for control)
    float coeffB_fast = exp(-filter_T / filter_fast_T1);
    float coeffA_fast = 1 - coeffB_fast;
    //
    // coefficiants for mean filtering
    float mean_filter_fast_a[2];
    mean_filter_fast_a[0] = (2 * filter_fast_T1 - filter_T) / (2 * filter_fast_T1 + filter_T);
    mean_filter_fast_a[1] = 1;
    float mean_filter_fast_b[2];
    mean_filter_fast_b[0] = (filter_T) / (2 * filter_fast_T1 + filter_T);
    mean_filter_fast_b[1] = mean_filter_fast_b[0];
    //
    float mean_filter_slow_a[2];
    mean_filter_slow_a[0] = (2 * filter_slow_T1 - filter_T) / (2 * filter_slow_T1 + filter_T);
    mean_filter_slow_a[1] = 1;
    float mean_filter_slow_b[2];
    mean_filter_slow_b[0] = (filter_T) / (2 * filter_slow_T1 + filter_T);
    mean_filter_slow_b[1] = mean_filter_slow_b[0];
    //
    // init coeffs for setpoint pt2 filtering:
    double setpoint_filter_a[3];
    double setpoint_filter_b[3];
    double T1 = (double)mean_control_Tn;
    double T2 = (double)filter_slow_T1;
    double Ts = (double)filter_T;
    //
    setpoint_filter_a[0] = -((2 * T1 - Ts) * (2 * T2 - Ts)) / ((2 * T1 + Ts) * (2 * T2 + Ts));
    setpoint_filter_a[1] = ((2 * T1 - Ts) / (2 * T1 + Ts)) + ((2 * T2 - Ts) / (2 * T2 + Ts));
    setpoint_filter_a[2] = 1;
    //
    setpoint_filter_b[0] = (Ts / (2 * T1 + Ts)) * (Ts / (2 * T2 + Ts));
    setpoint_filter_b[1] = 2 * setpoint_filter_b[0];
    setpoint_filter_b[2] = setpoint_filter_b[0];
    //
    // copy into all control objects
    for (int kk = 0; kk < 2; kk++)
    {
        for (int ll = 0; ll < 3; ll++)
        {
            mean_control[kk].x[ll] = 0;
            mean_control[kk].y[ll] = 0;
            mean_control[kk].a[ll] = setpoint_filter_a[ll];
            mean_control[kk].b[ll] = setpoint_filter_b[ll];
        }
    }
    //
    step_mean_control[0] = MC_IDLE;
    step_mean_control[1] = MC_IDLE;
    //
    while (1)
    {

        // =========================================================================================================================
        // === SWITCH CONTROL MODE =================================================================================================
        // if from wifi request to change to mode 1...

        /*
        try following:

        if(wifieventgroup = change_to_1/0){
            set hdrive_eventgroup[0/1] bit to mode_0/1
            clear wifi_event_bit
            DONT clear hdrive event bit
        }

        STATE MACHINE

        Idle
        NO changes in hdrive_event_bits!

        MANUAL
        if(hdrive_event_bit = MEAN){
            switch to state MEAN, leave all event bits as they are
        }

        MEAN



        */

        // This is a observer that
        if ((xEventGroupGetBits(wifi_event_group) & BIT_REQ_CHANGE_TO_MODE_1) == BIT_REQ_CHANGE_TO_MODE_1)
        {
            // clear request for mode 0...
            xEventGroupClearBits(wifi_event_group, BIT_REQ_CHANGE_TO_MODE_0);
            // ... wait for both drives to have changed to mode 1...
            if ((current_control_mode[0] == 1) && (current_control_mode[1] == 1))
            {
                // ... and clear request for mode 1
                xEventGroupClearBits(wifi_event_group, BIT_REQ_CHANGE_TO_MODE_1);
            }
            // same for change of mode to 0
        }
        else if ((xEventGroupGetBits(wifi_event_group) & BIT_REQ_CHANGE_TO_MODE_0) == BIT_REQ_CHANGE_TO_MODE_0)
        {
            xEventGroupClearBits(wifi_event_group, BIT_REQ_CHANGE_TO_MODE_1);
            if ((current_control_mode[0] == 0) && (current_control_mode[1] == 0))
            {
                xEventGroupClearBits(wifi_event_group, BIT_REQ_CHANGE_TO_MODE_0);
            }
        }
        // =========================================================================================================================
        // === CONTROL LOOPS =======================================================================================================
        // === START OF FOR LOOP ============
        // for each hdrive do mean control
        for (int hdrive_ch = 0; hdrive_ch < 2; hdrive_ch++)
        {

            switch (step_mean_control[hdrive_ch])
            {
            // ==========================================================================================
            // IDLE
            // ==========================================================================================
            case MC_IDLE:
                // in Idle-Mode, wait for hallS-trigger to filter display data
                //
                // filter display data:
                if ((xEventGroupGetBits(hdrive_event_group_array[hdrive_ch]) & BIT_VALID_MEASUREMENT) == BIT_VALID_MEASUREMENT)
                {
                    xEventGroupClearBits(hdrive_event_group_array[hdrive_ch], BIT_VALID_MEASUREMENT);
                    // ESP_LOGE(TAG,"FILTERING\n");
                    mean_control[hdrive_ch].actual_single_mean_value = dataAcq[hdrive_ch].mean;
                    mean_control_filter_displayData(&mean_control[hdrive_ch], coeffA_fast, coeffB_fast);
                }
                // wait for hdrive to signal non-idleness
                if ((xEventGroupGetBits(hdrive_event_group_array[hdrive_ch]) & BIT_HDRIVE_RUNNING) == BIT_HDRIVE_RUNNING)
                {
                    // check which control mode is active.
                    if (current_control_mode[hdrive_ch] == CONTROL_MODE_MANUAL)
                    {
                        ESP_LOGI(TAG, "[%i] changing control Mode: IDLE --> MANUAL", hdrive_ch);
                        // if Manual Mode, just switch to manual mode in state machine.

                        //...
                        step_mean_control[hdrive_ch] = MC_MANUAL;
                    }
                    else if (current_control_mode[hdrive_ch] == CONTROL_MODE_MEAN)
                    {
                        ESP_LOGI(TAG, "[%i] changing control Mode: IDLE --> MEAN/AUTO", hdrive_ch);
                        // if mean mode, initialize filters, controllers here:

                        // initialize mean filter with fast coefficiants
                        mean_control[hdrive_ch].actual_mean_value = wifi_fb[hdrive_ch].setpoint_mean_control;
                        mean_control[hdrive_ch].actual_mean_x[0] = mean_control[hdrive_ch].actual_mean_value;
                        mean_control[hdrive_ch].actual_mean_x[1] = mean_control[hdrive_ch].actual_mean_value;
                        mean_control[hdrive_ch].actual_mean_y[0] = mean_control[hdrive_ch].actual_mean_value;
                        mean_control[hdrive_ch].actual_mean_y[1] = mean_control[hdrive_ch].actual_mean_value;
                        mean_control[hdrive_ch].actual_mean_a[0] = mean_filter_fast_a[0];
                        mean_control[hdrive_ch].actual_mean_a[1] = mean_filter_fast_a[1];
                        mean_control[hdrive_ch].actual_mean_b[0] = mean_filter_fast_b[0];
                        mean_control[hdrive_ch].actual_mean_b[1] = mean_filter_fast_b[1];
                        ifilter_cycle_counter[hdrive_ch] = 0;
                        switchBit[hdrive_ch] = false;
                        switchBitCounter[hdrive_ch] = 0;

                        // initialize setpoint filter 2
                        mean_control[hdrive_ch].mean_setpoint = wifi_fb[hdrive_ch].setpoint_mean_control;
                        mean_control[hdrive_ch].x[0] = mean_control[hdrive_ch].mean_setpoint;
                        mean_control[hdrive_ch].x[1] = mean_control[hdrive_ch].mean_setpoint;
                        mean_control[hdrive_ch].x[2] = mean_control[hdrive_ch].mean_setpoint;
                        mean_control[hdrive_ch].y[0] = mean_control[hdrive_ch].mean_setpoint;
                        mean_control[hdrive_ch].y[1] = mean_control[hdrive_ch].mean_setpoint;
                        mean_control[hdrive_ch].y[2] = mean_control[hdrive_ch].mean_setpoint;
                        //
                        // initialize controller, with last control value. "setpointClosed" of hdrive-object should actually be exactly
                        // the last valid setpoint before any kind of idleness/switching of mode
                        mean_control[hdrive_ch].control_integral = hdrive[hdrive_ch].setpointClosed;
                        mean_control[hdrive_ch].mean_control_output = hdrive[hdrive_ch].setpointClosed;
                        // ESP_LOGE(TAG, "[%i] Controller has benn initialized with: %.2f", hdrive_ch, mean_control[hdrive_ch].mean_control_output);
                        //...
                        step_mean_control[hdrive_ch] = MC_MEAN;
                    }
                }
                if ((xEventGroupGetBits(wifi_event_group) & BIT_REQ_CHANGE_TO_MODE_1) == BIT_REQ_CHANGE_TO_MODE_1)
                {
                    ESP_LOGI(TAG, "[%i] IN IDLE: changing control Mode: MANUAL --> MEAN/AUTO, but still idle", hdrive_ch);
                    // wifi bit cleared before foor loop
                    // change mode for hdrive.c
                    xEventGroupClearBits(hdrive_event_group_array[hdrive_ch], BIT_CONTROL_MODE_MANUAL);
                    xEventGroupSetBits(hdrive_event_group_array[hdrive_ch], BIT_CONTROL_MODE_MEAN);
                    current_control_mode[hdrive_ch] = CONTROL_MODE_MEAN;
                }
                else if ((xEventGroupGetBits(wifi_event_group) & BIT_REQ_CHANGE_TO_MODE_0) == BIT_REQ_CHANGE_TO_MODE_0)
                {
                    ESP_LOGI(TAG, "[%i] IN IDLE: changing control Mode: MEAN/AUTO --> MANUAL, but still idle", hdrive_ch);
                    // change mode for hdrive.c
                    xEventGroupClearBits(hdrive_event_group_array[hdrive_ch], BIT_CONTROL_MODE_MEAN);
                    xEventGroupSetBits(hdrive_event_group_array[hdrive_ch], BIT_CONTROL_MODE_MANUAL);
                    // change global mode datapoint
                    current_control_mode[hdrive_ch] = CONTROL_MODE_MANUAL;
                }
                break;
            // ==========================================================================================
            // MANUAL
            // ==========================================================================================
            case MC_MANUAL:
                // in Manual-Mode, wait for hallS-trigger to filter display data, nothing else
                //
                // filter display data:
                if ((xEventGroupGetBits(hdrive_event_group_array[hdrive_ch]) & BIT_VALID_MEASUREMENT) == BIT_VALID_MEASUREMENT)
                {
                    xEventGroupClearBits(hdrive_event_group_array[hdrive_ch], BIT_VALID_MEASUREMENT);
                    // ESP_LOGE(TAG,"FILTERING\n");
                    mean_control[hdrive_ch].actual_single_mean_value = dataAcq[hdrive_ch].mean;
                    mean_control_filter_displayData(&mean_control[hdrive_ch], coeffA_fast, coeffB_fast);
                }
                //
                // listen if hdrive has gone idle:
                if ((xEventGroupGetBits(hdrive_event_group_array[hdrive_ch]) & BIT_HDRIVE_RUNNING) == 0)
                {
                    // just stop all activity and go to idle state. leave global control mode as it is
                    ESP_LOGW(TAG, "[%i] Hdrive mean control has gone IDLE", hdrive_ch);
                    step_mean_control[hdrive_ch] = MC_IDLE;
                    //
                    // listen to wifi for change of control mode
                }
                else if ((xEventGroupGetBits(wifi_event_group) & BIT_REQ_CHANGE_TO_MODE_1) == BIT_REQ_CHANGE_TO_MODE_1)
                {
                    ESP_LOGI(TAG, "[%i] changing control Mode: MANUAL --> MEAN/AUTO", hdrive_ch);

                    // initialize mean filter with fast coefficiants
                    mean_control[hdrive_ch].actual_mean_value = wifi_fb[hdrive_ch].setpoint_mean_control;
                    mean_control[hdrive_ch].actual_mean_x[0] = mean_control[hdrive_ch].actual_mean_value;
                    mean_control[hdrive_ch].actual_mean_x[1] = mean_control[hdrive_ch].actual_mean_value;
                    mean_control[hdrive_ch].actual_mean_y[0] = mean_control[hdrive_ch].actual_mean_value;
                    mean_control[hdrive_ch].actual_mean_y[1] = mean_control[hdrive_ch].actual_mean_value;
                    mean_control[hdrive_ch].actual_mean_a[0] = mean_filter_fast_a[0];
                    mean_control[hdrive_ch].actual_mean_a[1] = mean_filter_fast_a[1];
                    mean_control[hdrive_ch].actual_mean_b[0] = mean_filter_fast_b[0];
                    mean_control[hdrive_ch].actual_mean_b[1] = mean_filter_fast_b[1];
                    ifilter_cycle_counter[hdrive_ch] = 0;
                    switchBit[hdrive_ch] = false;
                    switchBitCounter[hdrive_ch] = 0;
                    //
                    // initialize setpoint filter 2
                    mean_control[hdrive_ch].mean_setpoint = wifi_fb[hdrive_ch].setpoint_mean_control;
                    mean_control[hdrive_ch].x[0] = mean_control[hdrive_ch].mean_setpoint;
                    mean_control[hdrive_ch].x[1] = mean_control[hdrive_ch].mean_setpoint;
                    mean_control[hdrive_ch].x[2] = mean_control[hdrive_ch].mean_setpoint;
                    mean_control[hdrive_ch].y[0] = mean_control[hdrive_ch].mean_setpoint;
                    mean_control[hdrive_ch].y[1] = mean_control[hdrive_ch].mean_setpoint;
                    mean_control[hdrive_ch].y[2] = mean_control[hdrive_ch].mean_setpoint;
                    //
                    // initialize controller with last control value (see in MC_IDLE)
                    mean_control[hdrive_ch].control_integral = hdrive[hdrive_ch].setpointClosed;
                    mean_control[hdrive_ch].mean_control_output = hdrive[hdrive_ch].setpointClosed;
                    //
                    // change mode for hdrive.c
                    xEventGroupClearBits(hdrive_event_group_array[hdrive_ch], BIT_CONTROL_MODE_MANUAL);
                    xEventGroupSetBits(hdrive_event_group_array[hdrive_ch], BIT_CONTROL_MODE_MEAN);
                    // change global mode datapoint
                    current_control_mode[hdrive_ch] = CONTROL_MODE_MEAN;
                    // ESP_LOGI(TAG, "[%i] Hdrive Mode Changed to AUTO", hdrive_ch);
                    step_mean_control[hdrive_ch] = MC_MEAN;
                }
                break;
            // ==========================================================================================
            // MEAN
            // ==========================================================================================
            case MC_MEAN:
                // in Mean-Mode, wait for hallS-trigger to filter display data, filter control data and do one cycle of control
                //..
                if ((xEventGroupGetBits(hdrive_event_group_array[hdrive_ch]) & BIT_VALID_MEASUREMENT) == BIT_VALID_MEASUREMENT)
                {
                    xEventGroupClearBits(hdrive_event_group_array[hdrive_ch], BIT_VALID_MEASUREMENT);
                    //
                    // if counter has reached x cycles, change mean filter time constant
                    if (ifilter_cycle_counter[hdrive_ch] <= 20)
                    {
                        ifilter_cycle_counter[hdrive_ch]++;
                    }
                    // or, if switchBit detects saturation problems, switch to/stay in fast coefficiant
                    else if (switchBit[hdrive_ch])
                    {
                        mean_control[hdrive_ch].actual_mean_a[0] = mean_filter_fast_a[0];
                        mean_control[hdrive_ch].actual_mean_a[1] = mean_filter_fast_a[1];
                        mean_control[hdrive_ch].actual_mean_b[0] = mean_filter_fast_b[0];
                        mean_control[hdrive_ch].actual_mean_b[1] = mean_filter_fast_b[1];
                    }
                    else
                    {
                        mean_control[hdrive_ch].actual_mean_a[0] = mean_filter_slow_a[0];
                        mean_control[hdrive_ch].actual_mean_a[1] = mean_filter_slow_a[1];
                        mean_control[hdrive_ch].actual_mean_b[0] = mean_filter_slow_b[0];
                        mean_control[hdrive_ch].actual_mean_b[1] = mean_filter_slow_b[1];
                    }
                    // update values to mean filter:
                    mean_control[hdrive_ch].actual_single_mean_value = dataAcq[hdrive_ch].mean;
                    // update values to setpoint filter:
                    mean_control[hdrive_ch].mean_setpoint = wifi_fb[hdrive_ch].setpoint_mean_control;
                    // filter display & mean data & setpoint:
                    mean_control_filter_displayData(&mean_control[hdrive_ch], coeffA_fast, coeffB_fast);
                    // filter setpoint
                    mean_control_filter_setpoint_2(&mean_control[hdrive_ch]);
                    // filter mean
                    mean_control_filter_mean_2(&mean_control[hdrive_ch]);
                    // control loop:
                    mean_control_controller(&mean_control[hdrive_ch], mean_control_KP, mean_control_Tn, mean_control_satHigh, mean_control_satLow);
                    // signal new setpoint closed
                    set_event_bit(hdrive_event_group_array[hdrive_ch], BIT_HDRIVE_CHANGE_AUTO_SETPOINT);
                    //
                    // **********************************************************************************************
                    // check if controller is in saturation for too long. if so, switch to fast filter coefficiants

                    // count up if controller is in high saturation...
                    if (mean_control[hdrive_ch].mean_control_output >= mean_control_satHigh)
                    {
                        switchBitCounter[hdrive_ch]++;
                    } //... count back down if not in high saturation anymore (but has been)
                    else if ((mean_control[hdrive_ch].mean_control_output <= mean_control_satHigh - saturation_hysteresis) && switchBitCounter[hdrive_ch] > 0)
                    {
                        switchBitCounter[hdrive_ch]--;
                    } // or, count down if controller is in low saturation...
                    else if (mean_control[hdrive_ch].mean_control_output <= mean_control_satLow)
                    {
                        switchBitCounter[hdrive_ch]--;
                    } // ... count back up if not in low saturation anymore (but has been)
                    else if ((mean_control[hdrive_ch].mean_control_output >= mean_control_satLow + saturation_hysteresis) && switchBitCounter[hdrive_ch] < 0)
                    {
                        switchBitCounter[hdrive_ch]++;
                    }
                    // check counter. If in saturation to long, set switchBit. If in boundaries for long enough, deactivate switchBit
                    if (switchBitCounter[hdrive_ch] >= saturation_threshhold)
                    {
                        switchBitCounter[hdrive_ch] = saturation_threshhold;
                        switchBit[hdrive_ch] = true;
                    }
                    else if (switchBitCounter[hdrive_ch] <= -saturation_threshhold)
                    {
                        switchBitCounter[hdrive_ch] = -saturation_threshhold;
                        switchBit[hdrive_ch] = true;
                    }
                    else if (switchBitCounter[hdrive_ch] == 0)
                    {
                        switchBit[hdrive_ch] = false;
                    }
                    // check for positive edge of switchbit (in HIGH saturation) --> signal to color sensor output for an output impulse
                    if ((switchBit[hdrive_ch] == true) && (switchBitOld[hdrive_ch] == false) && (switchBitCounter[hdrive_ch] >= saturation_threshhold))
                    {
                        // ESP_LOGE(TAG, "mean control triggered channel [%i]\n", hdrive_ch);
                        color_sensor_trigger(hdrive_ch);
                    }
                    // timeshift
                    switchBitOld[hdrive_ch] = switchBit[hdrive_ch];
                    // **********************************************************************************************
                    // print if hdrive 0 active control & filter cycle
                    //

                    if (hdrive_ch == 0)
                    {
                        // printf("%.3f;%.3f;%.3f;%.3f;%.3f;%.3f\n",mean_control[0].mean_setpoint,mean_control[0].mean_setpoint_filtered,mean_control[0].actual_single_mean_value,mean_control[0].actual_mean_value,mean_control[0].mean_control_output,mean_control[0].control_integral);
                        // printf("switchBit: %i = switchBitCounter: %i\n", switchBit[0], switchBitCounter[0]);
                    }
                }
                // listen if hdrive has gone idle:
                if ((xEventGroupGetBits(hdrive_event_group_array[hdrive_ch]) & BIT_HDRIVE_RUNNING) == 0)
                {
                    // just stop all activity and go to idle state. leave global control mode as it is
                    ESP_LOGW(TAG, "[%i] Hdrive mean control has gone IDLE", hdrive_ch);
                    step_mean_control[hdrive_ch] = MC_IDLE;
                    //
                    // listen to wifi for change of control mode
                }
                else if ((xEventGroupGetBits(wifi_event_group) & BIT_REQ_CHANGE_TO_MODE_0) == BIT_REQ_CHANGE_TO_MODE_0)
                {
                    ESP_LOGI(TAG, "[%i] changing control Mode: MEAN/AUTO --> MANUAL", hdrive_ch);
                    
                    // change mode for hdrive.c
                    xEventGroupClearBits(hdrive_event_group_array[hdrive_ch], BIT_CONTROL_MODE_MEAN);
                    xEventGroupSetBits(hdrive_event_group_array[hdrive_ch], BIT_CONTROL_MODE_MANUAL);
                    // change global mode datapoint
                    current_control_mode[hdrive_ch] = CONTROL_MODE_MANUAL;
                    // ESP_LOGE(TAG, "[%i] Hdrive Mode Changed to MANUAL\n", hdrive_ch);
                    step_mean_control[hdrive_ch] = MC_MANUAL;
                }
                break;
            // ==========================================================================================
            // DEFAULT
            // ==========================================================================================
            default:
                //
                break;
            } // end switch
        }     // end for

        // ==========================================================================================
        // print stuff every x cycles, i.e. every x ms
        if (cycleCount >= 5)
        {
            // printf("xTaskGetTickCount: %i\n",xTaskGetTickCount());

            // printf("[step] -%i---%i-\n",step_mean_control[0],step_mean_control[1]);

            // printf("[switchbit] -%i---%i-\n",switchBit[0],switchBit[1]);
            // printf("[switchbitCounter] -%i---%i-\n",switchBitCounter[0],switchBitCounter[1]);
            // printf("[dis:%.2f] - single: %.2f - ist: %.2f - soll: %.2f - soll_f: %.2f - I: %.2f - stell: %.1f\n",mean_control[0].display_mean,mean_control[0].actual_single_mean_value,mean_control[0].actual_mean_value,mean_control[0].mean_setpoint,mean_control[0].mean_setpoint_filtered,mean_control[0].control_integral,mean_control[0].mean_control_output);

            // = controller watch =
            // printf("%.2f;%.2f;%.2f;%.2f\n",mean_control[0].mean_setpoint,mean_control[0].mean_setpoint_filtered,mean_control[0].actual_mean_value,mean_control[0].mean_control_output);

            // printf("Set: %.3f = Setf: %.3f = Single: %.3f = act: %.3f = contVal: %.3f = Integral: %.3f\n",mean_control[0].mean_setpoint,mean_control[0].mean_setpoint_filtered,mean_control[0].actual_single_mean_value,mean_control[0].actual_mean_value,mean_control[0].mean_control_output,mean_control[0].control_integral);
            // printf("[1] Set: %.3f = Setf: %.3f = Single: %.3f = act: %.3f = contVal: %.3f = Integral: %.3f\n",mean_control[1].mean_setpoint,mean_control[1].mean_setpoint_filtered,mean_control[1].actual_single_mean_value,mean_control[1].actual_mean_value,mean_control[1].mean_control_output,mean_control[1].control_integral);

            //

            // color_sensor_trigger(0);

            // ESP_LOGE(TAG, "=== 3s? ===");
            cycleCount = 0;
        }
        // ====================================================
        // delay until next cycle, increment cycle counter
        cycleCount++;
        vTaskDelayUntil(&previousWakeTime, (configTICK_RATE_HZ / cycleFrequency));
        // vTaskDelay(200);
        //  ====================================================
    }
}