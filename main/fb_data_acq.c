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
#include "esp_event_loop.h"
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
#include "fb_data_acq.h"
#include "fb_hdrive.h"
#include "fb_sd_card.h"
#include "fb_esp_time.h"

//
static const char *TAG = "fb_data_acq";
//
// define Analog Inputs
#define ADC1_BTSR1_INPUT ADC1_CHANNEL_0
#define ADC1_BTSR2_INPUT ADC1_CHANNEL_3
#define ADC1_ENC_INPUT ADC1_CHANNEL_6
//
// === PARAMETERS ======================================================
// Parameters for mean value calculation
float fromDegree = 340;
float toDegree = 350;
//
#define ENC_MIN_DEFAULT 500;  // begin min @ 0.3V = ca. 500
#define ENC_MAX_DEFAULT 2000; // end max @ 1.5V = ca. 3300
//
// use every 4th measurement, so skip 3
int postDataCycle = 3; // only every x measurement is actually posted to be stored on sd card --> 1 actualy means, skip 1 measurement...
//
// === END PARAMETERS ==================================================
//

// =====================================================================
// fuction to initialize all gpio's, specifically for data acquisition
void dataAcq_init_gpio()
{
    // configure analog Inputs, ADC1, ATTEN_DB_6 for ~0-2.2V
    adc1_config_width(ADC_WIDTH_12Bit);
    adc1_config_channel_atten(ADC1_BTSR1_INPUT, ADC_ATTEN_DB_6);
    adc1_config_channel_atten(ADC1_BTSR2_INPUT, ADC_ATTEN_DB_6);
    adc1_config_channel_atten(ADC1_ENC_INPUT, ADC_ATTEN_DB_6);
}
// =====================================================================
// fuction to calculate yarn tension out of 12-bit analog input
// return yarn tension in [cN]
float dataAcq_get_yarn_tension(int adc_channel)
{
    float a = 0.0094, b = 1.61; // approximated function for 12-bit adc, 0.1V-10V = 0cN-50cN with adc calib. (see matlab/documentation)
    int rawData = adc1_get_raw(adc_channel);
    return (float)rawData * a + b;
    // return ((float)rawData * a + b) / 2; // use 25cN BTSR Sensor
}
// =====================================================================
// function to calculate mean value at certain time (e.g. 348°-352°)
// of ONE dataset of type "dataAcq_t". Works with adress, no return value
// Needs dataAcq object, from/to degree values [°], amount of samples in
// curent measurement
void dataAcq_calc_mean(dataAcq_t *pDataAcq, float fromDeg, float toDeg, int amount_of_samples)
{
    // check if parameters are valid
    if ((toDeg < fromDeg) || (fromDeg < 0) || (toDeg > 360))
    {
        printf("dataAcq_calc_mean function parameters not valid! PANIC!");
        return;
    }
    // calculate indexes of single measurement dependent on measurement length
    // because we have 1kHz measurement sample rate, we index of data is in ms!
    int fromMS = (int)round((float)amount_of_samples / 360.0 * fromDeg);
    int toMS = (int)round((float)amount_of_samples / 360.0 * toDeg);
    // temp variable for sum of valid samples
    float sum = 0;
    // amount of samples for mean calculation
    int num_valid_samples = (toMS - fromMS) + 1; // because, at least one sample must be evaluated!
    // check if valid amount of samples:
    if (num_valid_samples >= 1)
    {
        // calc sum of all valid samples
        for (int n = fromMS; n <= toMS; n++)
        {
            sum += (*pDataAcq).data[n];
        }
        // calc mean of all valid samples
        (*pDataAcq).mean = sum / (float)num_valid_samples;
    }
}
// =====================================================================
// function to filter ONE dataset of type "dataAcq_t"
// works with addresses, no return values
void dataAcq_filter_data(dataAcq_t *pDataAcq)
{
    // float A = 1, B = 0; // no filter
    //  float A = 0.6321, B = 0.3679; // PT1, ZOH, T1=1ms @ Ts=1ms
    float A = 0.1813, B = 0.8187; // PT1, ZOH, T1=5ms @ Ts=1ms
    // float A = 0.01, B = 0.99; // PT1, ZOH, T1=100ms @ Ts=1ms
    // float A = 0.3297, B = 0.6703; // PT1, ZOH, T1=5ms @ Ts=2ms
    // float A = 0.8647, B = 0.1353; // PT1, ZOH, T1=1ms @ Ts=2ms
    //  calculate new filtered value through actual value and old filtered value
    (*pDataAcq).act_value_filtered = (*pDataAcq).act_value * A + (*pDataAcq).act_value_filtered_old_array[0] * B;
    // copy actual filtered value into old
    (*pDataAcq).act_value_filtered_old_array[0] = (*pDataAcq).act_value_filtered;
    //...
}
// =====================================================================
// function to calibrate encoder
void dataAcq_calibrate_encoder(adc1_channel_t adc_ch, int16_t *pmin, int16_t *pmax)
{
    // read analog input
    int rawData = adc1_get_raw(adc_ch);
    // shift max value, reset if clearly out of bounds
    if (rawData > 4095)
    {
        *pmax = ENC_MAX_DEFAULT;
    }
    else if (rawData > *pmax)
    {
        *pmax = rawData;
    }
    // shift min value, reset if clearly out of bounds
    if (rawData < 0)
    {
        *pmin = ENC_MIN_DEFAULT;
    }
    else if (rawData < *pmin)
    {
        *pmin = rawData;
    }
}
// =====================================================================
// function to get machine angle from analog encoder
// method used in V1
void dataAcq_get_machine_angle(adc1_channel_t adc_ch, loom_t *ploom, float *oldtempAngle, float *oldActualAngle)
{
    // read analog input
    int rawData = adc1_get_raw(adc_ch);
    // temporary variables
    float tempAngle, dif, expectedAngle, shiftedAngle, a, b;
    //
    // calculate a coefficiant
    // check boundaries. enc_max - enc_min may not be zero or negative
    if (((*ploom).enc_max - (*ploom).enc_min) <= 0)
    {
        a = 0;
    }
    else
    {
        a = 360.0 / (float)((*ploom).enc_max - (*ploom).enc_min);
    }
    // calculate b coefficiant
    b = -a * (*ploom).enc_min;
    // calculate temporary angle without offset
    tempAngle = (float)(rawData)*a + b;
    // has to be between 0° and 360° at this point!
    if (tempAngle < 0.0)
    {
        tempAngle = 0;
    }
    else if (tempAngle > 360.0)
    {
        tempAngle = 360.0;
    }
    // uncomment next line if machine turns in other direction
    // tempAngle = 360.0 - tempAngle;
    //
    // calculate simple differential
    dif = tempAngle - *oldtempAngle;
    if (dif >= 0)
    {
        ploom->differential = dif; // if differential is in plausible range, copy to datastruct
    }
    // if differential is negativ (to a certain degree), then we also expect a very small angle
    // because "sharktooth" has just dropped from high to low.
    // if angle has not dropped enough, force angle to zero to avoid "fake" angles
    if ((dif < -18.0) && (tempAngle > 10.0)) // diff 18° = 5% of 360° // 25.06.21 i think we can use >350, as only around this angle do we expect a zero crossing
    {
        expectedAngle = 0;
    }
    else if (dif > 36.0) // a positive jump of >10% is not possible (can happen if sensor has noise @ zero crossing). hold angle if so
    {                    // 25.06.21 so if tempAngle does not jump to 0, in the next step we will have dif>x and expected angle will stay 0, until actual angle is <10!
        expectedAngle = *oldtempAngle;
    }
    else
    {
        expectedAngle = tempAngle; // maybe, just maybe, use a moving average filter at this point. If hardware-filter is not enough
    }
    // shift with offset
    shiftedAngle = expectedAngle - (float)(*ploom).angleOffset;
    // wrap around 360°
    int mod = (int)shiftedAngle % 360;
    if (shiftedAngle > 360.0)
    {
        (*ploom).actualAngle = (float)(mod);
    }
    else if (shiftedAngle < 0.0)
    {
        (*ploom).actualAngle = 360.0 + (float)(mod);
    }
    else
    {
        (*ploom).actualAngle = floor(shiftedAngle);
    }
    // timeshift
    *oldtempAngle = tempAngle;
    *oldActualAngle = (*ploom).actualAngle; // not needed i think...
}
// method that hold expected angle high until dif is positiv again --> delayed zero detection
void dataAcq_get_machine_angle_2(adc1_channel_t adc_ch, loom_t *ploom, float *oldtempAngle, float *oldexpectedAngle)
{
    // read analog input
    int rawData = adc1_get_raw(adc_ch);
    // temporary variables
    float tempAngle, dif, expectedAngle, shiftedAngle, a, b;
    // calculate a coefficiant. check boundaries. enc_max - enc_min may not be zero or negative
    if (((*ploom).enc_max - (*ploom).enc_min) <= 0)
    {
        a = 0;
    }
    else
    {
        a = 360.0 / (float)((*ploom).enc_max - (*ploom).enc_min);
    }
    // calculate b coefficiant
    b = -a * (*ploom).enc_min;
    // calculate temporary angle without offset
    tempAngle = (float)(rawData)*a + b;
    // has to be between 0° and 360° at this point!
    if (tempAngle < 0.0)
    {
        tempAngle = 0;
    }
    else if (tempAngle > 360.0)
    {
        tempAngle = 360.0;
    }
    // uncomment next line if machine turns in other direction
    tempAngle = 360.0 - tempAngle; // Telaio-035 uncommented
    //
    // calculate simple differential
    dif = tempAngle - *oldtempAngle;
    if (dif >= 0)
    {
        ploom->differential = dif; // if differential is in plausible range, copy to datastruct
    }
    // get plausible expected angle. This method will hold expected angle high until temp angle
    // has droped to 0 and is going back up again. 0-Crossing occurs only when tempangle
    // is going back up (meaning we detect zero crossing later than it actually occurs)
    if ((dif < 5) && (dif > 0))
    {
        expectedAngle = tempAngle;
    }
    else
    {
        expectedAngle = *oldexpectedAngle;
    }
    // shift with offset
    shiftedAngle = expectedAngle - (float)(*ploom).angleOffset;
    // wrap around 360°
    int mod = (int)shiftedAngle % 360;
    if (shiftedAngle > 360.0)
    {
        (*ploom).actualAngle = (float)(mod);
    }
    else if (shiftedAngle < 0.0)
    {
        (*ploom).actualAngle = 360.0 + (float)(mod);
    }
    else
    {
        (*ploom).actualAngle = floor(shiftedAngle);
    }
    // timeshift
    *oldtempAngle = tempAngle;
    *oldexpectedAngle = expectedAngle;
}
// method that forces expected angle to zero if dif2 is under certain value --> faster zero detection
void dataAcq_get_machine_angle_3(adc1_channel_t adc_ch, loom_t *ploom, float *oldtempAngle, float *oldexpectedAngle)
{
    // read analog input
    int rawData = adc1_get_raw(adc_ch);
    // temporary variables
    float tempAngle, dif, expectedAngle, shiftedAngle, a, b, dif2;
    // calculate a coefficiant. check boundaries. enc_max - enc_min may not be zero or negative
    if (((*ploom).enc_max - (*ploom).enc_min) <= 0)
    {
        a = 0;
    }
    else
    {
        a = 360.0 / (float)((*ploom).enc_max - (*ploom).enc_min);
    }
    // calculate b coefficiant
    b = -a * (*ploom).enc_min;
    // calculate temporary angle without offset
    tempAngle = (float)(rawData)*a + b;
    // has to be between 0° and 360° at this point!
    if (tempAngle < 0.0)
    {
        tempAngle = 0;
    }
    else if (tempAngle > 360.0)
    {
        tempAngle = 360.0;
    }
    // uncomment next line if machine turns in other direction
    // tempAngle = 360.0 - tempAngle;
    //
    // calculate simple differential
    dif = tempAngle - *oldtempAngle;
    dif2 = tempAngle - *oldexpectedAngle; // dif2 is differential between real value and estimated value
    if (dif >= 0)
    {
        ploom->differential = dif; // if differential is in plausible range, copy to datastruct
    }
    // get plausible expected angle. This method will hold expected angle high until temp angle
    // has droped to 0 and is going back up again. 0-Crossing occurs only when tempangle
    // is going back up (meaning we detect zero crossing later than it actually occurs)
    if ((dif < 5.0) && (dif > 0.0) && (dif2 < 180.0))
    {
        expectedAngle = tempAngle;
    }
    else
    {
        expectedAngle = *oldexpectedAngle;
    }
    // if difference between real vand estimated value is negative and relatively high,
    // we pull estimated angle down to zero. This way we will detect zero crossing earlier,
    // however the estimated angle may stay zero for a time
    if (dif2 < -181)
    {
        expectedAngle = 0;
    }
    // shift with offset
    shiftedAngle = expectedAngle - (float)(*ploom).angleOffset;
    // wrap around 360°
    int mod = (int)shiftedAngle % 360;
    if (shiftedAngle > 360.0)
    {
        (*ploom).actualAngle = (float)(mod);
    }
    else if (shiftedAngle < 0.0)
    {
        (*ploom).actualAngle = 360.0 + (float)(mod);
    }
    else
    {
        (*ploom).actualAngle = floor(shiftedAngle);
    }
    // timeshift
    *oldtempAngle = tempAngle;
    *oldexpectedAngle = expectedAngle;
}
// =====================================================================
// function to catch zero crossing
void dataAcq_catch_zero_angle(float *angle, float *oldAngle, bool *flag, uint8_t *counter, TickType_t CycleFreq)
{
    // function to set flag if zero angle is crossed
    float cycFreq = (float)CycleFreq; // [Hz] cycle freq of zero cross detection
    float stallTime = 10;             // [ms] minimum time between zero crossing detection
    // detect zero crossing through large diff from old and actual value
    // float diff = fabsf(*angle - *oldAngle);
    float diff = *angle - *oldAngle;
    // float diff2 = *angle - *oldAngle;

    // printf("%.2f\n", diff2);
    // ESP_LOGE(TAG, "%.2f, detected angle: %.2f == old angle: %.2f == %i\n", diff2, (*angle),(*oldAngle), xTaskGetTickCount());

    // make sure zero crossing is detected only every few ms ("prellen" verhindern)

    if ((*counter) < (int)(cycFreq / 1000.0 * stallTime))
    {
        (*counter)++;
        // use threshold 300°
    }
    else if (diff < -300.0)
    {
        *flag = true;
        //*angle = 0;

        // ESP_LOGE(TAG, "detected angle: %.2f == old angle: %.2f == even older angle: %.2f == %i\n", (*angle), (*oldAngle), (*evenOlderAngle), xTaskGetTickCount());

        // ESP_LOGE(TAG, "%.2f\n", (*angle));

        // printf("detected, old/actual angle: %.2f = %.2f === %i\n", *oldAngle, *angle, xTaskGetTickCount());
        (*counter) = 0;
    }
    // timeshift
    *oldAngle = *angle;
}

// =====================================================================
// data-acquisition task
// =====================================================================
void dataAcq_task(void *arg)
{
    // =================================================================
    // local variables
    // =================================================================
    // variable to store last tick time for exakt cycle time. Updates every cycle
    TickType_t previousWakeTime0 = xTaskGetTickCount();
    TickType_t cycleFrequency = 1000;                 // actual cycle frequency [Hz]
    float cycleTime = 1000.0 / (float)cycleFrequency; // actual cycle time [ms] of dataAcq loop
    uint64_t actual_counter_value = 0;                // counter value for testing/debugging
    uint64_t before_counter_value = 0;                // counter value for testing/debugging
    uint64_t after_counter_value = 0;                 // counter value for testing/debugging
    int difference_dataAcq = 0;
    uint64_t difference_dataAcq_array[15] = {};
    int difference_idx = 0;
    int cycleCount = 0; // count switch cycles
    int idx_data = 0;   // index for 0-max datapoint. Only one index for all Sensors because either all are acquiring or none
    //
    bool calibrate_enc_old = wifi_fb[0].calibrate_enc;
    // dt1 variables/parameters
    float Ts = cycleTime / 1000.0;
    float T1 = 0.25;
    filter_dt1_t dt1_angular_vel;
    dt1_angular_vel.input = 0;
    dt1_angular_vel.output = 0;
    dt1_angular_vel.b[0] = -2.0;
    dt1_angular_vel.b[1] = 2.0;
    dt1_angular_vel.a[0] = -2.0 * T1 + Ts;
    dt1_angular_vel.a[1] = 2.0 * T1 + Ts;
    dt1_angular_vel.x[0] = 0;
    dt1_angular_vel.x[1] = 0;
    dt1_angular_vel.y[0] = 0;
    dt1_angular_vel.y[1] = 0;
    //
    data_to_store_t testdata1, dataToPost;
    //
    uint8_t zeroCrossMem = 0;      // memory used for zero crossing detection
    float OldAngle = 0.0;          // memory value from encoder
    float getAngleMem[2] = {0, 0}; // memory for getting Machine Angle (old angle, old dif etc.)
    bool hallS_flag = false;       // Machine angle 0° has been detected
    //
    int postDataCounter = 0;
    int fullQueueCounter = 0;

    // =================================================================
    // Initialize stuff
    // =================================================================
    dataAcq_init_gpio();
    dataAcq_step_enum_t step_dataAcq = DA_IDLE;
    xEventGroupSetBits(dataAcq_event_group, BIT_DATA_ACQ_IS_IDLE);
    // init remanent variable
    loom.angleOffset = wifi_fb[0].angleOffset;
    // =================================================================
    // WHILE LOOP
    // =================================================================
    while (1)
    {
        // ========================================================================================
        // get yarn tension of both channels and filter them
        // ========================================================================================
        // channel 1 (ADC1 CH0)
        dataAcq[0].act_value = dataAcq_get_yarn_tension(ADC1_BTSR1_INPUT);
        dataAcq_filter_data(&dataAcq[0]);
        // channel 2 (ADC1 CH3)
        dataAcq[1].act_value = dataAcq_get_yarn_tension(ADC1_BTSR2_INPUT);
        dataAcq_filter_data(&dataAcq[1]);
        // ========================================================================================
        // get current machine angle and catch 0°
        // ========================================================================================
        // Update Angle offset if demanded by wifi (done with event group)
        if (check_clear_event_bit(wifi_event_group, BIT_SET_ANGLE_OFFSET))
        {
            loom.angleOffset = wifi_fb[0].angleOffset;
            ESP_LOGW(TAG, "New machine angle offset from wifi: %i", loom.angleOffset);
        }
        // watch for positive flank when starting calibration --> set default min/max.
        // (could also be done with event group)
        if (wifi_fb[0].calibrate_enc == true && calibrate_enc_old == false)
        {
            loom.enc_min = ENC_MIN_DEFAULT;
            loom.enc_max = ENC_MAX_DEFAULT;
        }
        calibrate_enc_old = wifi_fb[0].calibrate_enc;
        // switch on calibration if demanded
        if (wifi_fb[0].calibrate_enc)
        {
            dataAcq_calibrate_encoder(ADC1_ENC_INPUT, &loom.enc_min, &loom.enc_max);
        }
        // get machine angle every cycle
        dataAcq_get_machine_angle_2(ADC1_ENC_INPUT, &loom, &getAngleMem[0], &getAngleMem[1]);
        // catch zero crossing
        dataAcq_catch_zero_angle(&loom.actualAngle, &OldAngle, &hallS_flag, &zeroCrossMem, cycleFrequency);
        //
        // ========================================================================================
        // detect machine idleness
        // ========================================================================================
        // calc machine velocity
        dt1_angular_vel.input = loom.actualAngle;
        filter_dt1_360(&dt1_angular_vel);
        // decide if projetile loom is running or not
        if (dt1_angular_vel.output >= 360.0)
        {
            loom.machine_running = true;
        }
        else if (dt1_angular_vel.output <= 180)
        {
            loom.machine_running = false;
        }
        // alternative: if data is aquiring, then machine is running.
        // this means that running-status is only detetcted after zero crossing!
        // if (check_event_bit(dataAcq_event_group, BIT_DATA_ACQ_IS_ACTIVE))
        // {
        //     loom.machine_running = true;
        // }
        // else
        // {
        //     loom.machine_running = false;
        // }
        // ========================================================================================
        // Begin state machine
        // ========================================================================================
        //
        switch (step_dataAcq)
        {

        case DA_IDLE:
            // wait for HallS trigger
            if (hallS_flag == true)
            {
                // ============
                ESP_LOGW(TAG, "==== ZeroCross detected. Trying to Aquire Data ====");
                // ============
                // first dataPoint:
                idx_data = 0;
                dataAcq[0].data[idx_data] = dataAcq[0].act_value_filtered;
                dataAcq[1].data[idx_data] = dataAcq[1].act_value_filtered;
                idx_data++;
                hallS_flag = false;
                // signal other tasks that dataAcq is active
                xEventGroupClearBits(dataAcq_event_group, BIT_DATA_ACQ_IS_IDLE);
                xEventGroupSetBits(dataAcq_event_group, BIT_DATA_ACQ_IS_ACTIVE);
                // loom.machine_running = true;
                step_dataAcq = DA_ACQUIRING;
            }
            break;
        //
        case DA_ACQUIRING:
            // check if HallS detects next cycle
            if (hallS_flag == true)
            {
                hallS_flag = false;
                // calculate mean out of sample
                dataAcq_calc_mean(&dataAcq[0], fromDegree, toDegree, idx_data);
                dataAcq_calc_mean(&dataAcq[1], fromDegree, toDegree, idx_data);
                // save data and idx only if full measurement is done
                dataAcq[0].idx = idx_data;
                dataAcq[1].idx = idx_data;
                // copy data to array to be sent via wifi
                for (int nn = 0; nn < idx_data; nn++)
                {
                    dataAcq[0].valid_data[nn] = dataAcq[0].data[nn];
                    dataAcq[1].valid_data[nn] = dataAcq[1].data[nn];
                }
                // get current channel select (meaning the valid channel at 0°, equals the channel of last cycle)
                // this is in fact the channel corresponding to data in dataAcq
                dataAcq[0].valid_channel = hdrive_get_channel_select();
                // signal other tasks (for exampl mean_control_task)
                xEventGroupSetBits(hdrive_event_group_array[hdrive_get_channel_select()], BIT_VALID_MEASUREMENT);
                // get relevant data to post, with timestamp. Only change value1/2 of valid channel
                if (!dataAcq[0].valid_channel)
                {
                    dataToPost.value1 = dataAcq[0].mean;
                }
                else
                {
                    dataToPost.value2 = dataAcq[1].mean;
                }
                dataToPost.value3 = hdrive_get_currentSetpointClosedPercent(0);
                dataToPost.value4 = hdrive_get_currentSetpointClosedPercent(1);
                esp_time_get_current_timestamp(dataToPost.timestamp);
                // post data only every x cycle
                if (postDataCounter >= postDataCycle)
                {
                    postDataCounter = 0;
                    sd_card_post_on_queue(dataToPost);
                }
                else
                {
                    postDataCounter++;
                }
                //
                // ============
                // ESP_LOGI(TAG, "samples: %i == counter: %i\n", idx_data, xTaskGetTickCount());
                // ============
                // first data Point:
                idx_data = 0;
                dataAcq[0].data[idx_data] = dataAcq[0].act_value_filtered;
                dataAcq[1].data[idx_data] = dataAcq[1].act_value_filtered;
                idx_data++;
                // if not next cycle, check if data is not full, i.e. if machine is in fact running at expected speed
                // if ok, contnue to fill data array
            }
            else if (idx_data < DATA_ACQ_MAX_SAMPLES_PER_CYCLE)
            {
                dataAcq[0].data[idx_data] = dataAcq[0].act_value_filtered;
                dataAcq[1].data[idx_data] = dataAcq[1].act_value_filtered;
                idx_data++;
                // if to much data, machine has stopped running (or is to slow!)
                // in this case, go back to idle
            }
            else
            {
                // ============
                ESP_LOGW(TAG, "==== HallSignal Timeout. Going back to IDLE... ====");
                // ============
                // signal other tasks that dataAcq is idle
                xEventGroupClearBits(dataAcq_event_group, BIT_DATA_ACQ_IS_ACTIVE);
                xEventGroupSetBits(dataAcq_event_group, BIT_DATA_ACQ_IS_IDLE);
                // loom.machine_running = false;
                step_dataAcq = DA_IDLE;
            }
            break;
        //
        case DA_ERROR:
            // ...
            break;
        //
        default:
            // ...
            break;
        }

        // ==========================================================================================
        // print stuff every x cycles, i.e. every x ms
        if (cycleCount >= 1000)
        {
            // testing data saving
            // testdata1.value1 = 11.1;
            // testdata1.value2 = 22.2;
            // testdata1.value3 = 33.3;
            // testdata1.value4 = 44.4;
            // esp_time_get_current_timestamp(testdata1.timestamp);
            // // if queue is full, reset counter and wait 3 seconds until trying to post again
            // if (fullQueueCounter < 100 * 3)
            // {
            //     fullQueueCounter++;
            // }
            // else
            // {
            //     if ((sd_card_post_on_queue(testdata1) >= 2))
            //     {
            //         ESP_LOGE(TAG,"Failed to Post on Queue, waiting for 3 Seconds until next post");
            //         fullQueueCounter = 0;
            //     }
            // }

            // printf("xTaskGetTickCount: %i\n",xTaskGetTickCount());
            // printf("Tension RAW: %.2f = Tension FILTERED: %.2f \n", dataAcq[0].act_value, dataAcq[0].act_value_filtered);

            // IB printfs ===============
            // printf("HallS: %i, BTSR1: %i, BTSR2: %i\n", gpio_get_level(GPIO_HALLS_INPUT),adc1_get_raw(ADC1_BTSR1_INPUT),adc1_get_raw(ADC1_BTSR2_INPUT));
            // printf("mean 1 %.2f. mean 2 %.2f\n", dataAcq[0].mean, dataAcq[1].mean);
            // printf("BTSR1: %i = BTSR2: %i = ENC: %i\n", adc1_get_raw(ADC1_BTSR1_INPUT),adc1_get_raw(ADC1_BTSR2_INPUT),adc1_get_raw(ADC1_ENC_INPUT) );
            // ==========================

            // printf("samples[0]: %i - step: %i\n", dataAcq[0].idx,step_dataAcq);

            // printf("[data] -%i- idx: %i\n",difference_dataAcq,dataAcq[0].idx);

            // printf("ADC RAW: %i\n", adc1_get_raw(ADC1_BTSR1_INPUT));

            // ESP_LOGE(TAG,"Min: %i === Max: %i === angle: %f === \n",enc_min,enc_max,loom.actualAngle);

            // ESP_LOGE(TAG,"Angle: %.1f === Offset: %i\n",loom.actualAngle, loom.angleOffset);
            // ESP_LOGE(TAG,"Min: %i === Max: %i\n",loom.enc_min, loom.enc_max);

            // printf("%llu\n",difference_dataAcq_array[0]);
            // printf("%llu\n",difference_dataAcq_array[1]);
            // printf("%llu\n",difference_dataAcq_array[2]);
            // printf("%llu\n",difference_dataAcq_array[3]);
            //  printf("BLABLA\n");

            // printf("%i;%.2f;%.2f\n",adc1_get_raw(ADC1_BTSR1_INPUT),dataAcq[0].act_value,dataAcq[0].act_value_filtered);

            // printf("wifi: %i\n",uxTaskGetStackHighWaterMark(fb_wifi_task_handle));
            //  printf("hdrive: %i\n",uxTaskGetStackHighWaterMark(hdrive_task_handle));
            //  printf("dataAcq: %i\n",uxTaskGetStackHighWaterMark(dataAcq_task_handle));
            //  printf("mean: %i\n",uxTaskGetStackHighWaterMark(mean_control_task_handle));
            //  printf("nvs: %i\n",uxTaskGetStackHighWaterMark(nvs_task_handle));
            //  ESP_LOGE(TAG, "=== 1s? ===");

            // printf("angle: %.2f° = vel: %.2f°/s = running: %d\n", dt1_angular_vel.input, dt1_angular_vel.output, loom.machine_running);

            cycleCount = 0;
        }
        // ====================================================
        // delay until next cycle, increment cycle counter
        cycleCount++;
        vTaskDelayUntil(&previousWakeTime0, (configTICK_RATE_HZ / cycleFrequency));
        // ====================================================
    }
}
