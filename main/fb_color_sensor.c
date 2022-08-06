/*
    ...

*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
//#include "freertos/queue.h"
#include "freertos/event_groups.h"
//#include "esp_event_loop.h"
#include "driver/gpio.h"
//#include "driver/adc.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_err.h"
//#include "driver/ledc.h"

//#include "driver/ledc.h"
#include "esp_err.h"

#include "driver/periph_ctrl.h"
#include "driver/timer.h"

#include "fb_projdefs.h"
#include "fb_color_sensor.h"

// tag
static const char *TAG = "fb_color_sensor";

// ========================================================================================
// === PARAMETERS =========================================================================
// ========================================================================================

int iColorDetectOutputTime = 3; // [s] Detected Output Level Time until color sensor output switches back to "not detected"

// ========================================================================================

// Global vars (scope color_sensor)
// typedef enum for state machine
typedef enum
{
    CS_DETECTED_SENSOR,       // empty spool detected by Sensor
    CS_DETECTED_MEAN_CONTROL, // empty spool detected by mean control
    CS_NOT_DETECTED           // no empty spool detected
} color_sens_step_enum_t;
color_sens_step_enum_t colorSensStep[2] = {CS_NOT_DETECTED, CS_NOT_DETECTED};

int external_sensor_trigger[2] = {COLOR_SENSOR_NOT_DETECTED_LEVEL, COLOR_SENSOR_NOT_DETECTED_LEVEL};

// define digital Inputs
#define GPIO_COLOR_1_INPUT GPIO_NUM_32
#define GPIO_COLOR_2_INPUT GPIO_NUM_25
#define GPIO_INPUT_PIN_SEL ((1ULL << GPIO_COLOR_1_INPUT) | (1ULL << GPIO_COLOR_2_INPUT))
// define digital Outputs
#define GPIO_COLOR_1_OUTPUT GPIO_NUM_33
#define GPIO_COLOR_2_OUTPUT GPIO_NUM_26
#define GPIO_OUTPUT_PIN_SEL ((1ULL << GPIO_COLOR_1_OUTPUT) | (1ULL << GPIO_COLOR_2_OUTPUT))
int gpio_input_ch_select[2] = {GPIO_COLOR_1_INPUT, GPIO_COLOR_2_INPUT};
int gpio_output_ch_select[2] = {GPIO_COLOR_1_OUTPUT, GPIO_COLOR_2_OUTPUT};

// ========================================================================================
// fuction to initialize (gpio etc.)
// ========================================================================================
void color_sensor_init()
{
    // configure DigIns
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    // configure DigOut(s) and initialize "not detected"
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    // set outputs to not detected
    gpio_set_level(gpio_output_ch_select[0], COLOR_SENSOR_NOT_DETECTED_LEVEL);
    gpio_set_level(gpio_output_ch_select[1], COLOR_SENSOR_NOT_DETECTED_LEVEL);
}

// ========================================================================================
// fuction to trigger color sensor (IMPULSE) to level "empty spool detected"
// ========================================================================================
void color_sensor_trigger(int channel)
{
    // ESP_LOGE(TAG, "color sensor received trigger channel [%i]\n", channel);
    external_sensor_trigger[channel] = COLOR_SENSOR_DETECTED_LEVEL;
}
// =====================================================================
// color sensor task
// =====================================================================
void color_sensor_task(void *arg)
{
    // general parameters for cycle
    TickType_t previousWakeTime0 = xTaskGetTickCount();
    TickType_t cycleFrequency = 2;                    // actual cycle frequency [Hz]
    float pauseTime = 1000.0 / (float)cycleFrequency; // actual cycle time [ms] of loop
    int cycleCount = 0;                               // count switch cycles

    // local variables
    int iDetectedStateCounter[2] = {0, 0};
    int iColorDetectOutputTicks = iColorDetectOutputTime * cycleFrequency;

    int iColorSensorOutput[2] = {COLOR_SENSOR_NOT_DETECTED_LEVEL, COLOR_SENSOR_NOT_DETECTED_LEVEL};
    int iColorSensorOutputImpulse[2] = {COLOR_SENSOR_NOT_DETECTED_LEVEL, COLOR_SENSOR_NOT_DETECTED_LEVEL};
    int iColorSensorInput[2] = {COLOR_SENSOR_NOT_DETECTED_LEVEL, COLOR_SENSOR_NOT_DETECTED_LEVEL};

    int iColorSensorInput2[2] = {COLOR_SENSOR_NOT_DETECTED_LEVEL, COLOR_SENSOR_NOT_DETECTED_LEVEL};


    // initialize
    color_sensor_init();


    
    // =================================================================
    while (1)
    {

        

        // for loop over both channels
        for (int color_ch = 0; color_ch < 2; color_ch++)
        {
            // if any task triggers a sensor impulse:
            if (external_sensor_trigger[color_ch] == COLOR_SENSOR_DETECTED_LEVEL)
            {
                // reset trigger
                external_sensor_trigger[color_ch] = COLOR_SENSOR_NOT_DETECTED_LEVEL;
                // set Sensor Impulse
                iColorSensorOutputImpulse[color_ch] = COLOR_SENSOR_DETECTED_LEVEL;
                // reset counter
                iDetectedStateCounter[color_ch] = 0;
            }
            // if counter is reset, count up until timout is reached
            if (iDetectedStateCounter[color_ch] < iColorDetectOutputTicks)
            {
                iDetectedStateCounter[color_ch]++;
            }
            else
            {
                // if timout reached, reset Sensor impulse
                iColorSensorOutputImpulse[color_ch] = COLOR_SENSOR_NOT_DETECTED_LEVEL;
            }
            // read (dominant) color sensor input
            iColorSensorInput[color_ch] = gpio_get_level(gpio_input_ch_select[color_ch]);
            // set sensor output --> "detect" level of sensor input is dominant
            // so if sensor input is "detect", then it doesnt matter what Impulse level is. 
            // However if sensor input is "not detect", then output can still be "detect" if impulse is set by some task
            if(COLOR_SENSOR_DETECTED_LEVEL == 1){
                // logical OR if "detect" is logic 1
                iColorSensorOutput[color_ch] = iColorSensorInput[color_ch] || iColorSensorOutputImpulse[color_ch];
            }else{
                // logical AND if "detect" is logic 0
                iColorSensorOutput[color_ch] = iColorSensorInput[color_ch] && iColorSensorOutputImpulse[color_ch];
            }
            // set Sensor output
            gpio_set_level(gpio_output_ch_select[color_ch],iColorSensorOutput[color_ch]);
        }



        

        // ==========================================================================================
        // print stuff every second
        if (cycleCount >= cycleFrequency)
        {

            // printf("input : %i == impulse: %i == output: %i == counter: %i\n", iColorSensorInput[0], iColorSensorOutputImpulse[0],iColorSensorOutput[0],iDetectedStateCounter[0]);
            //printf("input : %i == impulse: %i == output: %i\n", iColorSensorInput[1], iColorSensorOutputImpulse[1],iColorSensorOutput[1]);
            
            //printf("free: %i\n", (int)uxTaskGetStackHighWaterMark(color_sensor_task_handle));

            //ESP_LOGE(TAG, "=== 1s? ===");
            cycleCount = 0;
        }
        // ====================================================
        // delay until next cycle, increment cycle counter
        cycleCount++;
        //vTaskDelayUntil(&previousWakeTime0,pauseTime);
        vTaskDelayUntil(&previousWakeTime0, (configTICK_RATE_HZ / cycleFrequency));
        // ====================================================
    }
}
