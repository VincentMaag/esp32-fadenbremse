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
#include "fb_blinker.h"

// tag
static const char *TAG = "fb_blinker";

// variables for blinker
int blinkingFrequency = 1; // blinking frequency in Hz
int blinkerCounter = 0;    // counter for blinking states
uint8_t blinkingStyle = 0; // 0 = single, 1 = double
int blinkerBurstTime = 1;  // burst time

// define digital Outputs
#define GPIO_LED_OUTPUT GPIO_NUM_12 // Boot fail if pulled high
#define GPIO_OUTPUT_PIN_SEL ((1ULL << GPIO_LED_OUTPUT))

// typedef enum for state machine
typedef enum
{
    BL_OFF,
    BL_ON,
    BL_BURST
} blinker_step_enum_t;
blinker_step_enum_t blinkerStep = BL_OFF;

// =====================================================================
// initialize blinker (gpio etc.)
// =====================================================================
void blinker_init()
{
    // configure DigOut(s) and initialize "low"
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    gpio_set_level(GPIO_LED_OUTPUT, 1);
    // init blinker params
    blinker_start();
    blinker_set_mode(BLINKER_SLOW);
}
// =====================================================================
// turn on blinker
// 
void blinker_start()
{
    blinkerStep = BL_ON;
    blinkerCounter = 0;
}
// =====================================================================
// turn off blinker
// 
void blinker_stop()
{
    blinkerStep = BL_OFF;
    blinkerCounter = 0;
}
// =====================================================================
// set short burst of blinker
// burstTime in [s]
void blinker_burst(int burstTime)
{
    blinkerStep = BL_BURST;
    blinkerCounter = 0;
    blinkerBurstTime = burstTime;
}
// =====================================================================
// set blinker mode
// 
void blinker_set_mode(int mode)
{
    switch (mode)
    {
    case BLINKER_FAST:
        blinkingFrequency = 10;
        blinkingStyle = 0;
        break;
    case BLINKER_MEDIUM:
        blinkingFrequency = 5;
        blinkingStyle = 0;
        break;
    case BLINKER_SLOW:
        blinkingFrequency = 1;
        blinkingStyle = 0;
        break;
    case BLINKER_FAST_DOUBLE:
        blinkingFrequency = 10;
        blinkingStyle = 1;
        break;
    case BLINKER_MEDIUM_DOUBLE:
        blinkingFrequency = 5;
        blinkingStyle = 1;
        break;
    case BLINKER_SLOW_DOUBLE:
        blinkingFrequency = 1;
        blinkingStyle = 1;
        break;
    default:
        break;
    }
    // reset counter
    blinkerCounter = 0;
}
// =====================================================================
// handle blinking in single blink mode
// Counter = memory pointer, FreqTicks = blinking frequency in Ticks
void blink_in_single_mode(int *Counter, int PeriodTicks)
{
    // single blinking mode
    if (*Counter < PeriodTicks / 2)
    {
        gpio_set_level(GPIO_LED_OUTPUT, 1);
        (*Counter)++;
    }
    else if (*Counter < PeriodTicks)
    {
        gpio_set_level(GPIO_LED_OUTPUT, 0);
        (*Counter)++;
    }
    else
    {
        *Counter = 0;
    }
}
// =====================================================================
// handle blinking in double blink mode
// Counter = memory pointer, FreqTicks = blinking frequency in Ticks
void blink_in_double_mode(int *Counter, int PeriodTicks)
{
    // increment counter, reset if we reached four Periods
    if (*Counter < 4 * PeriodTicks)
    {
        (*Counter)++;
    }
    else
    {
        *Counter = 0;
    }

    if (*Counter < PeriodTicks / 2)
    {
        gpio_set_level(GPIO_LED_OUTPUT, 1);
    }
    else if (*Counter < PeriodTicks)
    {
        gpio_set_level(GPIO_LED_OUTPUT, 0);
    }
    else if (*Counter < (PeriodTicks * 3) / 2)
    {
        gpio_set_level(GPIO_LED_OUTPUT, 1);
    }
    else if (*Counter < PeriodTicks * 4)
    {
        gpio_set_level(GPIO_LED_OUTPUT, 0);
    }
}

// =====================================================================
// blinker task
// =====================================================================
void blinker_task(void *arg)
{
    // general parameters for cycle
    TickType_t previousWakeTime0 = xTaskGetTickCount();
    TickType_t cycleFrequency = 40;                   // actual cycle frequency [Hz]. Must be 2*mimimum wanted blinking freq
    float cycleTime = 1000.0 / (float)cycleFrequency; // actual cycle time [ms] off dataAcq loop
    int cycleCount = 0;                               // count switch cycles
    // local variables
    int blinkerPeriodTicks = cycleFrequency / blinkingFrequency; // continuous blinking time in ticks
    int blinkerPeriodTicksBurst = cycleFrequency / 20;           // burst blinking time in ticks
    int burstCounter = 0;                                        // count cycles for temporary burst
    // initialize
    blinker_init();

    // =================================================================
    while (1)
    {
        // update Ticks calculated from cycleFreq and blinkingFreq
        blinkerPeriodTicks = cycleFrequency / blinkingFrequency;
        // implement blibker:
        switch (blinkerStep)
        {
        case BL_OFF:
            // turn blinker output off
            gpio_set_level(GPIO_LED_OUTPUT, 0);
            break;

        case BL_ON:
            // blink continuously
            if (blinkingStyle == 0)
            {
                // single blinking mode
                blink_in_single_mode(&blinkerCounter, blinkerPeriodTicks);
            }
            // blink in double mode
            else if (blinkingStyle == 1)
            {
                blink_in_double_mode(&blinkerCounter, blinkerPeriodTicks);
            }

            break;

        case BL_BURST:
            // single blink for short time, then go back to continuous blinking with unchanged settings
            if (burstCounter < (blinkerBurstTime * (float)cycleFrequency))
            {
                blink_in_single_mode(&blinkerCounter, blinkerPeriodTicksBurst);
                burstCounter++;
            }
            else
            {
                burstCounter = 0;
                blinkerStep = BL_ON;
            }

            break;

        default:
            break;
        }

        // ==========================================================================================
        // print stuff every second
        if (cycleCount >= cycleFrequency)
        {

            //printf("blinkingFrequency: %i == blinkingFreqTicks: %i == \n", blinkingFrequency, blinkerFreqTicks);
            //ESP_LOGE(TAG, "=== 1s? ===");
            cycleCount = 0;
        }
        // ====================================================
        // delay until next cycle, increment cycle counter
        cycleCount++;
        vTaskDelayUntil(&previousWakeTime0, (configTICK_RATE_HZ / cycleFrequency));
        // ====================================================
    }
}
