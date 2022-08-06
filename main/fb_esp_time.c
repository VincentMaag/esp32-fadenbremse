/* 


*/

#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "fb_esp_time.h"

static const char *TAG = "fb_esp_time";

// ======================================================================
// variables global to time
// struct esp_time_t esp_time;
struct fb_time_t fb_time;
time_t currentOffset;                // current Offset value for time calculation
time_t currentTime;                  // current estimated time as time_t
time_t currentOffsetCorrectionValue; // current offset error as told to us by external time source as time_t
//
// ======================================================================
// estimate "real" time out of esp-time & passed offset. Pass back current
// time, which is also the next needed Offset if esp boots again
void esp_time_estimate(void)
{
    // get esp time
    time(&fb_time.esp.as_time_t);
    // refactor to tm
    fb_time.esp.as_tm = *gmtime(&fb_time.esp.as_time_t);
    // refactor to string
    strcpy(fb_time.esp.as_string, asctime(&fb_time.esp.as_tm));
    // log
    // ESP_LOGI(TAG, "ESP, time_t: %ld", fb_time.esp.as_time_t);
    // ESP_LOGI(TAG, "ESP, tm.year: %i = tm.second: %i", fb_time.esp.as_tm.tm_year, fb_time.esp.as_tm.tm_sec);
    // ESP_LOGI(TAG, "ESP, string: %s", fb_time.esp.as_string);
    //
    // set offset from current global offset
    fb_time.offset.as_time_t = currentOffset;
    // refactor to tm
    fb_time.offset.as_tm = *gmtime(&fb_time.offset.as_time_t);
    // refactor to string
    strcpy(fb_time.offset.as_string, asctime(&fb_time.offset.as_tm));
    // log
    // ESP_LOGI(TAG, "OFFSET, time_t: %ld", fb_time.offset.as_time_t);
    // ESP_LOGI(TAG, "OFFSET, tm.year: %i = tm.second: %i", fb_time.offset.as_tm.tm_year, fb_time.offset.as_tm.tm_sec);
    // ESP_LOGI(TAG, "OFFSET, string: %s", fb_time.offset.as_string);
    //
    // estimate current time
    fb_time.estimated.as_time_t = fb_time.esp.as_time_t + fb_time.offset.as_time_t;
    // refactor to tm
    fb_time.estimated.as_tm = *gmtime(&fb_time.estimated.as_time_t);
    // refactor to string
    strcpy(fb_time.estimated.as_string, asctime(&fb_time.estimated.as_tm));
    // log
    // ESP_LOGI(TAG, "ESTIMATED, time_t: %ld", fb_time.estimated.as_time_t);
    // ESP_LOGI(TAG, "ESTIMATED, tm.year: %i = tm.second: %i", fb_time.estimated.as_tm.tm_year, fb_time.estimated.as_tm.tm_sec);
    // ESP_LOGI(TAG, "ESTIMATED, string: %s", fb_time.estimated.as_string);
    //
    // ESP_LOGI(TAG, "The estimated Date and Time is: %s", fb_time.estimated.as_string);
    // set current time (time_t) as value for next needed Offset at next esp boot
    currentTime = fb_time.estimated.as_time_t;
}
// ======================================================================
// calculate and return an offset value using year, month, day, hour, minute, second
time_t esp_time_calculate_offset(int year, int month, int day, int hour, int minute, int second)
{
    // temp tm & time_t object
    struct tm temp_tm;
    time_t temp_time_t;
    // copy values into global object
    temp_tm.tm_year = year - 1900;
    temp_tm.tm_mon = month - 1;
    temp_tm.tm_mday = day;
    temp_tm.tm_hour = hour;
    temp_tm.tm_min = minute;
    temp_tm.tm_sec = second;
    // refactor to time_t
    temp_time_t = mktime(&temp_tm);
    // log
    ESP_LOGI(TAG, "New offset calculated, time_t: %ld", temp_time_t);
    // return offset
    return temp_time_t;
}
// ======================================================================
// set offset from anywhere in any task
void esp_time_set_offset(time_t offset)
{
    currentOffset = offset;
}
// ======================================================================
// initialize offset from anywhere in any task
void esp_time_init_offset(time_t offset)
{
    currentOffset = offset;
    currentTime = offset;
    currentOffsetCorrectionValue = 0;
}
// ======================================================================
// get offset needed by esp at next boot
time_t esp_time_get_next_needed_offset(void)
{
    return currentTime;
}
// ======================================================================
// get current offset
time_t esp_time_get_current_offset(void)
{
    return currentOffset;
}
// ======================================================================
// calculate current offset value error through external time source
void esp_time_synch_offset_error(time_t absoluteTime)
{
    // ESP_LOGE(TAG, "before synching: currentOffset: %ld, absoluteTime: %ld, currentTime: %ld, ",currentOffset,absoluteTime,currentTime);
    currentOffsetCorrectionValue = (absoluteTime - currentTime);
    // request to synchronize time
    // requestToSynchTime = true;
    // ESP_LOGE(TAG, "after synching: currentOffset: %ld, absoluteTime: %ld, currentTime: %ld, ",currentOffset,absoluteTime,currentTime);
}
// ======================================================================
// synchronizes current offset value with help of current correction value
void esp_time_synch_offset(void)
{
    // ESP_LOGE(TAG, "before synching: currentOffset: %ld, absoluteTime: %ld, currentTime: %ld, ",currentOffset,absoluteTime,currentTime);
    currentOffset = currentOffset + currentOffsetCorrectionValue;
    // make sure to reset correction value back to 0 after synchronizing time
    currentOffsetCorrectionValue = 0;
    // ESP_LOGE(TAG, "after synching: currentOffset: %ld, absoluteTime: %ld, currentTime: %ld, ",currentOffset,absoluteTime,currentTime);
}
// ======================================================================
// get timestamp of current time. Maximum size of Timestamp is 19 characters
//  + null terminator = 20
void esp_time_get_current_timestamp(char *dest)
{
    // temp variables on heap
    int year, month, day, hour, minute, second;
    // copy global variables. Not thread safe of course
    year = fb_time.estimated.as_tm.tm_year + 1900;
    month = fb_time.estimated.as_tm.tm_mon + 1;
    day = fb_time.estimated.as_tm.tm_mday;
    hour = fb_time.estimated.as_tm.tm_hour;
    minute = fb_time.estimated.as_tm.tm_min;
    second = fb_time.estimated.as_tm.tm_sec;
    // copy to destination memory
    sprintf(dest, "%d.%d.%d;%d:%d:%d", day,month,year,hour,minute,second);

}
// ======================================================================
// esp_time task
void esp_time_task(void)
{
    // =====================================================
    // local variables
    // variable to store last tick time for exakt cycle time. Updates every cycle
    TickType_t previousWakeTime = xTaskGetTickCount();
    TickType_t cycleFrequency = 1;                    // actual cycle frequency [Hz]
    float cycleTime = 1000.0 / (float)cycleFrequency; // actual cycle time [ms] of loop
    int cycleCount = 0;                               // count hdrive swicth cycles

    // time_t t1;
    // time_t t2;
    // time_t t3;
    // time_t t4;

    // char teststring[20];

    while (1)
    {
        // estimate current time with current offset and update next needed offset
        esp_time_synch_offset();
        esp_time_estimate();
        // ============================================
        if (cycleCount >= 10)
        {
            ESP_LOGI(TAG, "HEAP min free: %d == free: %d",esp_get_minimum_free_heap_size(),esp_get_free_heap_size());
            ESP_LOGI(TAG, "Estimated Date and Time: %s", fb_time.estimated.as_string);
            
            // t1 = esp_time_calculate_offset(2021, 8, 11, 0, 0, 0);
            // t2 = esp_time_calculate_offset(2021, 8, 11, 0, 1, 0);
            // t3 = esp_time_calculate_offset(2021, 8, 11, 1, 0, 0);
            // t4 = esp_time_calculate_offset(2021, 8, 12, 0, 0, 0);

            // ESP_LOGE(TAG, "t1: %ld = t2: %ld = t3: %ld = t4: %ld = ", t1,t2,t3,t4);

            // esp_time_get_current_timestamp(teststring);
            // ESP_LOGE(TAG, "timestamp: %s", teststring);

            cycleCount = 0;
        }
        // ====================================================
        // delay until next cycle, increment cycle counter
        cycleCount++;
        vTaskDelayUntil(&previousWakeTime, (configTICK_RATE_HZ / cycleFrequency));
    }
}
