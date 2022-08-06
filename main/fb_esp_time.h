/*
    ...

*/
#ifndef __FB_ESP_TIME__
#define __FB_ESP_TIME__

#include "time.h"
//#include "sys/time.h"

struct esp_time_t{
    time_t as_time_t;       // time as value time_t (ms)
    struct tm as_tm;        // time as tm (struct year, month, day, seconds etc.)
    char as_string[32];     // date & time as string
};            

struct fb_time_t{
    struct esp_time_t esp;                   // time from esp
    struct esp_time_t offset;                // offset value from external source
    struct esp_time_t estimated;             // estimated value
};

void esp_time_estimate(void);
time_t esp_time_calculate_offset(int year, int month, int day, int hour, int minute, int second);
void esp_time_set_offset(time_t offset);
time_t esp_time_get_next_needed_offset(void);
void esp_time_synch_offset_error(time_t absoluteTime);
time_t esp_time_get_current_offset(void);
void esp_time_init_offset(time_t offset);
void esp_time_get_current_timestamp(char *dest);
void esp_time_synch_offset(void);

// main Task
void esp_time_task(void);



#endif /* __FB_ESP_TIME__ */