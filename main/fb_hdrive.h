/*
    ...

*/
#ifndef __FB_HDRIVE_H__
#define __FB_HDRIVE_H__


void init_gpio_hdrive();
void hdrive_task(void* arg);
int hdrive_degree_2_duty(float degree);
void hdrive_set_position(int hdrive_channel, float hdrive_setAngle);
static void IRAM_ATTR gpio_isr_handler_tickTimeBetweenEdges(void* hdrive_ch);
float hdrive_get_position(int hdrive_channel);
float percent_2_angle(float percent);
float angle_2_percent(float angle);
int hdrive_get_channel_select();
float hdrive_get_currentSetpointClosedPercent(int ch);

#endif /* __FB_HDRIVE_H__ */