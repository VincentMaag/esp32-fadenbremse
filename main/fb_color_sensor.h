/*
    ...

*/
#ifndef __FB_COLOR_SENSOR__
#define __FB_COLOR_SENSOR__

void color_sensor_task(void *arg);



void color_sensor_init();
void color_sensor_trigger(int channel);

#define COLOR_SENSOR_DETECTED_LEVEL     (0)     // logic level that projectile loom expects for EMPTY spool
#define COLOR_SENSOR_NOT_DETECTED_LEVEL (1)     // logic level that projectile loom expects for FULL spool



#endif /* __FB_COLOR_SENSOR__ */