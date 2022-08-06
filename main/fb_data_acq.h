/*
    ...

*/
#ifndef __FB_DATA_ACQ_H__
#define __FB_DATA_ACQ_H__

typedef enum
{
    DA_IDLE,
    DA_ACQUIRING,
    DA_ERROR
} dataAcq_step_enum_t;

void dataAcq_init_gpio();
void dataAcq_task(void* arg);
float dataAcq_get_yarn_tension(int adc_channel);
void dataAcq_calc_mean(dataAcq_t *pDataAcq, float fromDeg, float toDeg, int amount_of_samples);
void dataAcq_filter_data(dataAcq_t *pDataAcq);

#include "driver/adc.h"
void dataAcq_calibrate_encoder(adc1_channel_t adc_ch, int16_t *pmin, int16_t *pmax);
void dataAcq_get_machine_angle(adc1_channel_t adc_ch, loom_t *ploom, float *oldtempAngle, float *oldActualAngle);


#endif /* __FB_DATA_ACQ_H__ */