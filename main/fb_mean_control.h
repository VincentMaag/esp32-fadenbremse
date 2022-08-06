/*
    ...

*/
#ifndef __FB_MEAN_CONTROL_H__
#define __FB_MEAN_CONTROL_H__

void mean_control_filter_displayData(mean_control_t *pMean_control, float coeff_T1, float coeff_T);
void mean_control_filter_mean(mean_control_t *pMean_control, float coeff_T1, float coeff_T);
void mean_control_filter_mean_2(mean_control_t *pMean_control);
void mean_control_filter_setpoint_2(mean_control_t *pMean_control);
void mean_control_controller(mean_control_t *pMean_control, float Kp, float Tn, float saturationHigh, float saturationLow);
void mean_control_task(void* arg);


#endif /* __FB_MEAN_CONTROL_H__ */