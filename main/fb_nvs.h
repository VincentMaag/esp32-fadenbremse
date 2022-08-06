/*
    ...

*/

#ifndef __FB_NVS_H__
#define __FB_NVS_H__

// initialize nvs data
void init_nvs_data();
// task to store nvs data 
void nvs_task(void* arg);

#endif /* __FB_NVS_H__ */