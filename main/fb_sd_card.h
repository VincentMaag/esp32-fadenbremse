/*
    ...

*/
#ifndef __FB_SD_CARD__
#define __FB_SD_CARD__

typedef struct {
    float value1;           // log value 1
    float value2;           // log value 2
    float value3;           // log value 3
    float value4;           // log value 4
    char timestamp[20];     // timestamp as string
}data_to_store_t;


int sd_card_post_on_queue(data_to_store_t newData);

void sd_card_init(void);
void sd_card_task(void);



#endif /* __FB_BLINKER__ */