/*
    ...

*/
#ifndef __FB_BLINKER__
#define __FB_BLINKER__

// Blinker Modes
// #define BLINKER_OFF (0)
#define BLINKER_FAST (1)
#define BLINKER_MEDIUM (2)
#define BLINKER_SLOW (3)
#define BLINKER_FAST_DOUBLE (4)
#define BLINKER_MEDIUM_DOUBLE (5)
#define BLINKER_SLOW_DOUBLE (6)
// #define BLINKER_BURST (7)
// 


void blinker_task(void *arg);

void blinker_init();
void blinker_start();
void blinker_stop();
void blinker_set_mode(int mode);
void blinker_burst(int burstTime);


#endif /* __FB_BLINKER__ */