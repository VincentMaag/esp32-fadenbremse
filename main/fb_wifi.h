/*
    ...

*/
#ifndef __FB_WIFI_H__
#define __FB_WIFI_H__
// these are here so that websocket_callback knows it's types...
#include "websocket.h"
#include "websocket_server.h"
//
void wifi_init_sta();
void wifi_init_ap();
//
void wifi_try_connect_sta();
//
static void http_serve(struct netconn *conn);
static void server_task(void* pvParameters);
static void server_handle_task(void* pvParameters);
//
void websocket_callback(uint8_t num,WEBSOCKET_TYPE_t type,char* msg,uint64_t len);
//
void wifi_init_gpio();
//
void wifi_task(void* arg);
//
#endif /* __FB_WIFI_H__ */