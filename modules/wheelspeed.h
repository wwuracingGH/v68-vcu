/* WHEELSPEEDS */
#ifndef _MODULES_WHEELSPEED_H_
#define _MODULES_WHEELSPEED_H_

#include <stdint.h>

typedef struct {
    float last_velocity;
    float d_velocity;
    float last_dtime;
    int32_t last_tick;
} WS_data_t;

extern struct _ws_data_struct {
    WS_data_t fl;
    WS_data_t fr;
} WS_wheels;

void WS_Init(uint32_t ticks_of_period);
float WS_GetVelocity(WS_data_t* data);

#endif