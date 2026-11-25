/* WHEELSPEEDS */
#ifndef _MODULES_WHEELSPEED_H_
#define _MODULES_WHEELSPEED_H_

#include <stdint.h>

typedef struct
{
    float last_velocity;
    float d_velocity;
    float last_dtime;
    int32_t last_tick;
} WS_data_t;

typedef enum {
    WS_FRONT_LEFT = 0x0,
    WS_FRONT_RIGHT = 0x1,
    WS_REAR_LEFT = 0x2,
    WS_REAR_RIGHT = 0x3
} WheelID;

void WS_Init();
float WS_GetVelocity(WheelID id);

int16_t* WS_GetCANMSG();

/* Estimator velocity for a 2WD car, so it returns the average of the velocity of the two front wheels */
float WS_GetEstimatorVelocity();
/* returns the confidence from 0 to 1. MUST BE CALLED AFTER WS_GetEstimatorVelocity() */
float WS_GetEstimatorConfidence();

#endif