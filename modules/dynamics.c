#include "dynamics.h"
#include "wheelspeed.h"

VehicleDynamicState_t lastState;
ControlReq_t lastCTRL;
uint32_t lastTime = 0;

#define ACCEL_CONFIDENCE 0.8f

void VD_Init() {
    lastState.a_x = 0.0;
    lastState.a_y = 0.0;
    lastState.v_x = 0.0;
    lastState.omega_z = 0.0;

    lastTime = 0.0;
}

VehicleDynamicState_t* VD_updateModel(uint32_t newTime, ControlReq_t* newControl) {
    /* update model values */

    lastCTRL = *newControl;
    lastTime = newTime;

    return &lastState;
}

void VD_accelerometerUpdate(uint32_t newTime, float ax, float ay, float az) {
    /* update model to the current time before editing anything */
    VD_updateModel(newTime, &lastCTRL);
    (void)ax;
    (void)ay;
    (void)az;

    /* update accelerometer values */
}

void VD_wheelspeedUpdate(uint32_t newTime) {
    /* update model to the current time before editing anything */
    VD_updateModel(newTime, &lastCTRL);

    float velocity = WS_GetEstimatorVelocity();
    float confidence = WS_GetEstimatorConfidence();

    (void)velocity;
    (void)confidence;
    /* update wheelspeed values */
}

void VD_GNSSupdate(uint32_t newTime, float v_x, float dop) {
    /* update model to the current time before editing anything */
    VD_updateModel(newTime, &lastCTRL);

    (void)v_x;
    (void)dop;

    /* update GNSS values */
}
