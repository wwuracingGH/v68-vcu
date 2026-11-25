#ifndef _MODULES_LIMITER_H_
#define _MODULES_LIMITER_H_

#include <stdint.h>
#include "dynamics.h"

int16_t LIMIT_maxTorque(
    VehicleDynamicParams_t* params,    
    uint16_t bms_discharge_amps,
    float bms_bus_voltage,
    float velocity,
    float acceleration_y
);

int16_t LIMIT_regenTorque(
    VehicleDynamicParams_t* params,    
    uint16_t bms_charge_amps,
    float bms_bus_voltage,
    float velocity
);

int16_t LIMIT_getLastBMSTorqueLimit();
int16_t LIMIT_getLastSRTorqueLimit();
int16_t LIMIT_getLastKWTorqueLimit();
int16_t LIMIT_getLastRegenTorqueLimit();

/* should be called before torque request is gotten */
void LIMIT_updateSlipController();

#endif
