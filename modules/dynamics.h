/* 
 * Nicole Swierstra
 * ======= Vehicle Dynamics Modeling ======== 
 * Coming soon, maybe 
 */

#ifndef _MODULES_DYNAMICS_H_
#define _MODULES_DYNAMICS_H_

#include "control.h"

#define EFFICIENCY_LUT_LEN 32

/* TODO */
typedef struct {
    float g;

    float vehicle_mass;
    float tire_radius;
    float drive_ratio;
    float max_speed;

    float slip_ratio_target;
    float max_ay;
    float max_ax;
    float regen_speed_ramp_start;
    float regen_speed_ramp_end;
    float max_power;

    float torque_pid_kp;
    float torque_pid_kd;
    float torque_pid_ki;

    /* map from rpm / (6500 / 32) -> efficency */
    float cm200efficiency[EFFICIENCY_LUT_LEN]; 
    
    uint8_t slip_ratio_limit_enabled; 
    uint8_t kw_limit_enabled;
} VehicleDynamicParams_t;

/* TODO */
typedef struct {
    float Vx;
    float a_x;
    float a_y;
    float omega_z;
} VehicleDynamicState_t;

VehicleDynamicState_t* updatedModel(uint32_t newTime, ControlReq_t* newControl);

#endif