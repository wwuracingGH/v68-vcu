#include "limiter.h"

int16_t last_kw_limit = 0;
int16_t last_sl_limit = 0;
int16_t last_bms_limit = 0;
int16_t last_bms_regenlimit = 0;

int16_t LIMIT_getLastBMSTorqueLimit()   { return last_bms_limit; }
int16_t LIMIT_getLastSRTorqueLimit()    { return last_sl_limit; }
int16_t LIMIT_getLastKWTorqueLimit()    { return last_kw_limit; }
int16_t LIMIT_getLastRegenTorqueLimit() { return last_bms_regenlimit; }

float slip_torque_estimator = 0;

/* TODO: make these global parameters */
float cm200efficiencyatspeed(VehicleDynamicParams_t* params, float speed) {
    float speed_val = (speed / (params->max_speed)) * EFFICIENCY_LUT_LEN; 
    int max_idx = (int)speed_val;
    float lerpval = speed_val - max_idx;
    
    float min_val = 1.0;
    float max_val = params->cm200efficiency[max_idx];
    if (max_idx != 0) {
        min_val = params->cm200efficiency[max_idx - 1];
    }

    return lerpval * max_val + (1.0 - lerpval) * min_val;
}

float get_accel_y_multiplier(VehicleDynamicParams_t* params, float accel_y) {
    return __builtin_sqrtf(1.0 - accel_y / params->max_ay);
}

float saturate_regen_cutoff_speed(VehicleDynamicParams_t* params, float speed) {
    if (speed < params->regen_speed_ramp_start) return 0.0;
    if (speed > params->regen_speed_ramp_end) return 1.0;

    return (speed - params->regen_speed_ramp_start) / (params->regen_speed_ramp_end - params->regen_speed_ramp_start);
}

int16_t LIMIT_maxTorque(VehicleDynamicParams_t* params, uint16_t max_amps, float bus_voltage, float v_x, float a_y) {
    float max_power = bus_voltage * max_amps * cm200efficiencyatspeed(params, v_x);
    float motor_rps = v_x / params->tire_radius * params->drive_ratio;
    
    int16_t max_bms_torque = (int)((max_power / motor_rps) * 10.0f);

    int16_t max_80kw_torque = (int16_t)(cm200efficiencyatspeed(params, v_x) * (params->max_power / motor_rps));

    /* TODO: implement */
    int16_t max_slip_ratio_torque = (int16_t)(get_accel_y_multiplier(params, a_y) * slip_torque_estimator);

    int16_t max_torque = max_bms_torque; /* random big number */

    last_kw_limit = max_80kw_torque;
    last_sl_limit = max_slip_ratio_torque;
    last_bms_limit = max_bms_torque;

    if (max_torque > max_80kw_torque && params->kw_limit_enabled) 
        max_torque = max_80kw_torque;

    if (max_torque > max_slip_ratio_torque && params->slip_ratio_limit_enabled) 
        max_torque = max_slip_ratio_torque;

    return max_torque;
}

int16_t LIMIT_regenTorque(VehicleDynamicParams_t* params, uint16_t max_amps, float bus_voltage, float v_x) {
    float max_power = bus_voltage * max_amps * cm200efficiencyatspeed(params, v_x);
    float motor_rps = v_x / params->tire_radius * params->drive_ratio;

    int16_t min_torque = -(int16_t)((max_power / motor_rps) * saturate_regen_cutoff_speed(params, v_x) * 10);

    last_bms_regenlimit = min_torque;

    return min_torque;
}

void LIMIT_updateSlipController(VehicleDynamicParams_t* params, int16_t lastTorqueRequest) {
    static float error_integrator = 0.0f;
    const float dt = 0.001f; /* this should not be a constant float in here */
    const float decay = 0.5f * dt; /* TODO: should not be a constant */

    /* check if slip was the thing limiting torque last time or if it's close enough */
    if ((last_sl_limit - lastTorqueRequest) > 5) {
        /* this ideal torque should probably be pre-calculated */
        float ideal_torque_estimator = params->drive_ratio * params->max_ax * params->vehicle_mass / params->drive_ratio;
        
        /* if slip wasn't the thing limiting torque, very very slowly fade the torque request to the initial target with an IIR filter */
        slip_torque_estimator = slip_torque_estimator * decay + ideal_torque_estimator * (1.0 - decay);
        error_integrator *= (1.0 - decay); /* error also decays, is this a good thing ? */
    } else {
        float error = 0.0f; /* TODO: get error */
        error_integrator += error * dt;

        slip_torque_estimator += (params->torque_pid_kp * error + params->torque_pid_ki * error_integrator) * dt;
    }
}
