#ifndef _FLASH_SETTINGS_H_
#define _FLASH_SETTINGS_H_

#define DEFAULT_APPS1_MIN    2247 
#define DEFAULT_APPS1_MAX    1834 
#define DEFAULT_APPS2_MIN    1710 
#define DEFAULT_APPS2_MAX    2123 
#define DEFAULT_APPS3_MIN    2282
#define DEFAULT_APPS3_MAX    2697 
#define DEFAULT_APPS4_MIN    1685
#define DEFAULT_APPS4_MAX    1270
#define DEFAULT_BPS_MIN       372
#define DEFAULT_BPS_MAX      3348 
#define DEFAULT_BPS_RATIO   35000
	
#define DEFAULT_MAX_TORQUE   1000 
#define DEFAULT_HARD_BRAKING  100 
#define DEFAULT_MAX_BRAKING  3000

#define DEFAULT_TORQUE_CURVE  {0, 16, 32, 48, 64, 80, 96, 112, 128, 144, 160, 176, 192, 208, 224, 240}

/* DYNAMIC PARAMS */
#define DEFAULT_GRAVITY         9.806f

#define DEFAULT_VEHICLE_MASS    280.0f
#define DEFAULT_TIRE_RADIUS     0.2f
#define DEFAULT_DRIVE_RATIO     3.0f
#define DEFAULT_MAX_SPEED       45.3785f

#define DEFAULT_SLIP_TARGET     0.1f
#define DEFAULT_MAX_AX          13.7284f    /* gravity * 1.4 */
#define DEFAULT_MAX_AY          13.2381f    /* gravity * 1.4 */
#define DEFAULT_RG_RAMP_STRT    1.67f       /* a bit more than 5 km/h as per the rules */
#define DEFAULT_RG_RAMP_END     2.5f        /* like 9km/h */
#define DEFAULT_MAX_POWER       79950.0f    /* slightly less than 80kw */

#define DEFAULT_SLIP_PID_KP     1.0f        /* idk */
#define DEFAULT_SLIP_PID_KI     1.0f        /* idk */

#define DEFAULT_CM200_EFFICIENCY { 0.94f, 0.94f, 0.94f, 0.94f, 0.94f, 0.94f, 0.94f, 0.94f, 0.94f, 0.94f, 0.94f, 0.94f, 0.94f, 0.94f, 0.94f, 0.94f, 0.94f, 0.94f, 0.94f, 0.94f, 0.94f, 0.94f, 0.94f, 0.94f, 0.94f, 0.94f, 0.94f, 0.94f, 0.94f, 0.94f, 0.94f, 0.94f,}
#define DEFAULT_SLCTRL_ENABLED  1
#define DEFAULT_KWCTRL_ENABLED  1

#endif