#include "modules/flash.h"
#include <stdint.h>
#include "vendor/CMSIS/Device/ST/STM32H5/Include/stm32h5xx.h"
#include "modules/logging.h"

#include "../flashsettings.h"
#include "../vendor/v68-vsw-def/outputs/vcu_parampipe.c"

volatile CarParameters_t* stored_values = (CarParameters_t*)0x08070000;

CarParameters_t default_vals = {
	sizeof(CarParameters_t),
	{
		DEFAULT_APPS1_MIN,
		DEFAULT_APPS1_MAX,
		DEFAULT_APPS2_MIN,
		DEFAULT_APPS2_MAX,
		DEFAULT_APPS3_MIN,
		DEFAULT_APPS3_MAX,
		DEFAULT_APPS4_MIN,
		DEFAULT_APPS4_MAX,
		DEFAULT_BPS_MIN,
		DEFAULT_BPS_MAX,
		DEFAULT_BPS_RATIO
	},
    {
		DEFAULT_MAX_TORQUE,
		DEFAULT_HARD_BRAKING,
		DEFAULT_MAX_BRAKING,
		0,
		DEFAULT_TORQUE_CURVE
	},
	{
		DEFAULT_GRAVITY,
		DEFAULT_VEHICLE_MASS,
		DEFAULT_TIRE_RADIUS,
		DEFAULT_DRIVE_RATIO,
		DEFAULT_MAX_SPEED,
		DEFAULT_SLIP_TARGET,
		DEFAULT_MAX_AX,
		DEFAULT_MAX_AY,
		DEFAULT_RG_RAMP_STRT,
		DEFAULT_RG_RAMP_END,
		DEFAULT_MAX_POWER,
		DEFAULT_SLIP_PID_KP,
		DEFAULT_SLIP_PID_KI,
		DEFAULT_CM200_EFFICIENCY,
		DEFAULT_SLCTRL_ENABLED,
		DEFAULT_KWCTRL_ENABLED
	},
};

CarParameters_t ram_values = { sizeof(CarParameters_t), { 0 }, { 0 }, (VehicleDynamicParams_t){ 0 } };
int ram_initialized = 0;
const int use_default = 0;

/* 
 * Unlocks the flash to be able to write to it 
 */
void FLASH_Init() {
  	FLASH->NSKEYR = 0x45670123;
	FLASH->NSKEYR = 0xCDEF89AB;

	ParamPipe_APPS1_L = &ram_values.adc_bounds.APPS1_l;
	ParamPipe_APPS2_L = &ram_values.adc_bounds.APPS2_l;
	ParamPipe_APPS3_L = &ram_values.adc_bounds.APPS3_l;
	ParamPipe_APPS4_L = &ram_values.adc_bounds.APPS4_l;
	ParamPipe_APPS1_H = &ram_values.adc_bounds.APPS1_h;
	ParamPipe_APPS2_H = &ram_values.adc_bounds.APPS2_h;
	ParamPipe_APPS3_H = &ram_values.adc_bounds.APPS3_h;
	ParamPipe_APPS4_H = &ram_values.adc_bounds.APPS4_h;
	ParamPipe_BPS_S_MIN = &ram_values.adc_bounds.BPS_s_min;
	ParamPipe_BPS_S_MAX = &ram_values.adc_bounds.BPS_s_max;
	ParamPipe_BPS_F_BIAS = &ram_values.adc_bounds.BPS_f_bias;
	ParamPipe_MAX_TORQUE = &ram_values.params.max_torque;
	ParamPipe_HARD_BRAKING = &ram_values.params.hard_braking;
	ParamPipe_MAX_BRAKE_P = &ram_values.params.max_braking_pres;
}

/*
 * Takes in a location in device sram, a page # in high cycle flash, and writes one to the other
 * First clearing the memory with page erase, and then writing the new data from src.
 */
void FLASH_WriteSector(void* src, uint32_t* start, uint32_t len) {
	while((FLASH->NSSR & FLASH_SR_BSY) || (FLASH->NSSR & FLASH_SR_DBNE)); /* Wait for flash to be avaliable */

	if (FLASH->NSCR & FLASH_CR_LOCK) { /* Unlock the flash if it's locked */
		FLASH_Init();
	}

	FLASH->NSSR &= 0xFF << FLASH_SR_EOP_Pos; /* Clear error bits */

	FLASH->NSCR |= FLASH_CR_BKSEL; /* Select bank 2 */

	FLASH->NSCR |= FLASH_CR_SER; /* Tell flash we're doing a sector erase */

	uint32_t sector_start = ((uint32_t)start) - 0x08040000;
	sector_start /= 0x2000;
	uint32_t sector_end = ((uint32_t)start + (len / 4)) - 0x08040000;
	sector_end /= 0x2000;

	for (uint32_t sector = sector_start; sector <= sector_end; sector++) {
		FLASH->NSCR |= FLASH_CR_BKSEL; /* Select bank 2 */
		FLASH->NSCR |= FLASH_CR_SER; /* Tell flash we're doing a sector erase */
		FLASH->NSCR &= ~FLASH_CR_SNB_Msk; /* Clear sector selection bits */

		FLASH->NSCR |= ((sector << FLASH_CR_SNB_Pos) & FLASH_CR_SNB_Msk); /* tell flash which sector to erase */

		FLASH->NSCR |= FLASH_CR_START; /* Start the erase */
		while(FLASH->NSSR & FLASH_SR_BSY); /* wait for the erase to finish */

		FLASH->NSSR &= 0xFF << FLASH_SR_EOP_Pos; /* Clear error bits */
	}

	FLASH->NSCR &= ~FLASH_CR_SER_Msk; /* Tell the flash we're no longer doing a sector erase */

	FLASH->NSSR &= 0xFF << FLASH_SR_EOP_Pos; /* Clear error bits */

	if (!(FLASH->NSCR & FLASH_CR_PG)) { /* Tell the flash we're gonna program if we haven't yet */
		FLASH->NSCR |= FLASH_CR_PG;
	}

	for (uint32_t i = 0; i < len / 4; i++) {
		start[i] = ((uint32_t*)src)[i];
	}

	while(FLASH->NSSR & FLASH_SR_BSY); /* wait for the write to finish */

	FLASH->NSCR &= ~FLASH_CR_PG_Msk; /* clear programming bit */

	FLASH->NSCR |= FLASH_CR_LOCK; /* Lock the flash while we're not using it */
}

void FLASH_EraseMemory() {
    while ((FLASH->NSSR & FLASH_SR_BSY) || (FLASH->NSSR & FLASH_SR_DBNE)); /* Wait for flash to be avaliable */

    if (FLASH->NSCR & FLASH_CR_LOCK) { /* Unlock the flash if it's locked */
	    FLASH_Init();
    }

    FLASH->NSSR &= 0xFF << FLASH_SR_EOP_Pos; /* Clear error bits */

    for (int i = 24; i < 32; i++) {
    	FLASH->NSCR |= FLASH_CR_BKSEL; /* Select bank 2 */

    	FLASH->NSCR |= FLASH_CR_SER; /* Tell flash we're doing a sector erase */

    	FLASH->NSCR &= ~FLASH_CR_SNB_Msk; /* Clear sector selection bits */

    	FLASH->NSCR |= ((i << FLASH_CR_SNB_Pos) & FLASH_CR_SNB_Msk); /* tell flash which sector to erase */

    	FLASH->NSCR |= FLASH_CR_START; /* Start the erase */

    	while(FLASH->NSSR & FLASH_SR_BSY); /* wait for the erase to finish */

    	FLASH->NSSR &= 0xFF << FLASH_SR_EOP_Pos; /* Clear error bits */
    }

    FLASH->NSCR &= ~FLASH_CR_SER; /* Clear sector erase bit. */

    while ((FLASH->NSSR & FLASH_SR_BSY) || (FLASH->NSSR & FLASH_SR_DBNE)); /* Wait for flash to be avaliable */

    FLASH->NSSR &= 0xFF << FLASH_SR_EOP_Pos; /* Clear error bits */

    FLASH->NSCR |= FLASH_CR_LOCK; /* Lock the flash while we're not using it */
}

CarParameters_t* FLASH_getVals() {
	uint16_t* rm_ptr = (uint16_t*)&ram_values;

	const uint16_t* sv_ptr = use_default ? (uint16_t*)&default_vals : (uint16_t*)stored_values;
	const int writes = sizeof(CarParameters_t) / 2;
	if (use_default) {
		FLASH_WriteSector(&default_vals, (uint32_t*)stored_values, sizeof(CarParameters_t));
	}

	if (!ram_initialized) {
		for(int i = 0; i < writes; i++) {
			rm_ptr[i] = sv_ptr[i];
		}
	}

	return &ram_values;
}

int FLASH_getVal(int id) {
	return 	parampipe_getvalue(id);
}

void FLASH_storeVal(int id, int newVal, int write) {
	parampipe_setvalue(id, newVal);

	if (write) {
		FLASH_WriteSector(&ram_values, (uint32_t*)stored_values, sizeof(CarParameters_t));
	}
}
