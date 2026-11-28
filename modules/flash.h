#ifndef _MODULES_FLASH_H_
#define _MODULES_FLASH_H_
#include "modules/control.h"
#include "modules/dynamics.h"

typedef struct {
    uint32_t stored_len;
    ADC_Bounds_t adc_bounds;
    ControlParams_t params;
    VehicleDynamicParams_t vdp;
} CarParameters_t;

/* 
 * Unlocks the flash to be able to write to it 
 */
void FLASH_Init();

/*
 * Takes in a location in device sram, a page # in high cycle flash, and writes one to the other
 * First clearing the memory with page erase, and then writing the new data from src.
 */
void FLASH_WriteSector(void* src, uint32_t* start, uint32_t len);

/*
 * Clears all high cycle data.
 */
void FLASH_EraseMemory();

/*
 * Returns a reference to the values stored in SRAM
 */
CarParameters_t* FLASH_getVals();
int FLASH_getVal();

/*
 * stores the value newVal in the value with code id in ram
 */
void FLASH_storeVal(int id, int newVal, int write);

#endif
