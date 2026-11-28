/**************************************************************************************
 * Viking v68 VCU main.c
 * 
 * Nicole Swierstra, Renee Anderson, Jackson Newman
 * 10/13/2026
 * v68 is derived from v67 logic but integrates FreeRTOS
 *  
 * State machine logic:
 * There are 4 main states:
 * Idle
 * Ready to drive
 * MC-init
 * MC-reset
 * 
 * 
 * Idle is the state the VCU defaults to upon reset
 * Ready to Drive is the state where the motor controller is energized and initialized
 * 
 ***************************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#ifndef STM32H533xx
#define STM32H533xx
#endif
#include "vendor/CMSIS/Device/ST/STM32H5/Include/stm32h5xx.h"
#include "modules/gpio.h"
#include "modules/logging.h"
#include "modules/flash.h"
#include "modules/control.h"
#include "modules/dynamics.h"
#include "modules/can.h"
#include "vendor/printf/printf.h"
#include "canDefinitions.h"

#include "vendor/FreeRTOS-Kernel/include/FreeRTOS.h"
#include "vendor/FreeRTOS-Kernel/include/task.h"

#include "vendor/FreeRTOS-Kernel/include/timers.h"
#include "vendor/FreeRTOS-Kernel/include/semphr.h"

#define INPUT_BUTTONID_RTD 0

GPIO_TypeDef *  buttonPorts[] = { GPIOB };
uint8_t         buttonPins[]  = {    12 };
uint8_t         buttonAcLow[] = {     1 };
const uint8_t   buttonNum     = 1;

/*
 * WHICH PERIPHERALS ARE WHAT & CLOCK SPEED
 */
#define SYS_CLOCK 250000000UL
#define CTRL_CAN FDCAN1
#define DATA_CAN FDCAN2

/* definately roll these into their own things */
/*
 * EXTRA BEHAVIOR - 0 or 1 TODO: put this in flash - not a compiler target
 */
#define MC_WATCHDOG_ENABLED         1 /* drops out of ready to drive upon a fault */
#define MC_ON_BABYSITTING           1 /* enables waiting for motor controller to get enabled */
#define MC_RESET_LOOP               1 /* enables reset loop */
#define IGNORE_BRAKES               0 /* ignores brakes when checking for rtd and plaus */

/*
 * RESETABLE FAULTS
 *
 * these are the faults that go into reset state instead of idle
 */
#define MC_RESET_BITMASK    ( \
                            MC_PFAULT_PRECHARGE_FAILURE | \
                            MC_PFAULT_PRECHARGE_TIMEOUT | \
                            MC_PFAULT_DC_BUS_VOLT_LOW   | \
                            MC_PFAULT_POST_DESATURATION | \
                            \
                            MC_RFAULT_GATE_DESATURATION | \
                            MC_RFAULT_CAN_COMMAND_LOST  | \
                            MC_RFAULT_RESOLVER_DISCONNECTED \
                            )

static SemaphoreHandle_t state_mutex;

/* Car State */
typedef enum {
    VCU_STATE_IDLE      = 0x0,
    VCU_STATE_MCINIT    = 0x1,
    VCU_STATE_RTD       = 0x2,
    VCU_STATE_RESET     = 0x3
} VcuState_t;

struct {
    volatile VcuState_t state;

    volatile uint16_t fault_counter;
    uint16_t plausibility_latch;
    uint16_t last_valid_tr;
    uint16_t last_torque_bounds;

    ControlReq_t          last_ctrl_vec;
    VehicleDynamicState_t last_state_vec;

    ADC_Block_t         adc_dat;
    MC_HighSpeed        mc_hsmsg;
    MC_FaultCodes       mc_faults;
    MC_InternalStates   mc_istates;
    MC_VoltageInfo      mc_vinfo;
    DL_WheelSpeed       dl_wheelspeed;
    DL_CarAcceleration  dl_accel;

    uint32_t last_can_timestamp1; /* any message, fdcan1 */
    uint32_t last_can_timestamp2; /* any message, fdcan2 */
    uint32_t last_mc_reset_timestamp;

    /* more timestamps */
    uint32_t mc_hsmsg_timestamp;
    uint32_t mc_faults_timestamp;
    uint32_t mc_istates_timestamp;
    uint32_t mc_vinfo_timestamp;
    uint32_t dl_wheelspeed_timestamp;
    uint32_t dl_accel_timestamp;
} car_state;

void clock_init();

static int MC_faulted() {
    return car_state.mc_faults.postErrors || car_state.mc_faults.runtimeErrors;
}

static int MC_faultedR() {
    uint64_t resettable = (MC_RESET_BITMASK & car_state.mc_faults.postErrors ||
            (MC_RESET_BITMASK >> 32) & car_state.mc_faults.runtimeErrors);
    uint64_t other = (~(MC_RESET_BITMASK) & car_state.mc_faults.postErrors ||
            ~(MC_RESET_BITMASK >> 32) & car_state.mc_faults.runtimeErrors);
    return resettable && !other;
}

static void Shared_control();
static void Shared_diagnostics();

static void RTD_input();
static void Idle_input();
static void Reset_input();

static void RTD_start();
static void Idle_start();
static void Reset_start();
static void MCInit_start();

static void MCInit_loop();

static void BUZZERON()  { GPIOB->ODR |= GPIO_ODR_OD10;  }
static void BUZZEROFF() { GPIOB->ODR &= ~GPIO_ODR_OD10; }

void msgCallback(uint8_t bus, uint32_t id, uint8_t dlc, uint32_t* data);
void memcpy_32(uint32_t* dest, const uint32_t* src, uint32_t byte_size);

void switch_state(VcuState_t next_state) {
    /* needs a semaphore and mutex so no races */
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    car_state.state = next_state;
    switch (next_state) {
        case VCU_STATE_IDLE:
            Idle_start();
            break;
        case VCU_STATE_MCINIT:
            MCInit_start();
            break;
        case VCU_STATE_RTD:
            RTD_start();
            break;
        case VCU_STATE_RESET:
            Reset_start();
            break;
    }
    xSemaphoreGive(state_mutex);
}

uint32_t GetMS() {
    return pdTICKS_TO_MS(xTaskGetTickCount());
}

const int control_period        =       1;
const int mc_command_period     =       5;
const int process_can_period    =       1;
const int diagnostics_period    =       10;
const int input_period          =       50;
const int mc_watchdog_period    =       100;
const int mc_reset_period       =       50;

CarParameters_t* car_params;
ADC_Mult_t apps_mult = { 0 };

MC_Command          command_msg         = {   0, 0, 1, 0, 0, 0, 0, 0};
MC_ParameterCommand reset_msg           = {  20, 1, 0, 0, 0};
MC_ParameterCommand enable_fastm_msg    = { 227, 1, 0, 0xFFFE, 0};
MC_ParameterCommand shutup_msg          = { 148, 1, 0, 0b0001110011100111, 0xFFFF};

/* FreeRTOS */
#define CAN_RX_TASK_STACK       512 /* 2KB per task */
#define CONTROL_TASK_STACK      512
#define COMMAND_TASK_STACK      512
#define DIAGNOSTIC_TASK_STACK   512
#define INPUT_TASK_STACK        512

#define CONTROL_TASK_PRIORITY  5 /* Highest PRIO */
#define COMMAND_TASK_PRIORITY  4 /* 2nd Highest PRIO */
#define CAN1_RX_TASK_PRIORITY  3
#define CAN2_RX_TASK_PRIORITY  2
#define SLOW_TASK_PRIORITY     1

static void CAN_rxTask(void* params);
static void Control_Task(void* params);
static void Command_Task(void* params);
static void Diagnostic_Task(void* params);
static void Input_Task(void* params);
static void MC_Watchdog_Task();
TimerHandle_t buzzer_timer;
static void buzzer_off_cb(TimerHandle_t t) { (void)t; BUZZEROFF(); }

static void Shared_processCAN(CAN_Message_t* msg);

int main(void) {
    clock_init();
    GPIO_init();
    CTRL_ADCinit();
    logging_init();

    xTaskCreate(CAN_rxTask, "can1_rx", CAN_RX_TASK_STACK, (void*)CAN1_BUS, CAN1_RX_TASK_PRIORITY, NULL);
    xTaskCreate(CAN_rxTask, "can2_rx", CAN_RX_TASK_STACK, (void*)CAN2_BUS, CAN2_RX_TASK_PRIORITY, NULL);

    xTaskCreate(Control_Task, "control_task", CONTROL_TASK_STACK, NULL, CONTROL_TASK_PRIORITY, NULL);
    xTaskCreate(Command_Task, "command_task", COMMAND_TASK_STACK, NULL, COMMAND_TASK_PRIORITY, NULL);

    xTaskCreate(Input_Task,           "input_task", INPUT_TASK_STACK,      NULL, SLOW_TASK_PRIORITY, NULL);
    xTaskCreate(Diagnostic_Task, "diagnostic_task", DIAGNOSTIC_TASK_STACK, NULL, SLOW_TASK_PRIORITY, NULL);
    xTaskCreate(MC_Watchdog_Task,  "watchdog_task", 50, NULL, SLOW_TASK_PRIORITY, NULL);

    state_mutex = xSemaphoreCreateMutex();
    /*
     * Set a timer for 1500 to turn off buzzer using a callback
     */
    buzzer_timer = xTimerCreate("buzzer", pdMS_TO_TICKS(1500), pdFALSE, NULL, buzzer_off_cb);


    // _enable_logging_all();
    LOGLN("STARTING STM");
    car_state.plausibility_latch = 0;
    car_state.last_valid_tr = 0;
    car_params = FLASH_getVals();
    apps_mult = CTRL_getADCMultiplers(&car_params->adc_bounds);
    CAN_init();
    LOG("Hello World!\n");
    LOGLN("CLOCK CHECK: %d", SysTick->LOAD + 1);

    /* switch into idle state before doing anything */
    switch_state(VCU_STATE_IDLE);

    vTaskStartScheduler();
    for (;;) {
        // GO!!!!
    }
}

/*******************************************************************************
 * void CAN_rxTask(void* params)
 * 
 * params given at initialization (see main.c). Basically two tasks, one for each
 * CAN peripheral (two), CAN1 (can1_rx) is higher prio so if CAN2 is writing or
 * logging and CAN2's Queue has messages CAN1 will empty its messages first.
 * Loops stops when CAN_getMessage()'s xQueueReceive(rx_queue[bus], msg, wait)
 * notices queue is empty and sleeps letting other tasks run.
 * unblocks when additional messages come into the queue. When a message arrives
 * in the FIFO an interrupt fires and takes over the CPU (interrupted tasks TCB
 * gets sent to the stack). ISR functions requests xQueueSendFromISR() to write
 * the message to the queue, at the same time it compares the blocked task's
 * priority to the interrupted, regardless it places the blocked task CAN_rxTask
 * onto the READY list according to its priority. xQueueSendFromISR() changes
 * xHigherPriorityTaskWoken if the blocked task (CAN_rxTask) has higher priority
 * than the interrupted task. portYIELD_FROM_ISR(xHigherPriorityTaskWoken) will
 * activate the task switch/preemption if true. At that point instead of returning
 * to the lower priority task it just runs CAN_rxTask instead.
 *******************************************************************************/
static void CAN_rxTask(void* params) {
    CAN_Bus_t bus = (CAN_Bus_t)(uint32_t)params;
    CAN_Message_t msg;

    for (;;) {
        if (CAN_getMessage(bus, &msg, portMAX_DELAY))
            Shared_processCAN(&msg);
    }
}

/*******************************************************************************
 * void Control_Task(void* params)
 * 
 * Wakes every control_period ms.
 * Shared_control
 * MC_sendCommand
 * MC_sendStop
 * MCInit_loop
 *******************************************************************************/
static void Control_Task(void* params) {
	(void)params;
    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        Shared_control();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(control_period));
    }
}

/* cm200 communication is a seperate task for a reason */
static void Command_Task(void* params) {
	(void)params;
    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        if (car_state.state == VCU_STATE_RTD) {
            CAN_sendmessage(CTRL_CAN, MC_CANID_COMMAND, 8, (uint8_t*)&command_msg);
        } else if (car_state.state != VCU_STATE_MCINIT) {
            int ltr = command_msg.torqueCommand;
            command_msg.torqueCommand = 0;
            CAN_sendmessage(CTRL_CAN, MC_CANID_COMMAND, 8, (uint8_t*)&command_msg);
            command_msg.torqueCommand = ltr;
        } else {
            MCInit_loop();
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(mc_command_period));
    }
}

static void MC_Watchdog_Task() {
    if (!MC_WATCHDOG_ENABLED) return;
    if (MC_faulted()) {
        switch_state(VCU_STATE_IDLE);
    }
}

/*******************************************************************************
 * void Diagnostic_Task(void* params)
 * 
 * Maybe work?
 *******************************************************************************/
static void Diagnostic_Task(void* params) {
	(void)params;
    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        Shared_diagnostics();
        _update_logging();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(diagnostics_period));
    }
}

/*******************************************************************************
 * void Input_Task(void* params)
 * 
 * given the period check the state and do
 * Idle_input();
 * RTD_input();
 * Reset_MC();
 * Reset_input();
 * 
 *******************************************************************************/
static void Input_Task(void* params) {
	(void)params;
    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        switch (car_state.state) {
            case VCU_STATE_IDLE:
                Idle_input();
                break;
            case VCU_STATE_RTD:
                RTD_input();
                break;
            case VCU_STATE_RESET:
                Reset_input();
                break;
            default: break;
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(input_period));
    }
}

/* ======= RTD Specific functionality ======= */
static void RTD_start() {
    BUZZERON();
    command_msg.inverterEnable = 1;
    xTimerStart(buzzer_timer, 0); /* Start the Buzzer timer*/
}

static void RTD_input() {
    GPIO_Update(GetMS());
    if (GPIO_buttonReleased(INPUT_BUTTONID_RTD)) {
        GPIO_buttonConsume(INPUT_BUTTONID_RTD); /* action has been processed */
    } else if (GPIO_buttonHeldTime(INPUT_BUTTONID_RTD, GetMS()) > 1000) {
    	GPIO_buttonConsume(INPUT_BUTTONID_RTD); /* action has been processed */
        switch_state(VCU_STATE_IDLE);
    }
}

/* ======= Idle Specific Functionality ======== */
static void Idle_start() {
    command_msg.inverterEnable = 0;
}

/*TODO*/
static void Idle_input() {
	GPIO_Update(GetMS());

    /* check that it is in a state of hard braking before getting into rtd */
    int braking = IGNORE_BRAKES || (car_state.last_ctrl_vec.brake_pressure > car_params->params.hard_braking);
    if (braking && GPIO_buttonReleased(INPUT_BUTTONID_RTD) && car_state.last_valid_tr < 10) {
        GPIO_buttonConsume(INPUT_BUTTONID_RTD);
        if (MC_faultedR()) {
            switch_state(VCU_STATE_RESET);
        } else {
            switch_state(VCU_STATE_MCINIT);
        }
    }
}

/* ====== Shared functionality ====== */
static void Shared_diagnostics() {
    CAN_sendmessage(DATA_CAN, VCU_CANID_APPS_RAW, 8, (uint8_t*)&car_state.adc_dat);
    CAN_sendmessage(DATA_CAN, VCU_CANID_BPS_RAW, 4, (uint8_t*)&car_state.adc_dat.FBPS);
    CAN_sendmessage(DATA_CAN, VCU_CANID_CTRL_VEC, 8, (uint8_t*)&car_state.last_ctrl_vec);

    uint8_t state = car_state.state;
    VCU_VCUState st = { state, car_state.fault_counter, car_state.plausibility_latch, car_state.last_valid_tr };

    CAN_sendmessage(DATA_CAN, VCU_CANID_VCU_STATE, 4, (uint8_t*)&st);
}

static void Shared_processCAN(CAN_Message_t* msg) {
    uint32_t tick = GetMS();
    if (msg->bus_id == CAN1_BUS)
        car_state.last_can_timestamp1 = tick;
    if (msg->bus_id == CAN2_BUS)
        car_state.last_can_timestamp2 = tick;

    switch (msg->id) {
        case MC_CANID_HIGHSPEEDMESSAGE:
            memcpy_32((uint32_t*)&car_state.mc_hsmsg, msg->data, 2);
            car_state.mc_hsmsg_timestamp = tick;
            break;
        case DL_CANID_ACCEL:
            memcpy_32((uint32_t*)&car_state.dl_accel, msg->data, 2);
            car_state.dl_accel_timestamp = tick;
            break;
        case DL_CANID_WHEELSPEED:
            memcpy_32((uint32_t*)&car_state.dl_wheelspeed, msg->data, 2);
            car_state.dl_wheelspeed_timestamp = tick;
            break;
        case MC_CANID_FAULTCODES:
            memcpy_32((uint32_t*)&car_state.mc_faults, msg->data, 2);
            car_state.mc_faults_timestamp = tick;
            break;
        case MC_CANID_VOLTAGEINFO:
            memcpy_32((uint32_t*)&car_state.mc_vinfo, msg->data, 2);
            car_state.mc_vinfo_timestamp = tick;
            break;
        case MC_CANID_INTERNALSTATES:
            memcpy_32((uint32_t*)&car_state.mc_istates, msg->data, 2);
            car_state.mc_istates_timestamp = tick;
            break;
        /* TODO: Reprogram/Reveal */
        case VCU_CANID_PARAM_CHANGE:
        	VCU_ParamSet* ps = (VCU_ParamSet*)msg->data;
        	FLASH_storeVal(ps->id, ps->setValue, ps->write);
        	apps_mult = CTRL_getADCMultiplers(&car_params->adc_bounds);
        	break;
        case VCU_CANID_PARAM_REQUEST:
        	VCU_ParamReq* rq = (VCU_ParamReq*)msg->data;
        	VCU_ParamReveal rv = { FLASH_getVal(rq->id), rq->id };
        	CAN_sendmessage(msg->bus_id ? DATA_CAN : CTRL_CAN, VCU_CANID_PARAM_REVEAL, 6, (uint8_t*)&rv);
            break;
        default:
            break;
    }
}

static void Shared_control() {
    const int fault_ignore     = 20;
    const int fault_cutoff     = 80;
    const int fault_multiplier = 65536 / (fault_cutoff - fault_ignore);

    int max_torque = car_params->params.max_torque;
    ControlReq_t tr = CTRL_getCommand(&apps_mult, &car_params->params, max_torque);

    /* the silly */
    if (IGNORE_BRAKES) tr.flags &= ~(APPS_FAULT_PLAUS | APPS_FAULT_BSE);

    if ((tr.flags & APPS_FAULT_PLAUS)) {
        car_state.plausibility_latch = 1;
    } else if (car_state.plausibility_latch) {
        if (tr.torque < (car_params->params.max_torque / 10)) {
            car_state.plausibility_latch = 0;
        }
    }

    if (!((tr.flags & 0xFF00) || car_state.plausibility_latch)) {
        car_state.fault_counter = 0;
        car_state.last_valid_tr = tr.torque;
        command_msg.torqueCommand = tr.torque;
    } else {
    	if (car_state.fault_counter < 100) {
    		car_state.fault_counter += control_period;
        }

        if (car_state.fault_counter < fault_ignore) {
            command_msg.torqueCommand = car_state.last_valid_tr;
        } else if (car_state.fault_counter < fault_cutoff) {
            int j = fault_multiplier * (fault_cutoff - car_state.fault_counter); /* 1092 ~= 2^16 / 60 */
            command_msg.torqueCommand = (j * car_state.last_valid_tr) >> 16UL;
        } else {
            command_msg.torqueCommand = 0;
        }
    }

    car_state.last_ctrl_vec = tr;
    car_state.last_ctrl_vec.torque = command_msg.torqueCommand;

    LOG("%d %d %d %d\n", command_msg.torqueCommand, car_state.fault_counter, tr.torque, tr.flags);
}

/* ======= MC Init Specific Functionality ======= */
static void MCInit_start() {
    // disable lockout
    command_msg.inverterEnable = 1;
}

static void MCInit_loop() {
    /* Assuming this is fault protection */
    if (car_state.mc_istates.vsmState == 7) {
        switch_state(VCU_STATE_IDLE);
    } else if (car_state.mc_istates.vsmState == 6 || !MC_ON_BABYSITTING) {
        switch_state(VCU_STATE_RTD);
    } else if (car_state.mc_istates.vsmState == 4) { /* failed to to turn on try again*/
        MCInit_start();
    }
}



/* ======= Reset Specific Functionality ======== */
static void Reset_start() {
    CAN_sendmessage(CTRL_CAN, MC_CANID_PARAMCOM, 8, (uint8_t*)&reset_msg);
}

static void Reset_input() {
    /*TODO: proper timestamps*/
	GPIO_Update(GetMS());

    if (!MC_RESET_LOOP) {
        switch_state(VCU_STATE_IDLE);
    }

    int fltr = MC_faultedR();
    int flt = MC_faulted();

    if (!fltr && flt) {
       switch_state(VCU_STATE_IDLE);
       return;
    } else if (!flt) {
        switch_state(VCU_STATE_IDLE);
        return;
    }

    /* if pressed, cancel reset sequence */
    if (GPIO_buttonReleased(INPUT_BUTTONID_RTD)) {
        GPIO_buttonConsume(INPUT_BUTTONID_RTD);

        switch_state(VCU_STATE_IDLE);
    }
}

/* ======= Various util functions ======= */
void clock_init() { /* turns up the speed to 250mhz */
    /* use voltage scaling profile 0 */
    PWR->VOSCR |= (0b11 << PWR_VOSCR_VOS_Pos);
    /* change the flash access latency */
    FLASH->ACR |= 5 << FLASH_ACR_LATENCY_Pos;
    /* change the flash delay */
    FLASH->ACR |= 2 << FLASH_ACR_WRHIGHFREQ_Pos;

    /* prefetch */
    FLASH->ACR |= FLASH_ACR_PRFTEN;

    /* hsi clock /= 8 */
    RCC->CR |= (3 << RCC_CR_HSIDIV_Pos);

    /* enable pll1, divide incoming by 2 */
    RCC->PLL1CFGR |= RCC_PLL1CFGR_PLL1PEN | RCC_PLL1CFGR_PLL1QEN | (2 << RCC_PLL1CFGR_PLL1M_Pos);
    /* set the pll1 src to hsi clock, input freq is 2 - 4 */
    RCC->PLL1CFGR |= (1 << RCC_PLL1CFGR_PLL1SRC_Pos) | (1 << RCC_PLL1CFGR_PLL1RGE_Pos);

    /* set clock speed to 250mhz */
    RCC->PLL1DIVR &= ~(RCC_PLL1DIVR_PLL1N_Msk | RCC_PLL1DIVR_PLL1Q_Msk);
    RCC->PLL1DIVR |= ((((SYS_CLOCK/1000000UL) >> 1) - 1) << RCC_PLL1DIVR_PLL1N_Pos);
    RCC->PLL1DIVR |= (3 << RCC_PLL1DIVR_PLL1Q_Pos); /* PLL1_Q divide by 4 */

    /* Enables PLL1 and locks values */
    RCC->CR |= RCC_CR_PLL1ON;
    while (!(RCC->CR & RCC_CR_PLL1RDY)) {}

    /* sets system clock source as PLL1 */
    RCC->CFGR1 |= 0b11 << RCC_CFGR1_SW_Pos;
    while (!(RCC->CFGR1 & (0b11 << RCC_CFGR1_SWS_Pos))) {}

    /* sets CAN source as PLL1 Q ); */
    RCC->CCIPR5 |= 0b01 << RCC_CCIPR5_FDCANSEL_Pos;
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    (void)xTask;
    (void)pcTaskName;
    __asm volatile("cpsid i");
    for (;;) {}
}

