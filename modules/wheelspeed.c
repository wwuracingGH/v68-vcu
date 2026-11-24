#include "wheelspeed.h"
#include "stm32h5xx.h"

#define PRESCALER 25
#define PERIOD (PRESCALER / 250000000.0f) 

const float OFFSET = 0.85f;
const float DECAY = 10000.0f;

#define PI 3.14159265359f
#define RC 0.005f
#define MAX_TIM (0x80000000)

struct _ws_data_struct WS_wheels;

void dataInit(WS_data_t* data);
void update(WS_data_t* data);

void tim2_handler(void) {
    WS_wheels.fl.last_tick = __QSUB(WS_wheels.fl.last_tick, MAX_TIM); /* saturating subtract */
    WS_wheels.fr.last_tick = __QSUB(WS_wheels.fr.last_tick, MAX_TIM); /* saturating subtract */
    TIM2->SR &= ~TIM_SR_UIF;
}

void exti7_handler(void) {
    update(&WS_wheels.fl);
    EXTI->RPR1 |= EXTI_RPR1_RPIF7;
}

void exti6_handler(void) {
    update(&WS_wheels.fr);
    EXTI->RPR1 |= EXTI_RPR1_RPIF6;
}

void exti1_handler(void) {
    EXTI->RPR1 |= EXTI_RPR1_RPIF1;
}

void exti0_handler(void) {
    EXTI->RPR1 |= EXTI_RPR1_RPIF0;
}

void WS_Init() {
    RCC->APB1LENR |= RCC_APB1LENR_TIM2EN;
    
    TIM2->PSC = PRESCALER;  /* divides by 25 to get 10 mhz */
    TIM2->ARR = MAX_TIM; /* reloads after like 3m 30s */
    TIM2->CR1 |= TIM_CR1_URS; /* only have an update in the case of a overflow */ 
    TIM2->CR1 |= TIM_CR1_CEN; /* enable the timer */
    
    /*            FL               FR               RL               RR             */
    EXTI->RTSR1 = EXTI_RTSR1_RT7 | EXTI_RTSR1_RT6 | EXTI_RTSR1_RT1 | EXTI_RTSR1_RT0;
    
    EXTI->EXTICR[0] = (0x1 << EXTI_EXTICR1_EXTI0_Pos) | (0x1 << EXTI_EXTICR1_EXTI1_Pos);
    EXTI->EXTICR[1] = (0x0 << EXTI_EXTICR2_EXTI6_Pos) | (0x0 << EXTI_EXTICR2_EXTI7_Pos);

    dataInit(&WS_wheels.fl);
    dataInit(&WS_wheels.fr);
}

inline uint32_t get_tick() {
    return TIM2->CNT;
}

inline uint32_t get_d_ticks(WS_data_t* data) {
    return get_tick() - data->last_tick;
}

void dataInit(WS_data_t* data) {
    data->d_velocity = 0;
    data->last_dtime = 0;
    data->last_tick = get_tick();
    data->last_velocity = 0;
}

void update(WS_data_t* data) {
    float dt = get_d_ticks(data) * PERIOD;
    data->last_dtime = dt;
    data->last_tick = get_tick();

    float v_lp = (PI + 20.0f * RC * data->last_velocity) / (20.0f * (dt + RC));

    data->d_velocity = (v_lp - data->last_velocity) / dt;
    data->last_velocity = v_lp;
}

float max(float f1, float f2) { return (f1 > f2) ? f1 : f2; }
float min(float f1, float f2) { return (f1 < f2) ? f1 : f2; }

float WS_GetVelocity(WS_data_t* data) {
    float dt = get_d_ticks(data) * PERIOD;

    float f1 = data->last_velocity + data->d_velocity * dt;
    float f2 = data->last_velocity - DECAY * (OFFSET * dt - data->last_dtime);

    return max(min(f1, f2), 0);
}
