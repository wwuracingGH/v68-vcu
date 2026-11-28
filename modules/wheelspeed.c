#include "modules/wheelspeed.h"
#include "vendor/CMSIS/Device/ST/STM32H5/Include/stm32h5xx.h"

#define PRESCALER 25
#define PERIOD (PRESCALER / 250000000.0f)

const float OFFSET = 0.85f;
const float DECAY = 10000.0f;

#define PI 3.1415926535f
#define RC 0.005f
#define MAX_TIM (0x80000000)
#define MIN_CONFIDENCE_V 3.0f

WS_data_t WS_wheels[4];

void dataInit(WS_data_t* data);
void update(WS_data_t* data);

void tim2_handler(void) {
    WS_wheels[WS_FRONT_LEFT].last_tick = __QSUB(WS_wheels[WS_FRONT_LEFT].last_tick, MAX_TIM); /* saturating subtract */
    WS_wheels[WS_FRONT_RIGHT].last_tick = __QSUB(WS_wheels[WS_FRONT_RIGHT].last_tick, MAX_TIM); /* saturating subtract */
    TIM2->SR &= ~TIM_SR_UIF;
}

void exti7_handler(void) {
    update(&WS_wheels[WS_FRONT_LEFT]);
    EXTI->RPR1 |= EXTI_RPR1_RPIF7;
}

void exti6_handler(void) {
    update(&WS_wheels[WS_FRONT_RIGHT]);
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

    dataInit(&WS_wheels[WS_FRONT_LEFT]);
    dataInit(&WS_wheels[WS_FRONT_RIGHT]);
    dataInit(&WS_wheels[WS_REAR_LEFT]);
    dataInit(&WS_wheels[WS_REAR_RIGHT]);
}

inline uint32_t get_tick() {
    return TIM2->CNT;
}

inline uint32_t get_d_ticks(const WS_data_t* data) {
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

    /*  Unrolled python code that does the same thing as the below code:
     *  v = math.pi / (dt * 20)
     *  a = dt / (RC + dt)
     *  v_lp = a * v + (1.0 - a) * this.last_v
     *  https://en.wikipedia.org/wiki/Low-pass_filter#Simple_infinite_impulse_response_filter
     */

    /* low pass filter that also inverts the code */
    float v_lp = (PI + 20.0f * RC * data->last_velocity) / (20.0f * (dt + RC));

    data->d_velocity = (v_lp - data->last_velocity) / dt;
    data->last_velocity = v_lp;
}

float max(float f1, float f2) { return (f1 > f2) ? f1 : f2; }
float min(float f1, float f2) { return (f1 < f2) ? f1 : f2; }

float WS_GetVelocity(WheelID id) {
    const WS_data_t* data = &WS_wheels[id];
    float dt = get_d_ticks(data) * PERIOD;

    float f1 = data->last_velocity + data->d_velocity * dt;
    float f2 = data->last_velocity - DECAY * (OFFSET * dt - data->last_dtime);

    return max(min(f1, f2), 0);
}

int16_t canWheelspeeds[4];

int16_t* WS_GetCANMSG() {
    canWheelspeeds[WS_FRONT_LEFT]  = (int16_t)(WS_GetVelocity(WS_FRONT_LEFT)  * 512.0);
    canWheelspeeds[WS_FRONT_RIGHT] = (int16_t)(WS_GetVelocity(WS_FRONT_RIGHT) * 512.0);
    canWheelspeeds[WS_REAR_LEFT]   = (int16_t)(WS_GetVelocity(WS_REAR_LEFT)   * 512.0);
    canWheelspeeds[WS_REAR_RIGHT]  = (int16_t)(WS_GetVelocity(WS_REAR_RIGHT)  * 512.0);

    return canWheelspeeds;
}

float last_est_velocity;

float WS_GetEstimatorVelocity() {
    last_est_velocity = (WS_GetVelocity(WS_FRONT_LEFT) + WS_GetVelocity(WS_FRONT_RIGHT)) / 2.0;
    return last_est_velocity;
}

float WS_GetEstimatorConfidence() {
    float avg_velocity = last_est_velocity;
    float avg_dv = (WS_wheels[WS_FRONT_LEFT].d_velocity + WS_wheels[WS_FRONT_RIGHT].d_velocity) / 2.0f;
    if (avg_dv < 0) avg_dv = -avg_dv;

    return min(5.0 / (avg_dv + 1.0), 1.0) * min(avg_velocity / MIN_CONFIDENCE_V, 1);
}
