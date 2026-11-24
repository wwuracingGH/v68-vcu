#include "wheelspeed.h"

float PERIOD;

const float OFFSET = 0.85f;
const float DECAY = 10000.0f;

struct _ws_data_struct WS_wheels;

void dataInit(WS_data_t* data);
void gpio_interrupt_fl();
void gpio_interrupt_fr();
void gpio_interrupt_rr();
void gpio_interrupt_rl();

void WS_Init(uint32_t ticks_of_period) {
    /* init timer 2 or timer 5 here */

    PERIOD = ticks_of_period / 250000000.0f;

    dataInit(&WS_wheels.fl);
    dataInit(&WS_wheels.fr);
}

uint32_t get_tick() {
    /* return the tick */
    return 0;
}

uint32_t get_d_ticks() {
    /* return d tick */
    return 0;
}

void dataInit(WS_data_t* data) {
    data->d_velocity = 0;
    data->last_dtime = 0;
    data->last_tick = get_tick();
    data->last_velocity = 0;
}

void gpio_interrupt_fl() {

}

void gpio_interrupt_fr() {

}

void gpio_interrupt_rr() {
    /* not used */
}

void gpio_interrupt_rl() {
    /* not used */
}

float max(float f1, float f2) { return (f1 > f2) ? f1 : f2; }
float min(float f1, float f2) { return (f1 < f2) ? f1 : f2; }

float WS_GetVelocity(WS_data_t* data) {
    float dt = get_d_ticks() * PERIOD;

    float f1 = data->last_velocity + data->d_velocity * dt;
    float f2 = data->last_velocity - DECAY * (OFFSET * dt - data->last_dtime);

    return max(min(f1, f2), 0);
}
