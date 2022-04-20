/*
 * buck.c
 *
 *  Created on: 2022年4月5日
 *      Author: sakura
 */
/* DriverLib Includes */
#include <main.h>

#include <adc_gen0.h>
#include <pwm_gen1.h>
#include <buck.h>

extern uint32_t getADCValue[4];
#define point 96.2
extern PID_v pid_v;

extern float set;


// error state size
#define ERROR_STATE_SIZE        3

#define CUR_STATE               0
#define LAST_STATE              1
#define LAST_LAST_STATE         2

typedef struct tagPIController {
    // params
    float kp, ki;
    // error state
    float err[ERROR_STATE_SIZE];
} PIController;

float g_err, g_out, g_adc_in;

//! Global PI
static PIController pi_ctl = { 0 };

//
extern void buck_pi_init(float kp, float ki)
{
    pi_ctl.kp = kp;
    pi_ctl.ki = ki;
}

//! pi transfer
static float pi_transfer(float err)
{
    float inc_i = err - 2 * pi_ctl.err[LAST_STATE] + pi_ctl.err[LAST_LAST_STATE];
    // update state
    pi_ctl.err[LAST_LAST_STATE] = pi_ctl.err[LAST_STATE];
    pi_ctl.err[LAST_STATE] = pi_ctl.err[CUR_STATE];
    pi_ctl.err[CUR_STATE] = err;
    // forward transfer
    float cur_out = pi_ctl.kp * err + inc_i;
    // clamp
    if (cur_out < 0.2f)
    {
        cur_out = 0.2f;
    }
    else if (cur_out > 0.8f)
    {
        cur_out = 0.8;
    }
    // TODO clamp error
    return cur_out;
}

extern void buck_update(void)
{
    float v_ref = 8.0f;
    //
    // float u_in = getADCValue[0] / 96.2f;
    static float u_out_filter = 0;
    float u_out = getADCValue[1] / 96.2f;
    u_out = (u_out_filter * 3.0f + u_out) / 4.0f;
    g_adc_in = u_out;
    //
    float err = u_out - v_ref;
    g_err = err;
    //
    float timer_d = pi_transfer(err);
    g_out = timer_d;
    //
    PWM0init(timer_d);
}
