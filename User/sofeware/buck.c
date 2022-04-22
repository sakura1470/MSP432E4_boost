/*
 * buck.c
 *
 *  Created on: 2022年4月5日
 *      Author: sakura
 */
#include <main.h>

#include <adc_gen0.h>
#include <pwm_gen1.h>
#include <buck.h>

#define Pu 4096
#define clamp_max 3276
#define clamp_min 819

extern uint16_t getADCValue[4];

#define SCALE_RATIO     96.106f

uint16_t g_err, g_out, g_adc_in;

// pid
Buck_PID pid;

extern void Buck_PID_Construct(Buck_PID *pid, uint16_t kp, uint16_t ki, uint16_t kd, uint16_t kc)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->kc = kc;
    //
    pid->clampMax = INT16_MAX;
    pid->clampMin = INT16_MIN;
    //
    pid->_sum = 0;
    pid->_sat = 0;
    pid->_saterr = 0;
}
// 设置clamp
extern void Buck_PID_SetClamp(Buck_PID *pid, uint16_t min, uint16_t max)
{
    pid->clampMin = min;
    pid->clampMax = max;
}
// 传递
// err: 当前误差
extern uint16_t Buck_PI_Transfer(Buck_PID *pid, uint16_t err)
{
    uint16_t output = 0;
    // 饱和前输出 = 比例输出 + 上次积分值
    pid->_sat = pid->kp * err + pid->_sum;
    // 饱和限制
    if (pid->_sat > pid->clampMax)
    {
        output = pid->clampMax;
    }
    else if (pid->_sat < pid->clampMin)
    {
        output = pid->clampMin;
    }
    else {
        output = pid->_sat;
    }
    // 计算饱和误差
    uint16_t satErr = output - pid->_sat;
    // 带饱和校正的积分输出
    pid->_sum += pid->ki * err + pid->kc * satErr;
    // 积分结果clamp
    if (pid->_sum > pid->clampMax)
    {
        pid->_sum = pid->clampMax;
    }
    else if (pid->_sum < pid->clampMin)
    {
        pid->_sum = pid->clampMin;
    }
    return output;
}
//pid初始化
extern void buck_control_init(void)
{
    Buck_PID_Construct(&pid, 0.03, 0.002, 0, 0);
    Buck_PID_SetClamp(&pid, clamp_min, clamp_max);
}


extern void buck_update(float v_set)
{
    uint16_t v_ref = v_set * SCALE_RATIO ;
    static uint16_t u_filter = 0;
    //
    uint16_t u_out  = getADCValue[1];
    u_filter = (u_filter * 7 + u_out ) * 0.125;
    u_out = u_filter;
    //
    g_adc_in = u_out;
    //
    uint16_t err = v_ref - u_out;
    g_err = err;
    //
    uint16_t timer_d = Buck_PI_Transfer(&pid, err);
    g_out = timer_d;
    //
    PWM0init(timer_d / Pu);
}

