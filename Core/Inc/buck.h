/*
 * buck.h
 *
 *  Created on: 2022年4月5日
 *      Author: sakura
 */

#ifndef BUCK_BUCK_H_
#define BUCK_BUCK_H_

typedef struct tagBuck_PID
{
    uint16_t kp, ki, kd, kc;
    uint16_t clampMin, clampMax;
// private:
    uint16_t _sum;
    uint16_t _sat;
    uint16_t _saterr;
} Buck_PID;
// 构造
// kp: 比例系数
// ki: 积分系数
// kd: 微分系数
extern void Buck_PID_Construct(Buck_PID *pid, uint16_t kp, uint16_t ki, uint16_t kd, uint16_t kc);
// 设置clamp
extern void Buck_PID_SetClamp(Buck_PID *pid, uint16_t min, uint16_t max);
// 传递(PI)
// err: 当前误差
extern uint16_t Buck_PI_Transfer(Buck_PID *pid, uint16_t err);

extern void buck_control_init(void);

extern void buck_update(float v_set);

#endif /* BUCK_BUCK_H_ */
