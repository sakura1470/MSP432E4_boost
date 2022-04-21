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
    float kp, ki, kd, kc;
    float clampMin, clampMax;
// private:
    float _sum;
    float _sat;
    float _saterr;
} Buck_PID;
// 构造
// kp: 比例系数
// ki: 积分系数
// kd: 微分系数
extern void Buck_PID_Construct(Buck_PID *pid, float kp, float ki, float kd, float kc);
// 设置clamp
extern void Buck_PID_SetClamp(Buck_PID *pid, float min, float max);
// 传递(PI)
// err: 当前误差
extern float Buck_PI_Transfer(Buck_PID *pid, float err);

extern void buck_control_init(void);

extern void buck_update(void);

#endif /* BUCK_BUCK_H_ */
