/*
 * buck.h
 *
 *  Created on: 2022年4月5日
 *      Author: sakura
 */

#ifndef BUCK_BUCK_H_
#define BUCK_BUCK_H_

typedef struct PID
{
    float Kp;
    float Ki;
    float Kc;
    float clampMin, clampMax;
    float _sum , _sat ,_saterr;
} PID_v;
 
/*! PID Controller Init
 *
 * @param[in]   kp: Kp
 * @param[in]   ki: Ki
*/
extern void buck_pi_init(float kp, float ki);

extern void buck_update(void);

#endif /* BUCK_BUCK_H_ */
