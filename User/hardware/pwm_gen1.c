/*
 * pwm.c
 *
 *  Created on: 2022年3月27日
 *      Author: sakura
 *      OUT:PF2 PF3
 */
#include<main.h>
#define pwm 100000
#define SystemClock 120000000


void PWM0init(float pwm_width)
{
    /* The PWM peripheral must be enabled for use. */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    while(!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0)));

    /* Set the PWM clock to the system clock. */
    MAP_PWMClockSet(PWM0_BASE,PWM_SYSCLK_DIV_1);

    /* Enable the clock to the GPIO Port F for PWM pins */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));

    MAP_GPIOPinConfigure(GPIO_PF2_M0PWM2);
    MAP_GPIOPinConfigure(GPIO_PF3_M0PWM3);
    MAP_GPIOPinTypePWM(GPIO_PORTF_BASE, (GPIO_PIN_2 | GPIO_PIN_3));

    /* Configure the PWM0 to count up/down without synchronization.
     * Note: Enabling the dead-band generator automatically couples the 2
     * outputs from the PWM block so we don't use the PWM synchronization. */
    MAP_PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_UP_DOWN |
                        PWM_GEN_MODE_NO_SYNC);

    /* Set the PWM period to 250Hz.  To calculate the appropriate parameter
     * use the following equation: N = (1 / f) * SysClk.  Where N is the
     * function parameter, f is the desired frequency, and SysClk is the
     * system clock frequency.
     * In this case you get: (1 / 250Hz) * 16MHz = 64000 cycles.  Note that
     * the maximum period you can set is 2^16 - 1. */
    MAP_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, SystemClock/pwm);

    /* Set PWM0 PK4 to a duty cycle of 25%.  You set the duty cycle as a
     * function of the period.  Since the period was set above, you can use the
     * PWMGenPeriodGet() function.  For this example the PWM will be high for
     * 25% of the time or 16000 clock cycles (64000 / 4). */
    MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2,
                     MAP_PWMGenPeriodGet(PWM0_BASE, PWM_GEN_1)*pwm_width);

    /* Enable the dead-band generation on the PWM0 output signal.  PWM bit 2
     * (PF2), will have a duty cycle of 25% (set above) and PWM bit 3 will have
     * a duty cycle of 75%.  These signals will have a 10us gap between the
     * rising and falling edges.  This means that before PWM bit 2 goes high,
     * PWM bit 3 has been low for at LEAST 160 cycles (or 10us) and the same
     * before PWM bit 3 goes high.  The dead-band generator lets you specify
     * the width of the "dead-band" delay, in PWM clock cycles, before the PWM
     * signal goes high and after the PWM signal falls.  For this example we
     * will use 160 cycles (or 10us) on both the rising and falling edges of
     * PF2.  Reference the datasheet for more information on dead-band
     * generation. */
    MAP_PWMDeadBandEnable(PWM0_BASE, PWM_GEN_1, MAP_PWMGenPeriodGet(PWM0_BASE, PWM_GEN_1)/100, MAP_PWMGenPeriodGet(PWM0_BASE, PWM_GEN_1)/100);

    MAP_IntMasterEnable();

    /* This timer is in up-down mode.  Interrupts will occur when the
     * counter for this PWM counts to the load value (64000), when the
     * counter counts up to 64000/4 (PWM A Up), counts down to 64000/4
     * (PWM A Down), and counts to 0. */
    MAP_PWMGenIntTrigEnable(PWM0_BASE, PWM_GEN_1,PWM_TR_CNT_AD);
    MAP_IntEnable(INT_PWM0_1);
    MAP_PWMIntEnable(PWM0_BASE, PWM_INT_GEN_1);

    /* Enable the PWM0 Bit 2 (PF2) and Bit 3 (PF3) output signals. */
    MAP_PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT | PWM_OUT_3_BIT, true);

    /* Enables the counter for a PWM generator block. */
    MAP_PWMGenEnable(PWM0_BASE, PWM_GEN_1);

    /* Loop forever while the PWM signals are generated. */
}



