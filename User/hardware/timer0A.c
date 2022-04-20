#include<main.h>
#include<buck.h>


extern void timer1_init(uint32_t systemClock)
{
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);


    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0))
    {
    }
    //
    // Enable processor interrupts.
    //
    MAP_IntMasterEnable();

    //
    // Configure the 32-bit periodic timers.
    //
    MAP_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    MAP_TimerLoadSet(TIMER0_BASE, TIMER_A, systemClock/100);

    //
    // Setup the interrupts for the timer timeouts.
    //
    MAP_IntEnable(INT_TIMER0A);
    MAP_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    //
    // Enable the timers.
    //
    MAP_TimerEnable(TIMER0_BASE, TIMER_A);
}

void TIMER0A_IRQHandler(void)
{
    uint16_t gettimerstatus;
    gettimerstatus = MAP_TimerIntStatus(gettimerstatus,true);
    while(!gettimerstatus)
    {
        buck_update();
    }
}