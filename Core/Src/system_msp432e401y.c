#include <stdint.h>
#include <ti/devices/msp432e4/inc/msp.h>

/*----------------------------------------------------------------------------
  Define clocks
 *----------------------------------------------------------------------------*/
#define __SYSTEM_CLOCK    (16000000ul)

/* Update frequency to match the crystal frequency on your board */
#define XTAL_FREQ         (25000000ul)

/*----------------------------------------------------------------------------
  Clock Variable definitions
 *----------------------------------------------------------------------------*/
uint32_t SystemCoreClock = __SYSTEM_CLOCK; /*!< System Clock Frequency (Core Clock)*/


/*----------------------------------------------------------------------------
  Clock functions
 *----------------------------------------------------------------------------*/
void SystemCoreClockUpdate (void)            /* Get Core Clock Frequency      */
{
    uint32_t getClockDivider;
    uint32_t getPLLMIntValue;
    uint32_t getPLLNValue;
    uint32_t getPLLQValue;

    /* Update the default System Clock value for MSP432E4x devices */
    SystemCoreClock = __SYSTEM_CLOCK;

    if((SYSCTL->RSCLKCFG & SYSCTL_RSCLKCFG_USEPLL) == SYSCTL_RSCLKCFG_USEPLL)
    {
        getClockDivider = (SYSCTL->RSCLKCFG & SYSCTL_RSCLKCFG_PSYSDIV_M) >> SYSCTL_RSCLKCFG_PSYSDIV_S;

        getPLLMIntValue = (SYSCTL->PLLFREQ0 & SYSCTL_PLLFREQ0_MINT_M) >> SYSCTL_PLLFREQ0_MINT_S;
        getPLLNValue = (SYSCTL->PLLFREQ1 & SYSCTL_PLLFREQ1_N_M) >> SYSCTL_PLLFREQ1_N_S;
        getPLLQValue = (SYSCTL->PLLFREQ1 & SYSCTL_PLLFREQ1_Q_M) >> SYSCTL_PLLFREQ1_Q_S;

        if((SYSCTL->RSCLKCFG & SYSCTL_RSCLKCFG_PLLSRC_M) == SYSCTL_RSCLKCFG_PLLSRC_PIOSC)
        {
            SystemCoreClock = (__SYSTEM_CLOCK * (getPLLMIntValue)) / ((getPLLNValue + 1) * (getPLLQValue + 1));
            SystemCoreClock = SystemCoreClock / (getClockDivider + 1);
        }
        else if((SYSCTL->RSCLKCFG & SYSCTL_RSCLKCFG_PLLSRC_M) == SYSCTL_RSCLKCFG_PLLSRC_MOSC)
        {
            SystemCoreClock = (XTAL_FREQ * (getPLLMIntValue)) / ((getPLLNValue + 1) * (getPLLQValue + 1));
            SystemCoreClock = SystemCoreClock / (getClockDivider + 1);
        }
    }
    else
    {
        getClockDivider = (SYSCTL->RSCLKCFG & SYSCTL_RSCLKCFG_OSYSDIV_M) >> SYSCTL_RSCLKCFG_OSYSDIV_S;

        if((SYSCTL->RSCLKCFG & SYSCTL_RSCLKCFG_OSCSRC_M) == SYSCTL_RSCLKCFG_OSCSRC_PIOSC)
        {
            SystemCoreClock = __SYSTEM_CLOCK / (getClockDivider + 1);
        }
        else if((SYSCTL->RSCLKCFG & SYSCTL_RSCLKCFG_OSCSRC_M) == SYSCTL_RSCLKCFG_OSCSRC_MOSC)
        {
            SystemCoreClock = XTAL_FREQ / (getClockDivider + 1);
        }
    }
}

/**
 * Initialize the system
 *
 * @param  none
 * @return none
 *
 * @brief  Setup the microcontroller system.
 *         Initialize the System.
 */
void SystemInit (void)
{
  #if (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10*2) |                 /* set CP10 Full Access */
                   (3UL << 11*2)  );               /* set CP11 Full Access */
  #endif

#ifdef UNALIGNED_SUPPORT_DISABLE
  SCB->CCR |= SCB_CCR_UNALIGN_TRP_Msk;
#endif

  SystemCoreClock = __SYSTEM_CLOCK;

}
