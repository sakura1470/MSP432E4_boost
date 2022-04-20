#include<main.h>

uint32_t getADCValue[4];

extern uint32_t getSystemClock;

/* The control table used by the uDMA controller.  This table must be aligned
 * to a 1024 byte boundary. */
#if defined(__ICCARM__)
#pragma data_alignment=1024
uint8_t pui8ControlTable[1024];
#elif defined(__TI_ARM__)
#pragma DATA_ALIGN(pui8ControlTable, 1024)
uint8_t pui8ControlTable[1024];
#else
uint8_t pui8ControlTable[1024] __attribute__ ((aligned(1024)));
#endif


void adc_init(void)
{
    /* Enable the clock to GPIO Port E and wait for it to be ready */
        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
        while(!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)))
        {
        }

        /* Configure PE0-PE3 as ADC input channel */
        MAP_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
        MAP_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);
        MAP_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1);
        MAP_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0);

        /* Enable the clock to ADC-0 and wait for it to be ready */
        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
        while(!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)))
        {
        }

        /* Configure Sequencer 2 to sample the analog channel : AIN0-AIN3. The
         * end of conversion and interrupt generation is set for AIN3 */
        MAP_ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_CH0);
        MAP_ADCSequenceStepConfigure(ADC0_BASE, 2, 1, ADC_CTL_CH1);
        MAP_ADCSequenceStepConfigure(ADC0_BASE, 2, 2, ADC_CTL_CH2);
        MAP_ADCSequenceStepConfigure(ADC0_BASE, 2, 3, ADC_CTL_CH3 | ADC_CTL_IE |
                                     ADC_CTL_END);

        /* Enable sample sequence 2 with a timer signal trigger.  Sequencer 2
         * will do a single sample when the timer generates a trigger on timeout*/
        MAP_ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PWM1, 2);

        /* Since sample sequence 2 is now configured, it must be enabled. */
        MAP_ADCSequenceEnable(ADC0_BASE, 2);

        /* Clear the interrupt status flag before enabling. This is done to make
         * sure the interrupt flag is cleared before we sample. */
        MAP_ADCIntClearEx(ADC0_BASE, 2);
        MAP_ADCIntEnableEx(ADC0_BASE, 2);

        /* Enable the Interrupt generation from the ADC-0 Sequencer */
        MAP_IntEnable(INT_ADC0SS2);
}
