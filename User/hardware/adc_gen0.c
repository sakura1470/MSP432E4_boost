
//******************
//配置ADC0 四通道 
//IN：PE3 PE2 PE1 PE0
//OUT: getADCValue[4]
//由PWM1触发中断采样
//******************


#include<main.h>

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

uint16_t getADCValue[4];

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
                /* Enable the DMA and Configure Channel for TIMER0A for Ping Pong mode of
         * transfer */
        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
        while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_UDMA)))
        {
        }

        MAP_uDMAEnable();

        /* Point at the control table to use for channel control structures. */
        MAP_uDMAControlBaseSet(pui8ControlTable);

        /* Map the ADC0 Sequencer 2 DMA channel */
        MAP_uDMAChannelAssign(UDMA_CH16_ADC0_2);

        /* Put the attributes in a known state for the uDMA ADC0 Sequencer 2
         * channel. These should already be disabled by default. */
        MAP_uDMAChannelAttributeDisable(UDMA_CH16_ADC0_2,
                                        UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST |
                                        UDMA_ATTR_HIGH_PRIORITY |
                                        UDMA_ATTR_REQMASK);

        /* Configure the control parameters for the primary control structure for
         * the ADC0 Sequencer 2 channel. The primary control structure is used for
         * copying the data from ADC0 Sequencer 2 FIFO to srcBuffer. The transfer
         * data size is 16 bits and the source address is not incremented while
         * the destination address is incremented at 16-bit boundary.
         */
        MAP_uDMAChannelControlSet(UDMA_CH16_ADC0_2 | UDMA_PRI_SELECT,
                                  UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 |
                                  UDMA_ARB_4);

        /* Set up the transfer parameters for the ADC0 Sequencer 2 primary control
         * structure. The mode is Basic mode so it will run to completion. */
        MAP_uDMAChannelTransferSet(UDMA_CH16_ADC0_2 | UDMA_PRI_SELECT,
                                   UDMA_MODE_BASIC,
                                   (void *)&ADC0->SSFIFO2, (void *)&getADCValue,
                                   sizeof(getADCValue)/2);

        /* The uDMA ADC0 Sequencer 2 channel is primed to start a transfer. As
         * soon as the channel is enabled and the Timer will issue an ADC trigger,
         * the ADC will perform the conversion and send a DMA Request. The data
         * transfers will begin. */
        MAP_uDMAChannelEnable(UDMA_CH16_ADC0_2);
}
