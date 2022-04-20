/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
/******************************************************************************
 * MSP432E4 Empty Project
 *
 * Description: An empty project that uses DriverLib
 *
 *                MSP432E401Y
 *             ------------------
 *         /|\|                  |
 *          | |                  |
 *          --|RST               |
 *            |                  |-->PF2(PWM2)
 *(ADC0)PE3-->|                  |-->PF3(PWM3)
 *(ADC1)PE2-->|                  |
 *            |                  |
 *            |                  |
 * Author: 
*******************************************************************************/
#include <main.h>
#include <pwm_gen1.h>
#include <buck.h>
#include <adc_gen0.h>
#include <uartstdio.h>
#include <timer0A.h>
#include <stdarg.h>

uint32_t getSystemClock;
extern uint32_t getADCValue[4];

volatile bool bgetConvStatus = false;;

void ConfigureUART(uint32_t systemClock)
{
    /* Enable the clock to GPIO port A and UART 0 */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_UART0))
    {

    }
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_UART0))
    {

    }


    /* Configure the GPIO Port A for UART 0 */
    MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
    MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    /* Configure the UART for 115200 bps 8-N-1 format */
    UARTStdioConfig(0, 115200, systemClock);
}

void ADC0SS2_IRQHandler(void)
{
      uint32_t getIntStatus;

    /* Get the interrupt status from the ADC */
    getIntStatus = MAP_ADCIntStatus(ADC0_BASE, 2, true);

    /* If the interrupt status for Sequencer-2 is set the
     * clear the status and read the data */
    if(getIntStatus == 0x4)
    {
        /* Clear the ADC interrupt flag. */
        MAP_ADCIntClear(ADC0_BASE, 2);

        /* Read ADC Value. */
        MAP_ADCSequenceDataGet(ADC0_BASE, 2, getADCValue);

        bgetConvStatus = true;
    }
}

void main(void)
 {
    getSystemClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                     SYSCTL_OSC_MAIN |
                     SYSCTL_USE_PLL |
                     SYSCTL_CFG_VCO_480), 120000000);
    ConfigureUART(getSystemClock);
    adc_init();
    timer1_init(getSystemClock);
    buck_pi_init(0.1, 0.1);
    while(1)
    {
        while(!bgetConvStatus);
        bgetConvStatus = false;
        /* Display the AIN0-AIN03 (PE3-PE0) digital value on the console. */
        UARTprintf("Data = %4d %4d %4d %4d\r", getADCValue[0],
                   getADCValue[1], getADCValue[2], getADCValue[3]);
    }
}
