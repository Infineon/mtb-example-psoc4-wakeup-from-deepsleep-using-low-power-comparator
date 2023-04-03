/******************************************************************************
 * File Name:   main.c
 *
 * Description: This is the source code for the PSoC 4 Wakeup from
 * Deep Sleep using LPComp Example for ModusToolbox.
 *
 * Related Document: See README.md
 *
 *******************************************************************************
 * Copyright 2023, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 *******************************************************************************/

/******************************************************************************
 * Include header files
 ******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"

/*******************************************************************************
 * Macros
 ********************************************************************************/
#define LED_OFF                     (1u)
#define LED_ON                      (0u)

#define LPCOMP_OUTPUT_LOW        (0u)
#define LPCOMP_OUTPUT_HIGH       (1u)

/* Delays */
#define LPCOMP_LP_START_UP_DELAY_US  (10u)
#define TOGGLE_LED_PERIOD            (500u)
#define LED_ON_2S_BEFORE_SLEEP       (2000u)

/* Glitch delays */
#define LONG_GLITCH_DELAY_MS    (250U)   /* in ms */

/* To demonstrate how PDL drivers are used to manually configure the peripherals,
 * set the PDL_CONFIGURATION #define to 1, otherwise set to 0.
 */
#define PDL_CONFIGURATION   (0u)

/*******************************************************************************
 * Function Prototypes
 ********************************************************************************/
void lpcomp_isr();

/*******************************************************************************
 * Global Variables
 ********************************************************************************/
/* LPComp context structure */
cy_stc_lpcomp_context_t lpcomp_context;

/******************************************************************************
 * Interrupt configuration structure
 *******************************************************************************/
const cy_stc_sysint_t lpcomp_intr_config = {
        .intrSrc = lpcomp_interrupt_IRQn,  /* Source of interrupt signal */
        .intrPriority = 3                  /* Interrupt priority */
};

#if PDL_CONFIGURATION
const cy_stc_lpcomp_config_t lpcomp_config =
{
        .outputMode = CY_LPCOMP_OUT_DIRECT,
        .hysteresis = CY_LPCOMP_HYST_DISABLE,
        .power = CY_LPCOMP_MODE_ULP,
        .intType = CY_LPCOMP_INTR_FALLING,
};

cy_stc_gpio_pin_config_t pin_config = {
        /*.outVal     */ 1UL,                       /* Output = High */
        /*.driveMode  */ CY_GPIO_DM_STRONG_IN_OFF,  /* Resistive pull-up, input buffer on */
        /*.hsiom      */ P2_0_GPIO,                 /* Software controlled pin */
        /*.intEdge    */ CY_GPIO_INTR_DISABLE,      /* Rising edge interrupt */
        /*.vtrip      */ CY_GPIO_VTRIP_CMOS,        /* CMOS voltage trip */
        /*.slewRate   */ CY_GPIO_SLEW_FAST,         /* Fast slew rate */
};
#endif

/*******************************************************************************
 * Function Name: main
 ********************************************************************************
 * Summary:
 * System entrance point. This function performs
 *    1. Initializes the BSP.
 *    2. Initialize and enable GPIO interrupt.
 *    3. Initialize and enable the LPComp peripheral.
 *    4. Compares the GPIO input voltages.
 *    5. Operates the CPU in Deep Sleep and wakeup mode.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  int
 *
 *******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution. */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize and enable GPIO interrupt */
    result = Cy_SysInt_Init(&lpcomp_intr_config, lpcomp_isr);
    if(result != CY_SYSINT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Clearing and enabling the GPIO interrupt in NVIC */
    NVIC_ClearPendingIRQ(lpcomp_intr_config.intrSrc);

    NVIC_EnableIRQ(lpcomp_intr_config.intrSrc);

#if PDL_CONFIGURATION

    /* Initialize GPIO pin P2.0 */
    Cy_GPIO_Pin_Init(P2_0_PORT, P2_0_NUM, &pin_config);

    /* Positive input (inp) to the P0[0] */
    Cy_GPIO_Pin_FastInit(GPIO_PRT0, 0, CY_GPIO_DM_ANALOG, 1, HSIOM_SEL_GPIO);

    /* Negative input (inn) to the P0[1] */
    Cy_GPIO_Pin_FastInit(GPIO_PRT0, 1, CY_GPIO_DM_ANALOG, 1, HSIOM_SEL_GPIO);

    /* Initialize the LPComp peripheral */
    if(CY_LPCOMP_SUCCESS != Cy_LPComp_Init(LPCOMP, CY_LPCOMP_CHANNEL_0, &lpcomp_config, &lpcomp_context))
    {
        CY_ASSERT(0);
    }

    /* Configure LPComp interrupt */
    Cy_LPComp_SetInterruptMask(LPCOMP, CY_LPCOMP_CHANNEL0_INTR);

    /* Enable the LPComp channel 0 */
    Cy_LPComp_Enable(LPCOMP, CY_LPCOMP_CHANNEL_0, &lpcomp_context);

    /* It needs 10us start-up time to settle LPComp channel in LP mode after power up */
    Cy_SysLib_DelayUs(LPCOMP_LP_START_UP_DELAY_US);
#else
    /* Initialize and enable the LPComp peripheral */
    if(CY_LPCOMP_SUCCESS != Cy_LPComp_Init(LPCOMP, CY_LPCOMP_CHANNEL_0, &LPComp_1_config, &lpcomp_context))
    {
        CY_ASSERT(0);
    }

    /* Configure LPComp interrupt */
    Cy_LPComp_SetInterruptMask(LPCOMP, CY_LPCOMP_CHANNEL0_INTR);

    Cy_LPComp_Enable(LPCOMP, CY_LPCOMP_CHANNEL_0, &lpcomp_context);
#endif

    /* SysPm callback params */
    cy_stc_syspm_callback_params_t callbackParams = {
            /*.base       =*/ LPCOMP,
            /*.context    =*/ NULL
    };

    /* Callback declaration for Deep Sleep mode */
    cy_stc_syspm_callback_t deep_sleep_cb = {Cy_LPComp_DeepSleepCallback,  /* Callback function */
            CY_SYSPM_DEEPSLEEP,           /* Callback type */
            0,                            /* Skip mode */
            &callbackParams,              /* Callback params */
            NULL, NULL};                  /* For internal usage */

    /* Register Deep Sleep callback */
    if (true != Cy_SysPm_RegisterCallback(&deep_sleep_cb))
    {
        CY_ASSERT(0);
    }

    for (;;)
    {
        if (Cy_LPComp_GetCompare(LPCOMP, CY_LPCOMP_CHANNEL_0) == LPCOMP_OUTPUT_HIGH)
        {
            Cy_GPIO_Inv(P2_0_PORT, P2_0_NUM);
            Cy_SysLib_Delay(TOGGLE_LED_PERIOD);
        }
        else
        {
            Cy_GPIO_Write(P2_0_PORT, P2_0_NUM, LED_ON);
            Cy_SysLib_Delay(LED_ON_2S_BEFORE_SLEEP);
            Cy_GPIO_Write(P2_0_PORT, P2_0_NUM, LED_OFF);

            Cy_SysPm_CpuEnterDeepSleep();
            Cy_SysLib_Delay(LONG_GLITCH_DELAY_MS);
        }
    }
}

/*******************************************************************************
 * Function Name: lpcomp_isr
 ********************************************************************************
 * Summary:
 *  Interrupt service routine for the GPIO interrupt triggered from LPComp.
 *  This function clears the triggered GPIO pin interrupt. 
 *
 * Parameters:
 *  None
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void lpcomp_isr()
{
    /* Clear the triggered pin interrupt */
    Cy_LPComp_ClearInterrupt(LPCOMP, CY_LPCOMP_CHANNEL0_INTR);
}


/* [] END OF FILE */
