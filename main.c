/******************************************************************************
 * File Name:   main.c
 *
 * Description: This is the source code for the PSoC 4 Wakeup from
 * Deep Sleep using LPComp Example for ModusToolbox.
 *
 * Related Document: See README.md
 *
 *******************************************************************************
 * Copyright 2023-2024, Cypress Semiconductor Corporation (an Infineon company) or
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
 * Global Variables
 ********************************************************************************/
/* LPComp context structure */
cy_stc_lpcomp_context_t lpcomp_context;

/*******************************************************************************
 * Function Prototypes
 ********************************************************************************/
/* LPComp interrupt service routine */
void lpcomp_isr_callback();

/******************************************************************************
 * Interrupt configuration structure
 *******************************************************************************/
 /* LPComp interrupt service routine configuration */
const cy_stc_sysint_t lpcomp_intr_config = {
    .intrSrc = lpcomp_interrupt_IRQn,  /* Source of interrupt signal */
    .intrPriority = 3                  /* Interrupt priority */
};

/*******************************************************************************
 * Function Name: main
 ********************************************************************************
 * Summary:
 * System entrance point. This function performs
 *    1. Initializes the BSP.
 *    2. Initialize and enable LPComp interrupt.
 *    3. Initialize and enable the LPComp peripheral.
 *    4. Compares the LPComp input voltages.
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

    /* Variable to store the calibration value */
    uint32_t trim_val;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        /* Insert the error handling here */
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initializes LPComp interrupt */
    result = Cy_SysInt_Init(&lpcomp_intr_config, lpcomp_isr_callback);
    if(result != CY_SYSINT_SUCCESS)
    {
        /* Insert the error handling here */
        CY_ASSERT(0);
    }

    /* Clearing and enabling the LPComp interrupt in NVIC */
    NVIC_ClearPendingIRQ(lpcomp_intr_config.intrSrc);

    /* Enables LPComp interrupt */
    NVIC_EnableIRQ(lpcomp_intr_config.intrSrc);

    /* Initializes the LPComp peripheral */
    if(CY_LPCOMP_SUCCESS != Cy_LPComp_Init(LPCOMP, CY_LPCOMP_CHANNEL_0, &lpcomp_config, &lpcomp_context))
    {
        /* Insert the error handling here */
        CY_ASSERT(0);
    }

    /* Configure LPComp interrupt */
    Cy_LPComp_SetInterruptMask(LPCOMP, CY_LPCOMP_CHANNEL0_INTR);

    /* Enables the LPComp peripheral */
    Cy_LPComp_Enable(LPCOMP, CY_LPCOMP_CHANNEL_0, &lpcomp_context);

    /* Performs custom calibration of the input offset to minimize the error for a
    * specific set of conditions: the comparator reference voltage, supply voltage,
    * and operating temperature. A reference voltage in the range at which the
    * comparator will be used must be applied to the Vminus input of the
    * comparator. This can be done using an external resistive divider for example.
    */
    trim_val = Cy_LPComp_ZeroCal(LPCOMP, CY_LPCOMP_CHANNEL_0);

    /* Sets the Comparator trim value. Actually it is already done as a part of
    * ZeroCal() function. This demonstrates method to set trim register with specific
    * value.
    */
    Cy_LPComp_LoadTrim(LPCOMP, CY_LPCOMP_CHANNEL_0, trim_val);

    /* Sets Drive power mode and speed configuration. */
    Cy_LPComp_SetPower(LPCOMP, CY_LPCOMP_CHANNEL_0, CY_LPCOMP_MODE_ULP);

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
        /* Insert the error handling here */
        CY_ASSERT(0);
    }

    for (;;)
    {
        /* Sets the system into deep sleep power mode */
        Cy_SysPm_CpuEnterDeepSleep();
    }
}

/*******************************************************************************
 * Function Name: lpcomp_isr_callback
 ********************************************************************************
 * Summary:
 *  Interrupt service routine for LPComp.
 *  This function clears the triggered LPComp interrupt.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void lpcomp_isr_callback(void)
{
    /* Set the LED value accordingly to the PLComp output */
    Cy_GPIO_Write(USER_LED_PORT, USER_LED_NUM, Cy_LPComp_GetCompare(LPCOMP, CY_LPCOMP_CHANNEL_0));

    /* Clear pending interrupts */
    Cy_LPComp_ClearInterrupt(LPCOMP, CY_LPCOMP_CHANNEL0_INTR);
}


/* [] END OF FILE */
