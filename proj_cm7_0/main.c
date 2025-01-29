/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for CM7_0 in MCU switching between 
*              power modes application for ModusToolbox, and demonstrates how 
*              to transition MCU to the following power states:
*              - Power states - Active / Sleep / Deep Sleep / Hibernate
*
* Related Document: See Readme.md
*
*
*******************************************************************************
* Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
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

#include "cybsp.h"
#include "cy_pdl.h"
#include "cy_retarget_io.h"
#include "cy_syspm.h"

/*******************************************************************************
* Macros
********************************************************************************/

/* Constants to define LONG and SHORT presses on User Button (x10 = ms) */
#define QUICK_PRESS_COUNT       2u      /* 20 ms < press < 200 ms */
#define SHORT_PRESS_COUNT       20u     /* 200 ms < press < 2 sec */
#define LONG_PRESS_COUNT        200u    /* press > 2 sec */

/* PWM LED frequency constants (in Hz) */
#define PWM_FREQ_HZ             3
#define PWM_DIM_FREQ_HZ         100

/* PWM Duty cycles (Active Low, in %) */
#define PWM_50P_DUTY_CYCLE      (uint32_t)500
#define PWM_10P_DUTY_CYCLE      (uint32_t)100

/* Glitch delays */
#define SHORT_GLITCH_DELAY_MS   10u     /* in ms */
#define LONG_GLITCH_DELAY_MS    200u    /* in ms */

typedef enum
{
    SWITCH_NO_EVENT     = 0u,
    SWITCH_QUICK_PRESS  = 1u,
    SWITCH_SHORT_PRESS  = 2u,
    SWITCH_LONG_PRESS   = 3u,
} en_switch_event_t;

#define HIB_BTN          CYBSP_USER_BTN1


/*****************************************************************************
* Function Prototypes
********************************************************************************/
en_switch_event_t get_switch_event(void);
void handle_error(void);
cy_en_syspm_status_t pwm_power_callback_Sleep(cy_stc_syspm_callback_params_t* param, cy_en_syspm_callback_mode_t mode);
cy_en_syspm_status_t pwm_power_callback_DSleep(cy_stc_syspm_callback_params_t* param, cy_en_syspm_callback_mode_t mode);
cy_en_syspm_status_t pwm_power_callback_Hibernate(cy_stc_syspm_callback_params_t* param, cy_en_syspm_callback_mode_t mode);

static void gpio_interrupt_handler(void);

/*******************************************************************************
* Global Variables
********************************************************************************/


const cy_stc_sysint_t intr_cfg = {
    /* Bit 0-15 of intrSrc is used to store system interrupt value and bit 16-31 to store CPU IRQ value */
    .intrSrc = ((NvicMux0_IRQn << CY_SYSINT_INTRSRC_MUXIRQ_SHIFT) | ioss_interrupts_gpio_dpslp_0_IRQn),
    .intrPriority = 3
};

cy_stc_syspm_callback_params_t deepSleepParam1 = 
{
    (void*)NULL,
    (void*)CY_SYSPM_SLEEP
};
cy_stc_syspm_callback_t mySleep = 
{
    pwm_power_callback_Sleep,                                    /* callback */
    CY_SYSPM_SLEEP,                                              /* type     */
    0,                                                           /* skipMode */
    &deepSleepParam1,                                            /* callbackParams */
    NULL,                                                        /* prevItm */
    NULL,                                                        /* nextItm */
    0                                                            /* order   */
};
cy_stc_syspm_callback_t myDeepSleep = 
{
    pwm_power_callback_DSleep,                                   /* callback */
    CY_SYSPM_SLEEP,                                              /* type     */
    0,                                                           /* skipMode */
    &deepSleepParam1,                                            /* callbackParams */
    NULL,                                                        /* prevItm */
    NULL,                                                        /* nextItm */
    0                                                            /* order   */
};
cy_stc_syspm_callback_t myHibernate = 
{
    pwm_power_callback_Hibernate,                                /* callback */
    CY_SYSPM_HIBERNATE ,                                         /* type     */
    0,                                                           /* skipMode */
    &deepSleepParam1,                                            /* callbackParams */
    NULL,                                                        /* prevItm */
    NULL,                                                        /* nextItm */
    0                                                            /* order   */
};

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for the CM7_0. It does...
*    1. Initialize the UART block for uart logging.
*    2. Initialize the PWM block that controls the LED brightness.
*    3. Register power management callbacks.
*    Do Forever loop:
*    4. Check if User button was pressed and for how long.
*    5. If quickly pressed, go to Sleep mode.
*    6. If short pressed, go to DeepSleep mode.
*    7. If long pressed, go to Hibernate mode. 
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
    /* API return code */
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io for uart logging */
     Cy_SCB_UART_Init(UART_HW, &UART_config, NULL);
     Cy_SCB_UART_Enable(UART_HW);
     cy_retarget_io_init(UART_HW);
    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");
    printf("********************************************************************************\r\n");
    printf("PDL: Switching between power modes\r\n");
    printf("********************************************************************************\r\n");
    printf("Quickly press the USER BTN1 button to enter Sleep mode.\r\n");
    printf("Short press the USER BTN1 button to enter DeepSleep mode.\r\n");
    printf("Long press the USER BTN1 button to enter Hibernate mode.\r\n\r\n");
    printf("\r\n");

    /* Initialize the User Button */
    /* Enable the GPIO interrupt to wake-up the device */
    Cy_GPIO_ClearInterrupt(CYBSP_USER_BTN_PORT, CYBSP_USER_BTN_PIN);
    Cy_GPIO_SetInterruptEdge(CYBSP_USER_BTN_PORT, CYBSP_USER_BTN_PIN, (uint32_t)CY_GPIO_INTR_FALLING);
    Cy_GPIO_SetInterruptMask(CYBSP_USER_BTN_PORT, CYBSP_USER_BTN_PIN, (uint32_t)1UL);
    Cy_SysInt_Init(&intr_cfg, gpio_interrupt_handler);
    NVIC_ClearPendingIRQ((IRQn_Type)intr_cfg.intrSrc);
    NVIC_EnableIRQ((IRQn_Type) NvicMux0_IRQn);
    /* Initialize the PWM to control LED brightness */
    Cy_TCPWM_PWM_Init(PWM_HW, PWM_NUM, &PWM_config);
    Cy_TCPWM_PWM_Enable(PWM_HW, PWM_NUM);
    Cy_TCPWM_TriggerStart_Single(PWM_HW, PWM_NUM);

    Cy_SysPm_RegisterCallback(&mySleep);
    Cy_SysPm_RegisterCallback(&myDeepSleep);
    Cy_SysPm_RegisterCallback(&myHibernate);

    for(;;)
    {
        switch (get_switch_event())
        {
            case SWITCH_QUICK_PRESS:
                /* Print out the information, wait a bit to UART output */
                printf("Device entered into Sleep mode. Quickly press the USER BTN1 button to return to Active mode.\r\n");
                Cy_SysLib_Delay(SHORT_GLITCH_DELAY_MS);
                /* Go to sleep */
                Cy_SysPm_CpuEnterSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
                printf("Wake up from the Sleep mode.\r\n");
                /* Wait a bit to avoid glitches from the button press */
                Cy_SysLib_Delay(LONG_GLITCH_DELAY_MS);
                break;
            
            case SWITCH_SHORT_PRESS:
                /* Print out the information, wait a bit to UART output */
                printf("Device entered into DeepSleep mode. Quickly press the USER BTN1 button to return to Active mode.\r\n");
                Cy_SysLib_Delay(SHORT_GLITCH_DELAY_MS);

                /* Go to deep sleep */
                Cy_SysPm_CpuEnterDeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
                printf("Wake up from the DeepSleep mode.\r\n");
                /* Wait a bit to avoid glitches from the button press */
                Cy_SysLib_Delay(LONG_GLITCH_DELAY_MS);
                break;

            case SWITCH_LONG_PRESS:
                /* Print out the information, wait a bit to UART output */
                printf("Device entered into Hibernate mode. press the ResetButton to wake-up from Hibernate mode.\r\n");
                Cy_SysLib_Delay(SHORT_GLITCH_DELAY_MS);

                /* Go to hibernate and Configure a low logic level for the first wakeup-pin */
                //Cy_SysPm_SetHibernateWakeupSource(CY_SYSPM_HIBERNATE_PIN5_LOW);
                Cy_SysPm_SetHibernateWakeupSource(CY_SYSPM_HIBERNATE_NO_SRC);
                Cy_SysPm_SystemEnterHibernate();
                break;

            default:
                break;
        }
    }
}

/*******************************************************************************
* Function Name: get_switch_event
****************************************************************************//**
* Summary:
*  Returns how the User button was pressed:
*  - SWITCH_NO_EVENT: No press 
*  - SWITCH_QUICK_PRESS: Very quick press
*  - SWITCH_SHORT_PRESS: Short press was detected
*  - SWITCH_LONG_PRESS: Long press was detected
*
* Return:
*  Switch event that occurred, if any. 
*
*******************************************************************************/
en_switch_event_t get_switch_event(void)
{
    en_switch_event_t event = SWITCH_NO_EVENT;
    uint32_t pressCount = 0;

    /* Check if User button is pressed */
    while (Cy_GPIO_Read(CYBSP_USER_BTN_PORT, CYBSP_USER_BTN_PIN) == CYBSP_BTN_PRESSED)
    {
        /* Wait for 10 ms */
        Cy_SysLib_Delay(10);

        /* Increment counter. Each count represents 10 ms */
        pressCount++;
    }

    /* Check for how long the button was pressed */
    if (pressCount > LONG_PRESS_COUNT)
    {
        event = SWITCH_LONG_PRESS;
    }
    else if (pressCount > SHORT_PRESS_COUNT)
    {
        event = SWITCH_SHORT_PRESS;
    }
    else if (pressCount > QUICK_PRESS_COUNT)
    {
        event = SWITCH_QUICK_PRESS;
    }

    /* Add a delay to avoid glitches */
    Cy_SysLib_Delay(SHORT_GLITCH_DELAY_MS);

    return event;
}

/*******************************************************************************
* Function Name: pwm_power_callback_Sleep
********************************************************************************
* Summary:
*  Callback implementation for the PWM block. It changes the blinking pattern
*  based on the power state and MCU state.
*
* Parameters:
*  state - state the system or CPU is being transitioned into
*  mode  - callback mode
*  arg   - user argument (not used)
*
* Return:
*  Always true
*
*******************************************************************************/
cy_en_syspm_status_t pwm_power_callback_Sleep(cy_stc_syspm_callback_params_t* param, cy_en_syspm_callback_mode_t mode)
{

    /* Stop the PWM before applying any changes */
    Cy_TCPWM_PWM_Disable(PWM_HW, PWM_NUM);

    if (mode == CY_SYSPM_BEFORE_TRANSITION )
    {
        /* Modify the compare value here */
        Cy_TCPWM_Counter_SetCompare0Val(PWM_HW, PWM_NUM, PWM_10P_DUTY_CYCLE);
        /* Restart the counter with the new Compare 0 value */
        Cy_TCPWM_PWM_Enable(PWM_HW, PWM_NUM);
        Cy_TCPWM_TriggerReloadOrIndex_Single(PWM_HW, PWM_NUM);

    }
    else if (mode == CY_SYSPM_AFTER_TRANSITION)
    {
        /* Modify the compare value here */
        Cy_TCPWM_Counter_SetCompare0Val(PWM_HW, PWM_NUM, PWM_50P_DUTY_CYCLE);
        /* Restart the counter with the new Compare 0 value */
        Cy_TCPWM_PWM_Enable(PWM_HW, PWM_NUM);
        Cy_TCPWM_TriggerReloadOrIndex_Single(PWM_HW, PWM_NUM);

    }

    return (cy_en_syspm_status_t)CY_SYSPM_SUCCESS;
}
/*******************************************************************************
* Function Name: pwm_power_callback_DSleep
********************************************************************************
* Summary:
*  Callback implementation for the PWM block. It changes the blinking pattern
*  based on the power state and MCU state.
*
* Parameters:
*  state - state the system or CPU is being transitioned into
*  mode  - callback mode
*  arg   - user argument (not used)
*
* Return:
*  Always true
*
*******************************************************************************/
cy_en_syspm_status_t pwm_power_callback_DSleep(cy_stc_syspm_callback_params_t* param, cy_en_syspm_callback_mode_t mode)
{

    /* Stop the PWM before applying any changes */

    if (mode == CY_SYSPM_BEFORE_TRANSITION )
    {
        Cy_TCPWM_PWM_Disable(PWM_HW, PWM_NUM);
    }
    else if (mode == CY_SYSPM_AFTER_TRANSITION)
    {
        /* Modify the compare value here */
        Cy_TCPWM_Counter_SetCompare0Val(PWM_HW, PWM_NUM, PWM_50P_DUTY_CYCLE);
        /* Restart the counter with the new Compare 0 value */
        Cy_TCPWM_PWM_Enable(PWM_HW, PWM_NUM);
        Cy_TCPWM_TriggerReloadOrIndex_Single(PWM_HW, PWM_NUM);
    }

    return (cy_en_syspm_status_t)CY_SYSPM_SUCCESS;
}
/*******************************************************************************
* Function Name: pwm_power_callback_Hibernate
********************************************************************************
* Summary:
*  Callback implementation for the PWM block. It changes the blinking pattern
*  based on the power state and MCU state.
*
* Parameters:
*  state - state the system or CPU is being transitioned into
*  mode  - callback mode
*  arg   - user argument (not used)
*
* Return:
*  Always true
*
*******************************************************************************/
cy_en_syspm_status_t pwm_power_callback_Hibernate(cy_stc_syspm_callback_params_t* param, cy_en_syspm_callback_mode_t mode)
{

    /* Stop the PWM before applying any changes */
    /* In the case of this CE, the operation can be resumed by pressing the reset button.*/
    return (cy_en_syspm_status_t)CY_SYSPM_SUCCESS;
}
/*******************************************************************************
* Function Name: handle_error
********************************************************************************
* Summary:
* User defined error handling function
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void handle_error(void)
{
    /* Disable all interrupts. */
    __disable_irq();

    CY_ASSERT(0);
}
/*******************************************************************************
* Function Name: gpio_interrupt_handler
********************************************************************************
* Summary:
*   GPIO interrupt handler.
*
* Parameters:
*  void *handler_arg (unused)
*  
*
*******************************************************************************/
static void gpio_interrupt_handler(void)
{
    /* Clears the triggered pin interrupt */
    Cy_GPIO_ClearInterrupt(CYBSP_USER_BTN_PORT, CYBSP_USER_BTN_NUM);
}
/* [] END OF FILE */
