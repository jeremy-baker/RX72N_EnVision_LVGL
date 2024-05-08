/***********************************************************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products.
* No other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
* applicable laws, including copyright laws. 
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING THIS SOFTWARE, WHETHER EXPRESS, IMPLIED
* OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NON-INFRINGEMENT.  ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY
* LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE FOR ANY DIRECT,
* INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR
* ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability 
* of this software. By using this software, you agree to the additional terms and conditions found by accessing the 
* following link:
* http://www.renesas.com/disclaimer
*
* Copyright (C) 2022 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/

/***********************************************************************************************************************
* File Name        : Config_TMR0.c
* Component Version: 1.10.0
* Device(s)        : R5F572NNHxFB
* Description      : This file implements device driver for Config_TMR0.
***********************************************************************************************************************/

/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
/* Start user code for pragma. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "r_cg_macrodriver.h"
#include "Config_TMR0.h"
/* Start user code for include. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
#include "r_cg_userdefine.h"

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: R_Config_TMR0_Create
* Description  : This function initializes the TMR0 channel
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

void R_Config_TMR0_Create(void)
{

    /* Cancel TMR module stop state */
    MSTP(TMR01) = 0U;

    /* Set counter clear and interrupt */
    TMR0.TCR.BYTE = _00_TMR_CNT_CLR_DISABLE | _00_TMR_CMIA_INT_DISABLE | _00_TMR_CMIB_INT_DISABLE | 
                    _00_TMR_OVI_INT_DISABLE;

    /* Set A/D trigger and output */
    TMR0.TCSR.BYTE = _00_TMR_AD_TRIGGER_DISABLE | _E0_TMR02_TCSR_DEFAULT;

    /* Set compare match value */ 
    TMR0.TCORA = _77_TMR0_COMP_MATCH_VALUE_A;
    TMR0.TCORB = _77_TMR0_COMP_MATCH_VALUE_B;

    R_Config_TMR0_Create_UserInit();
}

/***********************************************************************************************************************
* Function Name: R_Config_TMR0_Start
* Description  : This function starts the TMR0 channel
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

void R_Config_TMR0_Start(void)
{

    /* Start counting */
    TMR0.TCCR.BYTE = _08_TMR_CLK_SRC_PCLK | _00_TMR_PCLK_DIV_1;
}

/***********************************************************************************************************************
* Function Name: R_Config_TMR0_Stop
* Description  : This function stops the TMR0 channel
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

void R_Config_TMR0_Stop(void)
{

    /* Stop counting */ 
    TMR0.TCCR.BYTE = _00_TMR_CLK_DISABLED;
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
