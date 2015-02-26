/*
 * FreeModbus Libary: RT-Thread Port
 * Copyright (C) 2013 Armink <armink.ztl@gmail.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: porttimer_m.c,v 1.60 2013/08/13 15:07:05 Armink add Master Functions$
 */

/*  Platform includes --------------------------------*/
#include "main.h"
#include "port.h"
#include "stm32f4xx_it.h"

/*  Modbus includes ----------------------------------*/
#include "mb_m.h"
#include "mbport.h"

#if MB_MASTER_RTU_ENABLED > 0 || MB_MASTER_ASCII_ENABLED > 0
/*	static functions --------------------------------------------------------*/
/*	static variables --------------------------------------------------------*/
//uint32_t counter_val = 0;
/*	exported variables ------------------------------------------------------*/
extern TIM_HandleTypeDef TimHandle_4;
extern TIM_HandleTypeDef TimHandle_5;

/* ----------------------- Start implementation -----------------------------*/
BOOL
xMBMasterPortTimersInit( USHORT usTim1Timerout50us )
{
    return TRUE;
}


void
vMBMasterPortTimersT35Enable()
{
	HAL_TIM_Base_Stop_IT(&TimHandle_5);		// Disable Timer-5 and Reset Counter
	__HAL_TIM_SET_COUNTER(&TimHandle_5, 0);
	
	HAL_TIM_Base_Start_IT(&TimHandle_5);	// Enable Timer-5
}

void 
vMBMasterPortTimersConvertDelayEnable()
{
	HAL_TIM_Base_Stop_IT(&TimHandle_4);		// Disable Timer-4 and Reset Counter
	__HAL_TIM_SET_COUNTER(&TimHandle_4, 0);
	
	/* Set current timer mode, don't change it.*/
    vMBMasterSetCurTimerMode(MB_TMODE_CONVERT_DELAY);
	
	HAL_TIM_Base_Start_IT(&TimHandle_4);	// Enable Timer-4
}

void 
vMBMasterPortTimersRespondTimeoutEnable()
{
	HAL_TIM_Base_Stop_IT(&TimHandle_4);		// Disable Timer-4 and Reset Counter
	__HAL_TIM_SET_COUNTER(&TimHandle_4, 0);
	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	HAL_TIM_Base_Start_IT(&TimHandle_4);	// Enable Timer-4
}

void
vMBMasterPortTimersDisable(  )
{
	HAL_TIM_Base_Stop_IT(&TimHandle_4);		// Disable Timer-4 and Reset Counter
	HAL_TIM_Base_Stop_IT(&TimHandle_5);		// Disable Timer-5 and Reset Counter
}

/**
	* @brief  This function handles TIM interrupt request.
  * @param  None
  * @retval None
  */
void 
TIMx_4_IRQHandler(void)
{
	printf("Timer 4 elapsed...\n");
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
	HAL_TIM_IRQHandler(&TimHandle_4);
}

/**
  * @brief  This function handles TIM interrupt request.
  * @param  None
  * @retval None
  */
void 
TIMx_5_IRQHandler(void)
{
	printf("Timer 5 elapsed...\n");
	HAL_TIM_IRQHandler(&TimHandle_5);
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim: TIM handle
  * @retval None
  */
void 
HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//counter_val = __HAL_TIM_GET_COUNTER(&TimHandle_5);
	//printf("Counter at interrupt: %d\n", counter_val);
	pxMBMasterPortCBTimerExpired();
}

#endif
