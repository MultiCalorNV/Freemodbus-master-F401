/**
  ******************************************************************************
  * @file    I2C/I2C_TwoBoards_ComDMA/Src/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    26-December-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"  

/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup I2C_TwoBoards_ComDMA
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Exported variables ---------------------------------------------------------*/
bool gui_Exec = false;
bool Touch_Flagged = false;
/* Private variables ---------------------------------------------------------*/
static volatile uint32_t touch_timer=0;
static volatile uint32_t msec_counter = 0;
static volatile uint32_t Timeout_Timer_0 = 0;

//extern UART_HandleTypeDef UartHandle;
//extern TIM_HandleTypeDef    TimHandle;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
  * @brief	Delay for the given milliseconds.
  * @param	None
  * @retval	None
  */
void delay_msec(uint32_t msec){
  uint32_t target;
  
  target = msec_counter + msec;
  while(msec_counter < target){
  	;
  }
}

/**
  * @brief	Count timeOuts.
  * @param	None
  * @retval	time
  */
uint32_t timeout_timer(void){
	uint32_t time, prev_time;
	
	time = Timeout_Timer_0;
	
	return time;
}

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	HAL_IncTick();
	
	Timeout_Timer_0++;
  
	//---------------------------
	// msec counter
	//---------------------------
	msec_counter++;
	switch(msec_counter % 10){
		case 0:
			gui_Exec = true;
			break;
			
		default:
			break;
	}

	//---------------------------
	// Touch-Timer
	//---------------------------
	touch_timer++;
	switch(touch_timer % 1){
		case 0:
			Touch_Flagged = true;
			break;
		
		default:
			Touch_Flagged = false;
			break;
	}

}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/
/**
  * @brief  This function handles DMA RX interrupt request.  
  * @param  None
  * @retval None   
  */
/*void USARTx_DMA_RX_IRQHandler(void)
{
  HAL_DMA_IRQHandler(UartHandle.hdmarx);
}*/

/**
  * @brief  This function handles DMA TX interrupt request.
  * @param  None
  * @retval None   
  */
/*void USARTx_DMA_TX_IRQHandler(void)
{
  HAL_DMA_IRQHandler(UartHandle.hdmatx);
}*/

/**
  * @brief  This function handles USARTx interrupt request.
  * @param  None
  * @retval None
  */
/*void USARTx_IRQHandler(void)
{
	uint8_t vRxBuffer_Usart[256];
	uint32_t tmp1 = 0, tmp2 = 0;
	uint8_t index = 0;
	UART_HandleTypeDef *huart;
	//printf("IRQ event\n");
	//HAL_UART_IRQHandler(&UartHandle);
	huart = &UartHandle;
	
	tmp1 = __HAL_UART_GET_FLAG(huart, UART_FLAG_PE);
	tmp2 = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_PE);  
	/* UART parity error interrupt occurred ------------------------------------
	if((tmp1 != RESET) && (tmp2 != RESET))
	{
		__HAL_UART_CLEAR_PEFLAG(huart);
		
		huart->ErrorCode |= HAL_UART_ERROR_PE;
	}
	
	tmp1 = __HAL_UART_GET_FLAG(huart, UART_FLAG_FE);
	tmp2 = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_ERR);
	/* UART frame error interrupt occurred -------------------------------------
	if((tmp1 != RESET) && (tmp2 != RESET))
	{
		__HAL_UART_CLEAR_FEFLAG(huart);

		huart->ErrorCode |= HAL_UART_ERROR_FE;
	}
	
	tmp1 = __HAL_UART_GET_FLAG(huart, UART_FLAG_NE);
	tmp2 = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_ERR);
	/* UART noise error interrupt occurred -------------------------------------
	if((tmp1 != RESET) && (tmp2 != RESET))
	{ 
		__HAL_UART_CLEAR_NEFLAG(huart);

		huart->ErrorCode |= HAL_UART_ERROR_NE;
	}
  
	tmp1 = __HAL_UART_GET_FLAG(huart, UART_FLAG_ORE);
	tmp2 = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_ERR);
	/* UART Over-Run interrupt occurred ----------------------------------------
	if((tmp1 != RESET) && (tmp2 != RESET))
	{ 
		__HAL_UART_CLEAR_OREFLAG(huart);

		huart->ErrorCode |= HAL_UART_ERROR_ORE;
	}
	
	tmp1 = __HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_RXNE);
	tmp2 = __HAL_UART_GET_IT_SOURCE(&UartHandle, UART_IT_RXNE);
	printf("Flag_RxNE : %s\n", tmp1 ? "SET" : "RESET");
	printf("IT_RxNE : %s\n", tmp2 ? "SET" : "RESET");
	/* UART in mode Receiver ---------------------------------------------------
	if((tmp1 != RESET) && (tmp2 != RESET))
	{
		//HAL_UART_RxCpltCallback(huart);

		vRxBuffer_Usart[index] = (uint8_t)(huart->Instance->DR & (uint8_t)0x00FF);
		printf("aRxBuffer[%d]: %d\n", index, vRxBuffer_Usart[index]);
	}
	
	if(huart->ErrorCode != HAL_UART_ERROR_NONE)
	{
		HAL_UART_ErrorCallback(huart);
		printf("Uart_ErrCode: %d\n", huart->ErrorCode);
	}
}*/

/**
  * @brief  This function handles TIM interrupt request.
  * @param  None
  * @retval None
  */
/*void TIMx_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&TimHandle);
}*/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
