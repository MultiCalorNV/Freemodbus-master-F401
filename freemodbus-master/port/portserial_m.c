/*
 * FreeModbus Libary: stm32f4 Port
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
 * File: $Id: portserial_m.c,v 1.60 2013/08/13 15:07:05 Armink add Master Functions $
 */

#include "main.h"
#include "port.h"
#include "stm32f4xx_it.h"

/*  Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

#if MB_MASTER_RTU_ENABLED > 0 || MB_MASTER_ASCII_ENABLED > 0
/*	static functions --------------------------------------------------------*/
static void     prvvUARTTxReadyISR( void );
static void     prvvUARTRxISR( void );

/*	exported variables ------------------------------------------------------*/
extern UART_HandleTypeDef UartHandle;

/*	Private variables -------------------------------------------------------*/
/*static volatile*/ uint8_t aTxBuffer_Usart[]= "serial Port up!";
/*static volatile*/ uint8_t aRxBuffer_Usart[RXBUFFERSIZEUSART];

__IO ITStatus UartReady_Transmit = RESET;
__IO ITStatus UartReady_Receive = RESET;

/* ----------------------- Start implementation -----------------------------*/
void vMBMasterPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
    if( xRxEnable )
    {
		/* Enable the UART Data Register not empty Interrupt */
		__HAL_UART_ENABLE_IT(&UartHandle, UART_IT_RXNE);
		printf("Receive enabled\n");
    }
    else
    {
		/* Disable the UART Data Register not empty Interrupt */
		__HAL_UART_DISABLE_IT(&UartHandle, UART_IT_RXNE);
    }
    if( xTxEnable )
    {
		/*	Enable DE transmit signal	*/
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
		/* Enable the UART Transmit data register empty Interrupt */
		__HAL_UART_ENABLE_IT(&UartHandle, UART_IT_TXE);
        prvvUARTTxReadyISR();
    }
    else
    {
		/* Disable the UART Transmit data register empty Interrupt */
		__HAL_UART_DISABLE_IT(&UartHandle, UART_IT_TXE);
		/* Enable the UART Transmit Complete Interrupt */    
		__HAL_UART_ENABLE_IT(&UartHandle, UART_IT_TC);
    }
}

void
vMBMasterPortClose( void )
{
}

BOOL
xMBMasterPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
    BOOL            bInitialized = TRUE;
    //USHORT          cfg = 0;
    //ULONG           reload = ( ( PCLK / ulBaudRate ) / 16UL );
	//uint32_t Uart_Error;
    //volatile char   dummy;
	
	//printf("USART sending ok\n");
	
	/*if(HAL_UART_Transmit_IT(&UartHandle, (uint8_t*)aTxBuffer_Usart, TXBUFFERSIZEUSART)!= HAL_OK){
		printf("USART Transmit Error\n");
	}*/
	
	//Uart_Error = HAL_UART_GetError(&UartHandle);
	//printf("Uart_Error: %d\n", Uart_Error);

    return bInitialized;
}

BOOL
xMBMasterPortSerialPutByte(CHAR ucByte)
{
	UART_HandleTypeDef *huart;
	
	huart = &UartHandle;
	
	huart->Instance->DR = (uint8_t)ucByte;
    return TRUE;
}

BOOL
xMBMasterPortSerialGetByte(CHAR * pucByte)
{
	UART_HandleTypeDef *huart;
	
	huart = &UartHandle;
	
	*pucByte = (uint8_t)(huart->Instance->DR & (uint8_t)0x00FF);
	
    return TRUE;
}

/**
  * @brief  This function handles USARTx interrupt request.
  * @param  None
  * @retval None
  */
void USARTx_IRQHandler(void)
{
	uint32_t tmp1 = 0, tmp2 = 0;
	bool            xtNeedPoll;
	UART_HandleTypeDef *huart;

	huart = &UartHandle;
	
	tmp1 = __HAL_UART_GET_FLAG(huart, UART_FLAG_PE);
	tmp2 = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_PE);  
	/* UART parity error interrupt occurred ------------------------------------*/
	if((tmp1 != RESET) && (tmp2 != RESET))
	{
		__HAL_UART_CLEAR_PEFLAG(huart);
		
		huart->ErrorCode |= HAL_UART_ERROR_PE;
	}
	
	tmp1 = __HAL_UART_GET_FLAG(huart, UART_FLAG_FE);
	tmp2 = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_ERR);
	/* UART frame error interrupt occurred -------------------------------------*/
	if((tmp1 != RESET) && (tmp2 != RESET))
	{
		__HAL_UART_CLEAR_FEFLAG(huart);

		huart->ErrorCode |= HAL_UART_ERROR_FE;
	}
	
	tmp1 = __HAL_UART_GET_FLAG(huart, UART_FLAG_NE);
	tmp2 = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_ERR);
	/* UART noise error interrupt occurred -------------------------------------*/
	if((tmp1 != RESET) && (tmp2 != RESET))
	{ 
		__HAL_UART_CLEAR_NEFLAG(huart);

		huart->ErrorCode |= HAL_UART_ERROR_NE;
	}
  
	tmp1 = __HAL_UART_GET_FLAG(huart, UART_FLAG_ORE);
	tmp2 = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_ERR);
	/* UART Over-Run interrupt occurred ----------------------------------------*/
	if((tmp1 != RESET) && (tmp2 != RESET))
	{ 
		__HAL_UART_CLEAR_OREFLAG(huart);

		huart->ErrorCode |= HAL_UART_ERROR_ORE;
	}
	
	tmp1 = __HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_RXNE);
	tmp2 = __HAL_UART_GET_IT_SOURCE(&UartHandle, UART_IT_RXNE);
	/* UART in mode Receiver ---------------------------------------------------*/
	if((tmp1 != RESET) && (tmp2 != RESET))
	{
		pxMBMasterFrameCBByteReceived();
	}
	
	tmp1 = __HAL_UART_GET_FLAG(huart, UART_FLAG_TXE);
	tmp2 = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_TXE);
	/* UART in mode Transmitter ------------------------------------------------*/
	if((tmp1 != RESET) && (tmp2 != RESET))
	{
		xtNeedPoll = pxMBMasterFrameCBTransmitterEmpty();
	}
	
	tmp1 = __HAL_UART_GET_FLAG(huart, UART_FLAG_TC);
	tmp2 = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_TC);
	/* UART in mode Transmitter end --------------------------------------------*/
	if((tmp1 != RESET) && (tmp2 != RESET))
	{
		/*	Enable DE receive signal	*/
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
		/* Disable the UART Transmit Complete Interrupt */    
		__HAL_UART_DISABLE_IT(huart, UART_IT_TC);
	}
	
	if(huart->ErrorCode != HAL_UART_ERROR_NONE)
	{
		HAL_UART_ErrorCallback(huart);
		printf("Uart_ErrCode: %d\n", huart->ErrorCode);
	}
}

/* 
 * Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call 
 * xMBPortSerialPutByte( ) to send the character.
 */
static void
prvvUARTTxReadyISR( void )
{
    pxMBMasterFrameCBTransmitterEmpty();
}

/* 
 * Create an interrupt handler for the receive interrupt for your target
 * processor. This function should then call pxMBFrameCBByteReceived( ). The
 * protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
 * character.
 */
static void
prvvUARTRxISR( void )
{
    pxMBMasterFrameCBByteReceived(  );
}

/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle. 
  * @note   This example shows a simple way to report end of DMA Tx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle){
  /* Set transmission flag: transfer complete */
  UartReady_Transmit = SET;
  printf("Byte is send\n");
  
}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and 
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle){
  /* Set transmission flag: transfer complete */
  UartReady_Receive = SET;
  printf("Byte received\n");
  
  prvvUARTRxISR();
  
}

/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle){
  /* Transfer error in reception/transmission process */
	printf("USART bus Error\n"); 
	printf("Uart Error Code: %d\n", UartHandle->ErrorCode);
}

#endif
