/**
  ******************************************************************************
  * @file    Templates/Src/stm32f4xx_hal_msp.c
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    26-December-2014
  * @brief   HAL MSP module.
  *         
  @verbatim
 ===============================================================================
                     ##### How to use this driver #####
 ===============================================================================
    [..]
    This file is generated automatically by MicroXplorer and eventually modified 
    by the user

  @endverbatim
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

/** @addtogroup STM32F4xx_HAL_Driver
  * @{
  */

/** @defgroup HAL_MSP
  * @brief HAL MSP module.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup HAL_MSP_Private_Functions
  * @{
  */

/**
  * @brief TIM MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  * @param htim: TIM handle pointer
  * @retval None
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* TIMx Peripheral clock enable */
  TIMx_4_CLK_ENABLE();
  TIMx_5_CLK_ENABLE();
  
  /*##-2- Configure the NVIC for TIMx ########################################*/
  /* Set Interrupt Group Priority */
  HAL_NVIC_SetPriority(TIMx_4_IRQn, 4, 0);
  HAL_NVIC_SetPriority(TIMx_5_IRQn, 4, 1);
  
  /* Enable the TIMx global Interrupt */
  HAL_NVIC_EnableIRQ(TIMx_4_IRQn);
  HAL_NVIC_EnableIRQ(TIMx_5_IRQn);
}
  
/**
  * @brief UART MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  *           - DMA configuration for transmission request by peripheral 
  *           - NVIC configuration for DMA interrupt request enable
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
  //static DMA_HandleTypeDef hdma_tx;
  //static DMA_HandleTypeDef hdma_rx;
  
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO TX/RX clock */
  USARTx_TX_GPIO_CLK_ENABLE();
  USARTx_RX_GPIO_CLK_ENABLE();
  /* Enable USART2 clock */
  USARTx_CLK_ENABLE(); 
  /* Enable DMA1 clock */
  //DMAx_CLK_ENABLE();   
  
  /*##-2- Configure peripheral GPIO ##########################################*/  
  /* UART TX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = USARTx_TX_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = USARTx_TX_AF;
  
  HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStruct);
    
  /* UART RX GPIO pin configuration  */
  GPIO_InitStruct.Pin = USARTx_RX_PIN;
  GPIO_InitStruct.Alternate = USARTx_RX_AF;
    
  HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStruct);
    
  /*##-3- Configure the DMA streams ##########################################*/
  /* Configure the DMA handler for Transmission process */
  //hdma_tx.Instance                 = USARTx_TX_DMA_STREAM;
  
  //hdma_tx.Init.Channel             = USARTx_TX_DMA_CHANNEL;
  //hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
  //hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
  //hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
  //hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  //hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  //hdma_tx.Init.Mode                = DMA_NORMAL;
  //hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;
  //hdma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
  //hdma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  //hdma_tx.Init.MemBurst            = DMA_MBURST_INC4;
  //hdma_tx.Init.PeriphBurst         = DMA_PBURST_INC4;
  
  //HAL_DMA_Init(&hdma_tx);   
  
  /* Associate the initialized DMA handle to the the UART handle */
  //__HAL_LINKDMA(huart, hdmatx, hdma_tx);
    
  /* Configure the DMA handler for Transmission process */
  //hdma_rx.Instance                 = USARTx_RX_DMA_STREAM;
  
  //hdma_rx.Init.Channel             = USARTx_RX_DMA_CHANNEL;
  //hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  //hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
  //hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
  //hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  //hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  //hdma_rx.Init.Mode                = DMA_NORMAL;
  //hdma_rx.Init.Priority            = DMA_PRIORITY_HIGH;
  //hdma_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;         
  //hdma_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  //hdma_rx.Init.MemBurst            = DMA_MBURST_INC4;
  //hdma_rx.Init.PeriphBurst         = DMA_PBURST_INC4; 

  //HAL_DMA_Init(&hdma_rx);
    
  /* Associate the initialized DMA handle to the the UART handle */
  //__HAL_LINKDMA(huart, hdmarx, hdma_rx);
    
  /*##-4- Configure the NVIC for DMA #########################################*/
  /* NVIC configuration for DMA transfer complete interrupt (USARTx_TX) */
  //HAL_NVIC_SetPriority(USARTx_DMA_TX_IRQn, 2, 1);
  //HAL_NVIC_EnableIRQ(USARTx_DMA_TX_IRQn);
    
  /* NVIC configuration for DMA transfer complete interrupt (USARTx_RX) */
  //HAL_NVIC_SetPriority(USARTx_DMA_RX_IRQn, 2, 0);   
  //HAL_NVIC_EnableIRQ(USARTx_DMA_RX_IRQn);
  
  /* NVIC configuration for USART TC interrupt */
  HAL_NVIC_SetPriority(USARTx_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(USARTx_IRQn);
}

/**
  * @brief UART MSP De-Initialization 
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO, DMA and NVIC configuration to their default state
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
  
  //static DMA_HandleTypeDef hdma_tx;
  //static DMA_HandleTypeDef hdma_rx;

  /*##-1- Reset peripherals ##################################################*/
  USARTx_FORCE_RESET();
  USARTx_RELEASE_RESET();

  /*##-2- Disable peripherals and GPIO Clocks #################################*/
  /* Configure UART Tx as alternate function  */
  HAL_GPIO_DeInit(USARTx_TX_GPIO_PORT, USARTx_TX_PIN);
  /* Configure UART Rx as alternate function  */
  HAL_GPIO_DeInit(USARTx_RX_GPIO_PORT, USARTx_RX_PIN);
   
  /*##-3- Disable the DMA Streams ############################################*/
  /* De-Initialize the DMA Stream associate to transmission process */
  //HAL_DMA_DeInit(&hdma_tx); 
  /* De-Initialize the DMA Stream associate to reception process */
  //HAL_DMA_DeInit(&hdma_rx);
  
  /*##-4- Disable the NVIC for DMA ###########################################*/
  //HAL_NVIC_DisableIRQ(USARTx_DMA_TX_IRQn);
  //HAL_NVIC_DisableIRQ(USARTx_DMA_RX_IRQn);
  
  /*##-3- Disable the NVIC for UART ##########################################*/
  HAL_NVIC_DisableIRQ(USARTx_IRQn);
}

/**
  * @brief I2C MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  *           - DMA configuration for transmission request by peripheral 
  *           - NVIC configuration for DMA interrupt request enable
  * @param hi2c: I2C handle pointer
  * @retval None
  */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c){
  
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /*##-1- Enable GPIO Clocks #################################################*/
  /* Enable GPIO TX/RX clock */
  I2Cx_SCL_GPIO_CLK_ENABLE();
  I2Cx_SDA_GPIO_CLK_ENABLE();
  /* PORTC-Reset pin bus enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();


  /*##-2- Configure peripheral GPIO ##########################################*/
  /* I2C TX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = I2Cx_SCL_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = I2Cx_SCL_AF;
  HAL_GPIO_Init(I2Cx_SCL_GPIO_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

  /* I2C RX GPIO pin configuration  */
  GPIO_InitStruct.Pin = I2Cx_SDA_PIN;
  GPIO_InitStruct.Alternate = I2Cx_SDA_AF;
  HAL_GPIO_Init(I2Cx_SDA_GPIO_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
  
  /* Init Reset Pin Touch */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
 
  /*##-3- Enable peripheral Clock ############################################*/
  /* Enable I2C1 clock */
  I2Cx_CLK_ENABLE();
  
}

/**
  * @brief I2C MSP De-Initialization 
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO, DMA and NVIC configuration to their default state
  * @param hi2c: I2C handle pointer
  * @retval None
  */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c){
  
  /*##-1- Reset peripherals ##################################################*/
  I2Cx_FORCE_RESET();
  I2Cx_RELEASE_RESET();

  /*##-2- Disable peripherals and GPIO Clocks ################################*/
  /* Configure I2C Tx as alternate function  */
  HAL_GPIO_DeInit(I2Cx_SCL_GPIO_PORT, I2Cx_SCL_PIN);
  /* Configure I2C Rx as alternate function  */
  HAL_GPIO_DeInit(I2Cx_SDA_GPIO_PORT, I2Cx_SDA_PIN);
  
}


/**
  * @brief SRAM init
  * param  handle to sram
  * retval None
  */
void HAL_SRAM_MspInit(SRAM_HandleTypeDef * hsram){
	// TODO -> PUT RESET ON GPIO PIN SO WE CAN RESET DISPLAY
	// now reset is on NRST pin on the STM32F4 discovery
	GPIO_InitTypeDef GPIO_InitStructure;

	// Clock Enable FSMC
	__HAL_RCC_FSMC_CLK_ENABLE();

	// Clock Enable Port-D and Port-E
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	//PORTD setup -> alternate function
	// PD0=FSMC_D2->DB2
	// PD1=FSMC_D3->DB3
	// PD4=FSMC_NOE->RD
	// PD5=FSMC_NWE->WR
	// PD7=FSMC_NE1->CS
	// PD8=FSMC_D13->DB15
	// PD9=FSMC_D14->DB16
	// PD10=FSMC_D15->DB17
	// PD11=FSMC_A16->RS
	// PD14=FSMC_D0->DB0
	// PD15=FSMC_D1->DB1
	GPIO_InitStructure.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_14 | GPIO_PIN_15;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Alternate = GPIO_AF12_FSMC;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

	//PORTE setup -> alternate function
	// PE7=FSMC_D4->DB4
	// PE8=FSMC_D5->DB5
	// PE9=FSMC_D6->DB6
	// PE10=FSMC_D7->DB7
	// PE11=FSMC_D8->DB10
	// PE12=FSMC_D9->DB11
	// PE13=FSMC_D10->DB12
	// PE14=FSMC_D11->DB13
	// PE15=FSMC_D12->DB14
	GPIO_InitStructure.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Alternate = GPIO_AF12_FSMC;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);

	/* Init Reset Pin Display */
	GPIO_InitStructure.Pin = GPIO_PIN_1;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
 
}
		
/**
  * @brief SRAM deinit
  * @param handle to sram
  * @retval None
  */
void HALL_SRAM_MspDeInit(SRAM_HandleTypeDef * hsram){
	// Disable peripheral and GPIO
	HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 |GPIO_PIN_11 | GPIO_PIN_14 | GPIO_PIN_15);
	HAL_GPIO_DeInit(GPIOE, GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
}


/**
  * @brief  Initializes the Global MSP.
  * @param  None
  * @retval None
  */
void HAL_MspInit(void)
{
  /* NOTE : This function is generated automatically by MicroXplorer and eventually  
            modified by the user
   */ 
}

/**
  * @brief  DeInitializes the Global MSP.
  * @param  None  
  * @retval None
  */
void HAL_MspDeInit(void)
{
  /* NOTE : This function is generated automatically by MicroXplorer and eventually  
            modified by the user
   */
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
