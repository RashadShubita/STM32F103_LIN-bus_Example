/*******************************************************************************
 * @file    usart.h
 * @author  Rashad Shubita
 * @email   shubitarashad@gmail.com
 * @date    23.02.2019
 *
 * @brief   USART configuration header file
 * @note
 *
@verbatim
Copyright (C) 2019, Rashad Shubita

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or any later version.

This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program.  If not, see <http://www.gnu.org/licenses/>.
@endverbatim
*******************************************************************************/


#ifndef INC_USART_H_
#define INC_USART_H_

#include "stm32f10x.h"
#include "gpio.h"
#include <math.h>
#include "dma.h"

#define Yes             1
#define No              0


/**
 * @brief   USART1 GPIO initialization function
 * @note    PA9  -> USART1_TX
 *          PA10 -> USART1_RX
 *          PA11 -> USART1_CTS
 *          PA12 -> USART1_RTS
 *          "Table 5. Medium-density STM32F103xx pin definitions" in Datasheet
 *          "Table 24. USARTs" in Reference manual
 * @param   HFC if = 1 -> Init. CTS & RTS pins
 * @retval  None
 */
void USART1_GPIO_Init(uint8_t HFC);

/**
 * @brief   USART BRR value calculation
 * @note    F_CK Input clock to the peripheral(PCLK1[APB1] for USART2, 3, 4, 5 or PCLK2[APB2] for USART1) & always  over-sampling by 16
 * @param   Baud_Rate:    Desired Baud Rate value
 *          F_CK:         Input clock to the peripheral in Hz
 * @retval  Value of BRR
 */
uint16_t Cal_USART_BRR_Val(uint32_t Baud_Rate, uint32_t F_CK);

/**
 * @brief   USART initialization function
 * @note    None
 * @param   BRR_Val:     Can be calculated using Cal_USART_BRR_Val function
 * @retval  None
 */
void USART1_Init(uint16_t BRR_Val);

/**
 * @brief   Enable USART transmitter and receiver
 * @note
 * @param   USARTx ,where x=1 ..3
 * @retval  None
 */
void USART_Enable(USART_TypeDef *USARTx);


/**
 * @brief   String transmit
 * @note
 * @param   USARTX, str
 * @retval  None
 */
void USART_Send_String(USART_TypeDef *USARTx,const char *str);

/**
 * @brief   DMA1 Channel4 initialization function
 * @note    Used for data transfer between two memory buffers
 * @param   None
 * @retval  None
 */
void USART1_TX_DMA_Config(void);

/**
 * @brief   DMA1 Channel5 initialization function
 * @note    Used for data transfer between two memory buffers
 * @param   None
 * @retval  None
 */
void USART1_RX_DMA_Config(void);

/**
 * @brief   IRQ callback function
 * @note
 * @param   None
 * @retval  None
 */
void USART1_RX_DMA_IRQ_Callback(void);

/**
 * @brief   IRQ callback function
 * @note
 * @param   None
 * @retval  None
 */
void USART1_TX_DMA_IRQ_Callback(void);

/**
 * @brief   IRQ callback function
 * @note
 * @param   None
 * @retval  None
 */
void USART1_IRQ_Callback(void);

/**
 * @brief   USART1 transmit and receive data
 * @note
 * @param   None
 * @retval  None
 */
void USART1_Process(void);

#endif /* INC_USART_H_ */
