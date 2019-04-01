/*******************************************************************************
 * @file    DS18B20.h
 * @author  Rashad Shubita
 * @email   shubitarashad@gmail.com
 * @date    20.01.2019
 *
 * @brief   Interfacing temperature sensor DS18B20 using UART over one-wire
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

#ifndef INC_DS18B20_H_
#define INC_DS18B20_H_

/* Includes */
#include "stm32f10x.h"

/**
 * @brief   Configure GPIO
 * @note    UART3_TX -> PB10, UART3_RX -> PB11 (Not Used) from "Datasheet rev17 page:30"
 *          GPIO configuration from "Reference manual [Table 24. USARTs]
 *          UART3 connected to APB1 with 36MHz max clock
 * @param   None
 * @retval  None
 */
void DS18B20_GPIO_Init(void);

/**
 * @brief   Configure DMA for USART TX
 * @note    USART3_TX -> DMA1_Channel2
 * @param   None
 * @retval  None
 */
void DS18B20_TX_DMA_Init(void);

/**
 * @brief   Configure DMA for UART RX
 * @note    USART3_RX -> DMA1_Channel3
 * @param   None
 * @retval  None
 */
void DS18B20_RX_DMA_Init(void);

/**
 * @brief   Configure USART3 for DS18B20
 * @note
 * @param   None
 * @retval  None
 */
void DS18B20_USART3_Init(void);

/**
 * @brief   Enable communications with DS18B20
 * @note
 * @param   None
 * @retval  None
 */
void DS18B20_USART3_Enable(void);

/**
 * @brief   DS18B20 process function
 * @note
 * @param   None
 * @retval  None
 */
void DS18B20_Process(void);

/**
 * @brief   IRQ callback function
 * @note
 * @param   None
 * @retval  None
 */
void USART3_TX_DMA_IRQ_Callback(void);

/**
 * @brief   IRQ callback function
 * @note
 * @param   None
 * @retval  None
 */
void USART3_RX_DMA_IRQ_Callback(void);


#endif /* INC_DS18B20_H_ */
