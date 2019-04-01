/*******************************************************************************
 * @file    LIN_Slave.h
 * @author  Rashad Shubita
 * @email   shubitarashad@gmail.com
 * @date    23.01.2019
 *
 * @brief   LIN slave node driver
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

/* Define to prevent recursive inclusion */
#ifndef __INC_LIN_SLAVE_H_
#define __INC_LIN_SLAVE_H_

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

/* Includes */
#include "stm32f10x.h"

/**
 * @addtogroup stm32_examples
 * @{
 */

/**
 * @addtogroup LIN_Slave
 * @{
 */

/**
 * @defgroup LIN_Slave_exported_typedefs
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup LIN_Slave_exported_defines
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup LIN_Slave_exported_macros
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup LIN_Slave_exported_constants
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup LIN_Slave_exported_functions
 * @{
 */

void LIN_Slave_GPIO_Init(void);
void LIN_Slave_USART3_Init(void);
void LIN_Slave_TX_DMA_Init(void);
void LIN_Slave_RX_DMA_Init(void);
void LIN_Slave_Enable(void);
void LIN_Slave_RX_DMA_IRQ_Callback(void);
void LIN_Slave_TX_DMA_IRQ_Callback(void);
void LIN_Slave_USART3_IRQ_Callback(void);
void LIN_Slave_Process(void);
void LIN_Send_Break_Field(void);
void LIN_Master_Process(uint8_t data0, uint8_t data1, uint8_t CheckSum);

/**
 * @}
 */
/**
 * @}
 */
/**
 * @}
 */

/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif /*__INC_LIN_SLAVE_H_ */
