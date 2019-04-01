/*******************************************************************************
 * @file    itm.h
 * @author  Rashad Shubita
 * @email   shubitarashad@gmail.com
 * @date    20.01.2019
 *
 * @brief   Basic functionality of ITM
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
#ifndef __INC_ITM_H_
#define __INC_ITM_H_

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
 * @addtogroup itm
 * @{
 */

/**
 * @defgroup itm_exported_typedefs
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup itm_exported_defines
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup itm_exported_macros
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup itm_exported_constants
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup itm_exported_functions
 * @{
 */
/**
 * @brief   ITM_Printf
 * @note
 * @param   str, length
 * @retval  None
 */
void ITM_Printf(char *str, int length);

/**
 * @brief   SendChar
 * @note
 * @param   port, ch
 * @retval
 */
void ITM_SendChar_Port(uint8_t port, uint8_t ch);

/**
 * @brief   ITM_Printf_Port
 * @note
 * @param   port, str, length
 * @retval  None
 */
void ITM_Printf_Port(uint8_t port, char *str, int length);

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

#endif /*__INC_ITM_H_ */
