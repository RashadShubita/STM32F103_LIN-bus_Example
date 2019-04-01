/*******************************************************************************
 * @file    itm.c
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


/* Includes */
#include <stddef.h>
#include "itm.h"

/**
 * @addtogroup stm32_examples
 * @{
 */

/**
 * @defgroup itm
 * @brief
 * @{
 */

/**
 * @defgroup itm_private_typedefs
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup itm_private_defines
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup itm_private_macros
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup itm_private_constants
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup itm_private_variables
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup itm_private_function_prototypes
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup itm_private_functions
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
void ITM_Printf(char *str, int length)
{
  /* Check null pointers */
  if(NULL != str)
  {
    /* Start transmission to ITM port 0 */
    for(int idx = 0; idx < (length - 1); idx++)
    {
      /* Send a char */
      ITM_SendChar(*str);

      /* Increment to the next char */
      str++;
    }
  }
}

/**
 * @brief   SendChar
 * @note
 * @param   port, ch
 * @retval
 */
void ITM_SendChar_Port(uint8_t port, uint8_t ch)
{
  if ((ITM->TCR & ITM_TCR_ITMENA_Msk)                  &&      /* ITM enabled */
      (ITM->TER & (1UL << port)        )                    )     /* ITM Port #0 enabled */
  {
    while (ITM->PORT[port].u32 == 0);
    ITM->PORT[port].u8 = ch;
  }
}

/**
 * @brief   ITM_Printf_Port
 * @note
 * @param   port, str, length
 * @retval  None
 */
void ITM_Printf_Port(uint8_t port, char *str, int length)
{
  /* Check null pointers */
  if(NULL != str)
  {
    /* Start transmission to ITM port 0 */
    for(int idx = 0; idx < (length - 1); idx++)
    {
      /* Send a char */
      ITM_SendChar_Port(port, *str);

      /* Increment to the next char */
      str++;
    }
  }
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
