/*******************************************************************************
 * @file    nvic.h
 * @author  Rashad Shubita
 * @email   shubitarashad@gmail.com
 * @date    08.02.2019
 *
 * @brief   NVIC configuration header file
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


#ifndef INC_NVIC_H_
#define INC_NVIC_H_


/* Includes */
#include "stm32f10x.h"

/**
 * @brief   NVIC IRQs initialization function
 * @note
 * @param   None
 * @retval  None
 */
void NVIC_Init(void);


#endif /* INC_NVIC_H_ */
