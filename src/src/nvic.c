/*******************************************************************************
 * @file    nvic.c
 * @author  Rashad Shubita
 * @email   shubitarashad@gmail.com
 * @date    08.02.2019
 *
 * @brief   NVIC configuration source file
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
#include "nvic.h"


/**
 * @brief   NVIC IRQs initialization function
 * @note
 * @param   None
 * @retval  None
 */
void NVIC_Init(void)
{
	  /* Set priority group to 3
	   * bits[3:0] are the sub-priority,
	   * bits[7:4] are the pre-empt priority */

	  NVIC_SetPriorityGrouping(3);

	  /* Set priority levels */
	  //NVIC_SetPriority(EXTI0_IRQn, 1);
	  //NVIC_SetPriority(DMA1_Channel3_IRQn, 1);
	  //NVIC_SetPriority(DMA1_Channel4_IRQn, 1);
	  //NVIC_SetPriority(DMA1_Channel5_IRQn, 1);
	  NVIC_SetPriority(DMA1_Channel2_IRQn, 1);
	  NVIC_SetPriority(DMA1_Channel3_IRQn, 1);
	  //NVIC_SetPriority(USART1_IRQn,1);
	  NVIC_SetPriority(USART3_IRQn,1);

	  /* Enable interrupts at NVIC */
	  //NVIC_EnableIRQ(EXTI0_IRQn);
	  //NVIC_EnableIRQ(DMA1_Channel3_IRQn);
	  //NVIC_EnableIRQ(DMA1_Channel4_IRQn);
	  //NVIC_EnableIRQ(DMA1_Channel5_IRQn);
	  NVIC_EnableIRQ(DMA1_Channel2_IRQn);
	  NVIC_EnableIRQ(DMA1_Channel3_IRQn);
	  //NVIC_EnableIRQ(USART1_IRQn);
	  NVIC_EnableIRQ(USART3_IRQn);

}
