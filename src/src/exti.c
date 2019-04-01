/*******************************************************************************
 * @file    exti.c
 * @author  Rashad Shubita
 * @email   shubitarashad@gmail.com
 * @date    08.02.2019
 *
 * @brief   EXTI configuration source file
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


#include "exti.h"
#include "stm32f10x.h"

void EXTI_Init_PB(void)
{

 /* --------------- EXTI initialization and connect the External line to the GPIO --------------- */
 /* Enable AFIO_EXTICR clock */
    RCC-> APB2ENR |= RCC_APB2ENR_AFIOEN;

 /* MAP the PA0 to EXTI0  */
    AFIO-> EXTICR[0] |= AFIO_EXTICR1_EXTI0_PA ;

 /* Rising trigger selection ( 1 = trigger enable) */
	EXTI-> RTSR |= EXTI_RTSR_TR0  ;

 /* Falling trigger selection ( 0 = trigger disable) */
	EXTI-> FTSR &= ~EXTI_FTSR_TR0  ;

 /* Interrupt Mask Register ( 1 = not masked [enable], 0=masked) */
    EXTI-> IMR |= EXTI_IMR_MR0 ;

 /* Enable event line */
    EXTI -> EMR |= EXTI_EMR_MR0;

 /* Clear pending if any */
	EXTI->PR |= EXTI_PR_PR0;

}
