/*******************************************************************************
 * @file    gpio.c
 * @author  Rashad Shubita
 * @email   shubitarashad@gmail.com
 * @date    15.01.2019
 *
 * @brief   Some examples on how to use STM32 GPIOs
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
#include "gpio.h"

/**
 * @brief   LED initialization function
 * @note    Yellow_LED -> PC13
 * @param   None
 * @retval  None
 */
void GPIO_OnBoard_Init_LED(void)
{
 /* Enable clock for GPIOC */
    RCC ->APB2ENR |= RCC_APB2ENR_IOPCEN;
 /* Configure PC.13 in output mode, max speed 2 MHz. */
    GPIOC ->CRH &= ~GPIO_CRH_MODE13;
    GPIOC ->CRH |=  GPIO_CRH_MODE13_1;
 /* Configure PC.13 as  general purpose output push-pull */
    GPIOC ->CRH  &= ~GPIO_CRH_CNF13;
 /* Led OFF */
    GPIOC ->ODR  |= GPIO_ODR_ODR13;

}

/**
 * @brief   Push button initialization function
 * @note    Push button connected to PA0
 * @param   None
 * @retval  None
 */
void GPIO_Init_PB(void)
{
	RCC -> APB2ENR |=  RCC_APB2ENR_IOPAEN;  //Enable Clock for GPIOA
	GPIOA -> CRL   &= ~GPIO_CRL_MODE0;      //Input mode (reset state)
	GPIOA -> CRL   &= ~GPIO_CRL_CNF0_0;     //Input with pull-up / pull-down
	GPIOA -> CRL   |=  GPIO_CRL_CNF0_1;     //
	GPIOA ->ODR    &= ~GPIO_ODR_ODR0;       //active pull-down resistor
}



/**
 * @brief   USART2 GPIO initialization function
 * @note    PB10 -> USART1_TX, PB11 -> USART1_RX
 * @param   None
 * @retval  None
 */
void GPIO_USART2_Init(void)
{
 /* GPIOA clock enable */
  	RCC ->APB2ENR   |= RCC_APB2ENR_IOPBEN;

 /* PB10 TX: Output mode, max speed 2 MHz. */
	GPIOB ->CRH     &= ~GPIO_CRH_MODE10;
	GPIOB ->CRH     |=  GPIO_CRH_MODE10_1;

 /* PB10 TX: Alternate function output Push-pull */
	GPIOB ->CRH     &= ~GPIO_CRH_CNF10;
	GPIOB ->CRH     |=  GPIO_CRH_CNF10_1;

 /* PB11 RX: Floating input */
	GPIOB ->CRH     &= ~GPIO_CRH_CNF11;
	GPIOB ->CRH     |=  GPIO_CRH_CNF11_0;

 /* PB11 RX: Input mode */
	GPIOB ->CRH     &= ~GPIO_CRH_MODE11;

}

