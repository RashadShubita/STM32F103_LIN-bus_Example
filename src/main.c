/*******************************************************************************
 * @file    main.c
 * @author  Rashad Shubita
 * @email   shubitarashad@gmail.com
 * @date    20.01.2019
 *
 * @brief   main application called after startup
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



/* Includes ------------------------------------------------------------------*/
#include "nvic.h"
#include "SysTick.h"
#include "gpio.h"
#include "LIN_Slave.h"
#include "itm.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/
int main(void)
{
	uint32_t x= 0;

	SysTick_Init();
    GPIO_OnBoard_Init_LED();
    NVIC_Init();

    LIN_Slave_GPIO_Init();
    LIN_Slave_USART3_Init();
    LIN_Slave_TX_DMA_Init();
    LIN_Slave_RX_DMA_Init();

    /* Clear PRIMASK, enable IRQs */
    __enable_irq();
    /* Send 10 to ITM port 2 */

    LIN_Slave_Enable();
    SysTick_Delay(400);

 /* Infinite loop */
 while(1)
 {
	 LIN_Master_Process(0x1A, 0x1B, 0xCA);
	 SysTick_Delay(200);
	 LIN_Master_Process(0x0A, 0x0B, 0xEA);
	 SysTick_Delay(200);
 }

}


