/*******************************************************************************
 * @file    gpio.h
 * @author  Rashad Shubita
 * @email   shubitarashad@gmail.com
 * @date    15.01.2019
 *
 * @brief   Some examples on how to use STM32 GPIOs
 *
 * @note    Example: calculate the bit band address of the Yellow LED
 *                   connected to PC13
 *
 *          General equation:
 *          bit_word_addr = bit_band_base + (byte_offset * 32) + (bit_number * 4)
 *          ->bit_word_addr : is the address of the word in the alias memory region that maps to the targeted bit.
 *          ->bit_band_base : is the starting address of the alias region
 *          ->byte_offset   : is the number of the byte in the bit-band region that contains the targeted bit
 *          peripheral_base_address = 0x40000000 (Fixed for all ARM Cortex-M3)
 *          bit_band_base(alias)    = 0x42000000 (Fixed for all ARM Cortex-M3)
 *          port_C_address          = 0x40011000 (From STM32F1 datasheet page 34)
 *          register_ODR_offset     = 0x0C       (From STM32F1 reference manual page 172)
 *
 *          byte_offset = (port_C_address + register_ODR_offset) - peripheral_base_address
 *                      = (0x40011000 + 0x0C) - 0x40000000
 *                      = 0x0001100C
 *
 *          bit_number  = 13 (Yellow LED connected to PC13)
 *
 *          bit_word_addr = 0x42000000 + (0x0001100C * 32) + (13 * 4)
 *                        = 0x42000000 + 0x220180 + 0x34
 *                        = 0x422201B4
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


#ifndef INC_GPIO_H_
#define INC_GPIO_H_

/* Includes */
#include "stm32f10x.h"
#include <stddef.h>

#define   GPIO_Pin_Set(GPIOx, GPIO_PIN)       ( GPIOx ->BSRR  = (1<<GPIO_PIN)  )
#define   GPIO_Pin_Reset1(GPIOx, GPIO_PIN)    ( GPIOx ->BRR   = (1<<GPIO_PIN)  )
#define   GPIO_Pin_Reset2(GPIOx, GPIO_PIN)    ( GPIOx ->BSRR  = (uint32_t)(1<<GPIO_PIN) << 16 )
#define   GPIO_Pin_Read(GPIOx, GPIO_PIN)      ( GPIOx ->IDR   & (1<<GPIO_PIN) )
#define   GPIO_Pin_Toggle(GPIOx, GPIO_PIN)    ( GPIOx ->ODR   ^= (1<<GPIO_PIN) )

// Convert SRAM address to alias region
#define BITBAND_SRAM(byte_addr,bit_number)  ((SRAM_BB_BASE + (byte_addr-SRAM_BASE)*32   + (bit_number*4)))

// Convert PERI address to alias region
#define BITBAND_PERI(byte_addr,bit_number)  ((PERIPH_BB_BASE + (byte_addr-PERIPH_BASE)*32   + (bit_number*4)))

/**
 * @brief   Define address of GPIOC
 */
//#define GPIOC_ODR (GPIOC_BASE+0x0C)

#define GPIOC_ODR     (GPIOC_BASE+offsetof(GPIO_TypeDef,ODR))


/**
 * @brief   Yellow LED connected to GPIOC bit 13
 */
#define Yellow_LED_BITBAND *((volatile uint32_t *)(BITBAND_PERI(GPIOC_ODR, 13)))


/**
 * @brief   bit-band for any bit in GPIO register example: GPIO_BITBAND(GPIOA_BASE,ODR,5)=1;
 */
#define GPIO_BITBAND(GPIO_BASE,structure_member,bit_number)    *((volatile uint32_t *)(BITBAND_PERI( (GPIO_BASE+offsetof(GPIO_TypeDef,structure_member)), bit_number)))

/**
 * @brief   LED initialization function
 * @note    Yellow_LED -> PC13
 * @param   None
 * @retval  None
 */
void GPIO_OnBoard_Init_LED();

/**
 * @brief   Push button initialization function
 * @note    Push button connected to PA0
 * @param   None
 * @retval  None
 */
void GPIO_Init_PB(void);

/**
 * @brief   USART2 GPIO initialization function
 * @note    PB10 -> USART1_TX, PB11 -> USART1_RX
 * @param   None
 * @retval  None
 */
void GPIO_USART2_Init(void);

#endif /* INC_GPIO_H_ */
