/*******************************************************************************
 * @file    dma.h
 * @author  Rashad Shubita
 * @email   shubitarashad@gmail.com
 * @date    15.02.2019
 *
 * @brief   DMA configuration header file
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
#ifndef INC_DMA_H_
#define INC_DMA_H_

#include "stm32f10x.h"

/**
 * @brief   DMA1 Channel3 initialization function
 * @note    Used for data transfer between two memory buffers
 * @param   None
 * @retval  None
 */
void DMA1_Channel3_Init(void);

/**
 * @brief   Enable DMA
 * @note
 * @param   DMAx_Channely   where: x= 1 or 2 , y= 1 ..7 for DMA1 and 1 ..5 for DMA2
 * @retval  None
 */
void DMA_ChannelEnable(DMA_Channel_TypeDef *DMA_Channel,uint8_t DMA_Num, uint8_t Ch_Num);

/**
 * @brief   Disable DMA
 * @note
 * @param   DMAx_Channely   where: x= 1 or 2 , y= 1 ..7 for DMA1 and 1 ..5 for DMA2
 * @retval  None
 */
void DMA_ChannelDisable(DMA_Channel_TypeDef *DMA_Channel);


#endif /* INC_DMA_H_ */
