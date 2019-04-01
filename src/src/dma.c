/*******************************************************************************
 * @file    dma.c
 * @author  Rashad Shubita
 * @email   shubitarashad@gmail.com
 * @date    15.02.2019
 *
 * @brief   DMA configuration source file
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


#include "dma.h"

/**
 * @brief   DMA1 Channel3 initialization function
 * @note    Used for data transfer between two memory buffers
 * @param   None
 * @retval  None
 */
void DMA1_Channel3_Init(void)
{
  /* Enable clock for DMA1*/
  RCC ->AHBENR |= RCC_AHBENR_DMA1EN;

  /* disable channel, since this registers must not be written when the channel is enabled */
  DMA_ChannelDisable(DMA1_Channel3);

  /* Set Peripheral size 32-bits (10)*/
  DMA1_Channel3 ->CCR   &= ~DMA_CCR1_PSIZE;
  DMA1_Channel3 ->CCR   |= DMA_CCR1_PSIZE_1;

  /* Set Memory size 32-bits (10)*/
  DMA1_Channel3 ->CCR   &= ~DMA_CCR1_MSIZE;
  DMA1_Channel3 ->CCR   |= DMA_CCR1_MSIZE_1;

  /* Set Channel priority Very high (11)*/
  DMA1_Channel3 ->CCR   |= DMA_CCR1_PL;

  /* Enable Peripheral increment mode (1) */
  DMA1_Channel3 ->CCR   |= DMA_CCR1_PINC;

  /* Enable memory increment mode (1)*/
  DMA1_Channel3 ->CCR   |= DMA_CCR1_MINC;

  /* Enable M2M Mode (1) */
  DMA1_Channel3 ->CCR   |= DMA_CCR1_MEM2MEM;

  /* Data transfer direction Read from memory (1)*/
  DMA1_Channel3 ->CCR   |= DMA_CCR1_DIR;

  /* Enable Transfer complete interrupt */
  DMA1_Channel3 ->CCR   |= DMA_CCR1_TCIE;

	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
}


/*
 * @brief   DMA Channel set addresses function
 * @note    Sets the addresses of the memory and peripheral ports,
 *          and number of data items to be transfered in a specific channel
 * @param   DMA_Channel, Peripheral_Adr, Memory_Adr, size
 * @retval  None
 */
void DMA_Channel_Set_Addresses(DMA_Channel_TypeDef *DMA_Channel, const uint32_t * Peripheral_Adr,
                                uint32_t * Memory_Adr, const uint32_t size)
{
    /* Set address for peripheral */
	DMA_Channel ->CPAR   = (uint32_t)Peripheral_Adr;

    /* Set address for memory */
	DMA_Channel ->CMAR   = (uint32_t)Memory_Adr;

    /* Set no. of data to transfer */
	DMA_Channel ->CNDTR  = size;
}

/**
 * @brief   Enable DMA
 * @note
 * @param   DMAx_Channely   where: x= 1 or 2 , y= 1 ..7 for DMA1 and 1 ..5 for DMA2
 * @retval  None
 */
void DMA_ChannelEnable(DMA_Channel_TypeDef *DMA_Channel,uint8_t DMA_Num, uint8_t Ch_Num)
{
  /*Clears the GIF, TEIF, HTIF and TCIF flags in the DMA_ISR register */
	if(DMA_Num == 1)DMA1->IFCR |= (1 << (4*(Ch_Num-1)) );
	else DMA2->IFCR |= (1 << (4*(Ch_Num-1)) );
 /* Channel enable */
	DMA_Channel->CCR |=  DMA_CCR1_EN;
}

/**
 * @brief   Disable DMA
 * @note
 * @param   DMAx_Channely   where: x= 1 or 2 , y= 1 ..7 for DMA1 and 1 ..5 for DMA2
 * @retval  None
 */
void DMA_ChannelDisable(DMA_Channel_TypeDef *DMA_Channel){
  DMA_Channel->CCR &= ~DMA_CCR1_EN;
}
