/*******************************************************************************
 * @file    LIN_Slave.c
 * @author  Rashad Shubita
 * @email   shubitarashad@gmail.com
 * @date    23.01.2019
 *
 * @brief   LIN slave node driver
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
#include "gpio.h"
#include "LIN_Slave.h"
#include "usart.h"
#include "itm.h"

/* Private typedef -----------------------------------------------------------*/

/**
 * @brief   LIN slave states definition
 */
typedef enum
{
  LINS_IDLE,
  LINS_BREAK_RECEIVED,
  LINS_WAIT_FOR_HEADER,
  LINS_HEADER_RECEIVED,
  LINS_RX_DATA,
  LINS_WAIT_FOR_RX_DATA,
  LINS_RX_DATA_RECEIVED,
  LINS_TX_DATA
} LIN_Slave_StateType;

/**
 * @brief   LIN slave error status definition
 */
typedef enum
{
  LINS_NO_ERROR,
  LINS_SYNC_ERROR,
  LINS_CHECKSUM_INVALID_ERROR
} LIN_Slave_ErrorStatusType;

/* Private define ------------------------------------------------------------*/

/**
 * @brief   LIN Header Length
 */
#define LIN_HEADER_LENGTH                    ((uint8_t) 2u)

/**
 * @brief   LIN data length including the checksum
 */
#define LIN_DATA_LENGTH                      ((uint8_t) 3u)

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/**
 * @brief   LIN slave current state
 */
static LIN_Slave_StateType currentState = LINS_IDLE;

/**
 * @brief   LIN slave current error status
 */
static LIN_Slave_ErrorStatusType currentErrorStatus = LINS_NO_ERROR;

/**
 * @brief   LIN Header, 2 bytes
 */
static uint8_t linHeader[LIN_HEADER_LENGTH];

/**
 * @brief   LIN Header For TX, 2 bytes
 */
static uint8_t linHeader_TX[LIN_HEADER_LENGTH];

/**
 * @brief   LIN Data, 3 bytes
 */
static uint8_t linData[LIN_DATA_LENGTH];

/**
 * @brief   DMA received flag
 */
static uint8_t RxDMAReceived = 0;

/* Private function prototypes -----------------------------------------------*/

/**
 * @brief   DMA receive
 * @note
 * @param   data, size
 * @retval  None
 */
static void DMAReceive(const uint8_t * data, uint8_t size);

/**
 * @}
 */

/**
 * @defgroup LIN_Slave_private_functions
 * @{
 */

/**
 * @brief   DMA receive
 * @note
 * @param   data, size
 * @retval  None
 */
static void DMAReceive(const uint8_t * data, uint8_t size)
{
  /* Check null pointers */
  if(NULL != data)
  {
	  /* disable channel, since this registers must not be written when the channel is enabled */
	 	 if(DMA_CCR1_EN == (DMA_CCR1_EN & DMA1_Channel3->CCR))
	 	  {
	 	    /* DMA 1 channel 3 is enabled, shall be disabled first */
	 		  DMA_ChannelDisable(DMA1_Channel3);

	 	    /* Wait until EN bit is cleared */
	 	  while(DMA_CCR1_EN == (DMA_CCR1_EN & DMA1_Channel3->CCR))
	 	    {
	 	      /* Do nothing until EN bit is cleared */
	 	    }
	 	  }
	 	  else
	 	  {
	 	    /* Do nothing, channel 3 is not enabled */
	 	  }

		 /* Set address for memory */
		  DMA1_Channel3 ->CMAR   = (uint32_t)data;

		 /* Set number of data items */
		  DMA1_Channel3 ->CNDTR  =  size;

		 /* Clear transfer complete flag */
	     DMA1->IFCR |= DMA_IFCR_CTCIF3;

		 /* Clears the GIF, TEIF, HTIF and TCIF flags in the DMA_ISR register */
		 DMA1->IFCR |= DMA_IFCR_CGIF3;

	    /* Clear any USART pending DMA requests */
	    USART3->CR3 &= ~USART_CR3_DMAR;

	    /* Enable DMA mode for reception */
	    USART3->CR3 |= USART_CR3_DMAR;
	     /* Finally, I find  the error after 2 days of debugging xD */
	    int temp = USART3->DR;

	    /* Enable DMA 1 channel 3 */
	    DMA1_Channel3->CCR |=  DMA_CCR1_EN;

  }
  else
  {
    /* Null pointers, do nothing */
  }
}

/**
 * @brief   DMA transmit
 * @note
 * @param   data, size
 * @retval  None
 */
static void DMATransmit(const uint8_t * data, uint8_t size)
{
	 /* Check null pointers */
	  if(NULL != data)
	  {
		  /* disable channel, since this registers must not be written when the channel is enabled */
		 	 if(DMA_CCR1_EN == (DMA_CCR1_EN & DMA1_Channel2->CCR))
		 	  {
		 	    /* DMA 1 channel 2 is enabled, shall be disabled first */
		 		DMA1_Channel2->CCR &= ~DMA_CCR1_EN;

		 	    /* Wait until EN bit is cleared */
		 	  while(DMA_CCR1_EN == (DMA_CCR1_EN & DMA1_Channel2->CCR))
		 	    {
		 	      /* Do nothing until EN bit is cleared */
		 	    }
		 	  }
		 	  else
		 	  {
		 	    /* Do nothing, channel 2 is not enabled */
		 	  }

		 /* Set address for memory */
		  DMA1_Channel2 ->CMAR   = (uint32_t)data;

		 /* Set number of data items */
		  DMA1_Channel2 ->CNDTR  =  size;

		 /* Clear transfer complete flag */
	     DMA1->IFCR |= DMA_IFCR_CTCIF2;

		 /* Clears the GIF, TEIF, HTIF and TCIF flags in the DMA_ISR register */
		 DMA1->IFCR |= DMA_IFCR_CGIF2;

	    /* Clear any USART pending DMA requests */
	    USART3->CR3 &= ~USART_CR3_DMAT;

	    /* Enable DMA mode for transmitter */
	    USART3->CR3 |= USART_CR3_DMAT;

	    while(USART_SR_TC != (USART_SR_TC & USART3->SR)){}

	    /* */
	    USART3->SR &= ~USART_SR_TC;
	    NVIC_ClearPendingIRQ(DMA1_Channel2_IRQn);
	    /* Enable DMA 1 channel 2 */
	    DMA1_Channel2->CCR |=  DMA_CCR1_EN;
	  }
	  else
	  {
	    /* Null pointers, do nothing */
	  }

}

/**
 * @}
 */

/**
 * @defgroup LIN_Slave_exported_functions
 * @{
 */

/**
 * @brief   Configure GPIO
 * @note    UART3_TX -> PB10, UART3_RX -> PB11 from "Datasheet rev17 page:30"
 *          GPIO configuration from "Reference manual [Table 24. USARTs]
 *          UART3 connected to APB1 with 36MHz max clock
 * @param   None
 * @retval  None
 */
void LIN_Slave_GPIO_Init(void)
{
	 /* GPIOB clock enable */
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

/**
 * @brief   Configure UART5
 * @note
 * @param   None
 * @retval  None
 */
void LIN_Slave_USART3_Init(void)
{
	  /* Enable USART3 clock */
	  RCC-> APB1ENR |= RCC_APB1ENR_USART3EN;

	  /* select 1 Start bit, 8 Data bits, n Stop bit  */
	  USART3 ->CR1    &= ~USART_CR1_M;

	  /* STOP bits, 00: 1 Stop bit */
	  USART3->CR2    &= ~USART_CR2_STOP;

      /* Select LIN mode */
	  USART3->CR2 |= USART_CR2_LINEN;

      /* Select LIN break detection length 11 bits */
	  USART3->CR2 |= USART_CR2_LBDL;

      /* Enable LIN break detection interrupt */
	  USART3->CR2 |= USART_CR2_LBDIE;

     /* Set baud rate = 9600 Bps */
	  USART3->BRR =  Cal_USART_BRR_Val(9600,8000000);
}

/**
 * @brief   Configure DMA for USART TX
 * @note    USART3_TX -> DMA1_Channel2
 * @param   None
 * @retval  None
 */
void LIN_Slave_TX_DMA_Init(void)
{
	  /* Enable clock for DMA1*/
	  RCC ->AHBENR |= RCC_AHBENR_DMA1EN;

	  /* disable channel, since this registers must not be written when the channel is enabled */
	 	 if(DMA_CCR1_EN == (DMA_CCR1_EN & DMA1_Channel2->CCR))
	 	  {
	 	    /* DMA 1 channel 2 is enabled, shall be disabled first */
	 		DMA1_Channel2->CCR &= ~DMA_CCR1_EN;

	 	    /* Wait until EN bit is cleared */
	 	  while(DMA_CCR1_EN == (DMA_CCR1_EN & DMA1_Channel2->CCR))
	 	    {
	 	      /* Do nothing until EN bit is cleared */
	 	    }
	 	  }
	 	  else
	 	  {
	 	    /* Do nothing, channel 2 is not enabled */
	 	  }

	  /* Set Peripheral size 8-bits (00)*/
	 	DMA1_Channel2 ->CCR   &= ~DMA_CCR1_PSIZE;

	  /* Set Memory size 8-bits (00)*/
	 	DMA1_Channel2 ->CCR   &= ~DMA_CCR1_MSIZE;

	  /* Set Channel priority Very high (11)*/
	 	DMA1_Channel2 ->CCR   |= DMA_CCR1_PL;

	  /* Disable Peripheral increment mode (0) */
	 	DMA1_Channel2 ->CCR   &= ~DMA_CCR1_PINC;

	  /* Enable memory increment mode (1)*/
	 	DMA1_Channel2 ->CCR   |= DMA_CCR1_MINC;

	  /* Disable Circular mode (0)*/
	 	DMA1_Channel2 ->CCR   &= ~DMA_CCR1_CIRC;

	  /* Diable M2M Mode (0) */
	 	DMA1_Channel2 ->CCR   &= ~DMA_CCR1_MEM2MEM;

	  /* Data transfer direction Read from memory(memory-to-peripheral) (1)*/
	 	DMA1_Channel2 ->CCR   |= DMA_CCR1_DIR;

	  /* Enable Transfer complete interrupt */
	 	DMA1_Channel2 ->CCR   |= DMA_CCR1_TCIE;

	  /* Set address for peripheral */
	 	DMA1_Channel2 ->CPAR   = (uint32_t)&USART3->DR;

		__ASM("NOP");
		__ASM("NOP");
		__ASM("NOP");
		__ASM("NOP");

}

/**
 * @brief   Configure DMA for UART RX
 * @note    USART3_RX -> DMA1_Channel3
 * @param   None
 * @retval  None
 */
void LIN_Slave_RX_DMA_Init(void)
{
	 /* Enable clock for DMA1*/
	  RCC ->AHBENR |= RCC_AHBENR_DMA1EN;

	  /* disable channel, since this registers must not be written when the channel is enabled */
	 	 if(DMA_CCR1_EN == (DMA_CCR1_EN & DMA1_Channel3->CCR))
	 	  {
	 	    /* DMA 1 channel 3 is enabled, shall be disabled first */
	 		  DMA_ChannelDisable(DMA1_Channel3);

	 	    /* Wait until EN bit is cleared */
	 	  while(DMA_CCR1_EN == (DMA_CCR1_EN & DMA1_Channel3->CCR))
	 	    {
	 	      /* Do nothing until EN bit is cleared */
	 	    }
	 	  }
	 	  else
	 	  {
	 	    /* Do nothing, channel 3 is not enabled */
	 	  }

	  /* Set Peripheral size 8-bits (00)*/
	 	DMA1_Channel3 ->CCR   &= ~DMA_CCR1_PSIZE;

	  /* Set Memory size 8-bits (00)*/
	 	DMA1_Channel3 ->CCR   &= ~DMA_CCR1_MSIZE;

	  /* Set Channel priority Very high (11)*/
	 	DMA1_Channel3 ->CCR   |= DMA_CCR1_PL;

	  /* Disable Peripheral increment mode (0) */
	 	DMA1_Channel3 ->CCR   &= ~DMA_CCR1_PINC;

	  /* Enable memory increment mode (1)*/
	 	DMA1_Channel3 ->CCR   |= DMA_CCR1_MINC;

	  /* Disable Circular mode (0)*/
	 	DMA1_Channel3 ->CCR   &= ~DMA_CCR1_CIRC;

	  /* Diable M2M Mode (0) */
	 	DMA1_Channel3 ->CCR   &= ~DMA_CCR1_MEM2MEM;

	  /* Data transfer direction Read from peripheral (0)*/
	 	DMA1_Channel3 ->CCR   &= ~DMA_CCR1_DIR;

	  /* Enable Transfer complete interrupt */
	 	DMA1_Channel3 ->CCR   |= DMA_CCR1_TCIE;

	    /* Set address for peripheral */
	 	DMA1_Channel3 ->CPAR   = (uint32_t)&USART3->DR;

		__ASM("NOP");
		__ASM("NOP");
		__ASM("NOP");
		__ASM("NOP");

}

/**
 * @brief   Enable LIN slave node communications
 * @note
 * @param   None
 * @retval  None
 */
void LIN_Slave_Enable(void)
{
	  /* Enable USART3 */
	  USART3->CR1 |= USART_CR1_UE;

	  /* Enable transmitter */
	  USART3->CR1 |= USART_CR1_TE;

	  /* Enable receiver */
	  USART3->CR1 |= USART_CR1_RE;
}

/**
 * @brief   IRQ callback function
 * @note
 * @param   None
 * @retval  None
 */
void LIN_Slave_RX_DMA_IRQ_Callback(void)
{
    //ITM_SendChar_Port(2, 60);
	 /* Check transfer complete flag */
	if((DMA1->ISR & DMA_ISR_TCIF3) == DMA_ISR_TCIF3)
	{
		/* DMA transfer is complete */

	    /* Clear transfer complete flag */
		DMA1->IFCR |= DMA_IFCR_CTCIF3;

		 /* Clears the GIF, TEIF, HTIF and TCIF flags in the DMA_ISR register */
	   	DMA1->IFCR |= DMA_IFCR_CGIF3;
	    /* Set transfer complete flag */
	   	RxDMAReceived = 1;

	}

	else
	 {
	    /* Do nothing, this interrupt is not handled */

	 }


}

/**
 * @brief   IRQ callback function
 * @note
 * @param   None
 * @retval  None
 */
void LIN_Slave_TX_DMA_IRQ_Callback(void)
{
    //ITM_SendChar_Port(2, 55);
	 /* Check transfer complete flag */
	if((DMA1->ISR & DMA_ISR_TCIF2) == DMA_ISR_TCIF2)
	{
		/* DMA transfer is complete */

	    /* Clear transfer complete flag */
		DMA1->IFCR |= DMA_IFCR_CTCIF2;

		 /* Clears the GIF, TEIF, HTIF and TCIF flags in the DMA_ISR register */
	   	DMA1->IFCR |= DMA_IFCR_CGIF2;
	}

	else
	 {
	    /* Do nothing, this interrupt is not handled */
	 }

}

/**
 * @brief   IRQ callback function
 * @note
 * @param   None
 * @retval  None
 */
void LIN_Slave_USART3_IRQ_Callback(void)
{
    //ITM_SendChar_Port(2, 50);
  /* Check if LIN break detected */
  if((USART3->SR & USART_SR_LBD) == USART_SR_LBD)
  {
    /* Clear LIN break detection flag */
	  USART3->SR &= ~(USART_SR_LBD);

    /* Clear read data register not empty */
	  USART3->SR &= ~(USART_SR_RXNE);

    /* Set LIN break detected */
    currentState = LINS_BREAK_RECEIVED;
  }
  else
  {
    /* No new LIN Frame received */
  }

}

/**
 * @brief   LIN slave node task
 * @note    This function shall be called periodically.
 * @param   None
 * @retval  None
 */
void LIN_Slave_Process(void)
{
  /* Check current LIN slave state */
  switch (currentState)
  {
    case LINS_IDLE:
      /* Wait for LIN break */
      break;

    case LINS_BREAK_RECEIVED:
      /* Start DMA for LIN header reception */
      //DMAReceive(linHeader, LIN_HEADER_LENGTH);

      /* Wait for LIN header */
      currentState = LINS_WAIT_FOR_HEADER;
      break;

    case LINS_WAIT_FOR_HEADER:
      /* Wait for LIN header */
      if(1 == RxDMAReceived)
      {
        /* New DMA data was received, reset DMA flag */
        RxDMAReceived = 0;

        /* Go to next state */
        currentState = LINS_HEADER_RECEIVED;
      }
      else
      {
        /* Do nothing */
      }
      break;

    case LINS_HEADER_RECEIVED:
      /* Check sync field */
      if(0x55 == linHeader[0])
      {
        /* Sync is OK, Check PID */
        /* We expecting only one frame with PID = 0xEC */
        if(0xEC == linHeader[1])
        {
          /* PID is OK, go to TX data */
          currentState = LINS_TX_DATA;
        }
        else
        {
          /* PID is unknown, go to idle */
          currentState = LINS_IDLE;
        }
      }
      else
      {
        /* Sync field not OK, go to idle */
        currentErrorStatus = LINS_SYNC_ERROR;
        currentState = LINS_IDLE;
      }
      break;

    case LINS_TX_DATA:
       	/* Start sending the Response */
    	DMATransmit(linData, LIN_DATA_LENGTH );
    	currentState = LINS_IDLE;

    	break;

    case LINS_RX_DATA:
      /* Start DMA for LIN header reception */
      DMAReceive(linData, LIN_DATA_LENGTH);

      /* Wait for LIN Data reception */
      currentState = LINS_WAIT_FOR_RX_DATA;
      break;

    case LINS_WAIT_FOR_RX_DATA:
      /* Wait for LIN data */
      if(1 == RxDMAReceived)
      {
        /* New DMA data was received, reset DMA flag */
        RxDMAReceived = 0;

        /* Go to next state */
        currentState = LINS_RX_DATA_RECEIVED;
      }
      else
      {
        /* Do nothing */
      }
      break;

    case LINS_RX_DATA_RECEIVED:
      /* Check the received checksum, shall be calculated based
       * on the received data */
      if((0xCA == linData[2]) || (0xEA == linData[2]))
      {
        /* Checksum is OK, check required command for the green LED */
        if(0x0A == linData[0])
        {
          /* Turn off green LED */

        }
        else if(0x1A == linData[0])
        {
          /* Turn on green LED */

        }
        else
        {
          /* Do nothing */
        }

        /* Command for the red LED */
        if(0x0B == linData[1])
        {
          /* Turn off red LED */

        }
        else if(0x1B == linData[1])
        {
          /* Turn on red LED */

        }
        else
        {
          /* Do nothing */
        }

        /* Reset error state and go to idle */
        currentState = LINS_IDLE;
        currentErrorStatus = LINS_NO_ERROR;
      }
      else
      {
        /* Checksum error, go to idle */
        currentState = LINS_IDLE;
        currentErrorStatus = LINS_CHECKSUM_INVALID_ERROR;
      }
      break;

    default:
      break;

  }

}
void LIN_Send_Break_Field(void)
{
	/* Send break character */
	USART3->CR1 |= USART_CR1_SBK;
}
void LIN_Master_Process(uint8_t data0, uint8_t data1, uint8_t CheckSum)
{
	/* Set Sync field */
	linHeader_TX[0]=0x55;

	/* Set PID = 0xEC as Master PID for Transmission */
	linHeader_TX[1]=0xEC;

	/* Set Response to transmit */
	linData[0] = data0;
	linData[1] = data1;
	linData[2] = CheckSum;

	/* Rest Header Frame */
	linHeader[0] = 0;
	linHeader[1] = 0;

	LIN_Send_Break_Field();

	/* Wait for Break Field to be sent */
	while(currentState != LINS_BREAK_RECEIVED){}

	DMAReceive(linHeader, LIN_HEADER_LENGTH);
	DMATransmit(linHeader_TX, LIN_HEADER_LENGTH);

    do{
    	LIN_Slave_Process();
    }while(currentState != LINS_IDLE);


}



