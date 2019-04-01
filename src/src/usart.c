/*******************************************************************************
 * @file    usart.c
 * @author  Rashad Shubita
 * @email   shubitarashad@gmail.com
 * @date    15.01.2019
 *
 * @brief   USART configuration source file
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

#include "usart.h"
#include <stddef.h>

/* Private typedef -----------------------------------------------------------*/
/**
 * @brief   USART1 states definition
 */
typedef enum
{
  USART1_IDLE,
  USART1_WAIT_FOR_RESPONCE,
  USART1_ASK_FOR_NAME,
  USART1_WAIT_FOR_NAME,
  USART1_WAIT_FOR_COMMAND,
} USART1_StateType;

/**
 * @brief   USART1 error status definition
 */
typedef enum
{
  USART1_NO_ERROR,
  USART1_PARITY_ERROR
} USART1_ErrorStatusType;

/**
 * @brief   Return type
 */
typedef enum
{
  STR_NOT_EQUAL,
  STR_EQUAL
} strCmpReturnType;

/* Private define ------------------------------------------------------------*/
/**
 * @brief   Maximum USART reception buffer length
 */
#define MAX_BUFFER_LENGTH                     ((uint32_t) 200u)

/* Private variables constants -----------------------------------------------*/
/**
 * @brief   USART1 messages to be transmitted
 */
static const char hello_world[]        = "Hello World!";
static const char ask_for_name[]       = "What is your name?";
static const char hi[]                 = "Hi,";
static const char ask_for_command[]    = "Please, send command";
static const char ask_for_command_ex[] = "Action[turn_on / turn_off] Led[green_led / red_led]";
static const char turn_on_green_led[]  = "turn_on green_led";
static const char turn_on_red_led[]    = "turn_on red_led";
static const char turn_off_green_led[] = "turn_off green_led";
static const char turn_off_red_led[]   = "turn_off red_led";
static const char done[]               = "Done";
static const char wrong_command[]      = "Wrong Command";
static const char parity_error[]       = "Parity Error";

/* Private variables ---------------------------------------------------------*/
/**
 * @brief   USART1 current state
 */
static USART1_StateType currentState = USART1_IDLE;

/**
 * @brief   USART1 current error status
 */
static USART1_ErrorStatusType currentErrorStatus = USART1_NO_ERROR;


/**
 * @brief   USART1 RX message buffer
 */
static char RxBuffer[MAX_BUFFER_LENGTH ];
char RxDMABuffer[MAX_BUFFER_LENGTH];

/**
 * @brief   USART1 message length
 */
static uint8_t RxMessageLength = 0;


/**
 * @brief   Compare two strings
 * @note    take the size of the predefined string
 * @param   str1, str2, size
 * @retval  strCmpReturnType
 */
static strCmpReturnType strCmp(const char * str1, const char * str2,
    const uint8_t size)
{
  /* Compare status */
  strCmpReturnType cmpStatus = STR_EQUAL;

  /* Check null pointers */
  if((NULL != str1) && (NULL != str2))
  {
    /* Start comparing */
    for (int idx = 0; idx < size; idx++)
    {
      /* When not equal set the return status */
      if(str1[idx] != str2[idx])
      {
        cmpStatus = STR_NOT_EQUAL;
      }
      else
      {
        /* Do nothing */
      }
    }
  }
  else
  {
    /* Null pointers, do nothing */
  }
  return cmpStatus;
}

/**
 * @brief   IRQ callback function
 * @note
 * @param   None
 * @retval  None
 */
void USART1_IRQ_Callback(void)
{
  /* Check if parity error detected */
  if((USART1->SR & USART_SR_PE) == USART_SR_PE)
  {
    while((USART1->SR & USART_SR_RXNE) != USART_SR_RXNE)
    {
      /* Wait for RXNE flag to be set */
    }

    /* Read data register to clear parity error */
    (void)USART1->DR;

    /* Set parity error */
    currentErrorStatus = USART1_PARITY_ERROR;

    /* Disable DMA Channel for RX  */
    DMA_ChannelDisable(DMA1_Channel5);
  }
  else
  {
    /* No parity error */
  }

  /* Check if idle line detected */
  if((USART1->SR & USART_SR_IDLE) == USART_SR_IDLE)
  {
	  uint32_t temp = USART1->SR & USART_SR_IDLE;
	  temp = USART1->DR;

    /* Disable DMA Channel for RX  */
    DMA_ChannelDisable(DMA1_Channel5);
    /*force DMA1_Channel5_IRQn(USART1_RX_DMA_IRQ_Callback) instead of disable the channel since STM32f1
      can not requests the end of transfers when EN bit is cleared by software */
    NVIC_SetPendingIRQ(DMA1_Channel5_IRQn);
  }
  else
  {
    /* No new data received */
  }
}


/**
 * @brief   DMA1 Channel4 initialization function
 * @note    Used for data transfer between two memory buffers
 * @param   None
 * @retval  None
 */
void USART1_TX_DMA_Config(void)
{
  /* Enable clock for DMA1*/
  RCC ->AHBENR |= RCC_AHBENR_DMA1EN;

  /* disable channel, since this registers must not be written when the channel is enabled */
 	 if(DMA_CCR1_EN == (DMA_CCR1_EN & DMA1_Channel4->CCR))
 	  {
 	    /* DMA 2 stream 5 is enabled, shall be disabled first */
 		  DMA_ChannelDisable(DMA1_Channel4);

 	    /* Wait until EN bit is cleared */
 	  while(DMA_CCR1_EN == (DMA_CCR1_EN & DMA1_Channel4->CCR))
 	    {
 	      /* Do nothing until EN bit is cleared */
 	    }
 	  }
 	  else
 	  {
 	    /* Do nothing, stream 5 is not enabled */
 	  }

  /* Set Peripheral size 8-bits (00)*/
  DMA1_Channel4 ->CCR   &= ~DMA_CCR1_PSIZE;

  /* Set Memory size 8-bits (00)*/
  DMA1_Channel4 ->CCR   &= ~DMA_CCR1_MSIZE;

  /* Set Channel priority Very high (11)*/
  DMA1_Channel4 ->CCR   |= DMA_CCR1_PL;

  /* Disable Peripheral increment mode (0) */
  DMA1_Channel4 ->CCR   &= ~DMA_CCR1_PINC;

  /* Enable memory increment mode (1)*/
  DMA1_Channel4 ->CCR   |= DMA_CCR1_MINC;

  /* Disable Circular mode (0)*/
  DMA1_Channel4 ->CCR   &= ~DMA_CCR1_CIRC;

  /* Diable M2M Mode (0) */
  DMA1_Channel4 ->CCR   &= ~DMA_CCR1_MEM2MEM;

  /* Data transfer direction Read from memory (1)*/
  DMA1_Channel4 ->CCR   |= DMA_CCR1_DIR;

  /* Enable Transfer complete interrupt */
  DMA1_Channel4 ->CCR   |= DMA_CCR1_TCIE;

  /* Set address for peripheral */
  DMA1_Channel4 ->CPAR   = (uint32_t)&USART1->DR;

	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
}


/**
 * @brief   DMA1 Channel5 initialization function
 * @note    Used for data transfer between two memory buffers
 * @param   None
 * @retval  None
 */
void USART1_RX_DMA_Config(void)
{
	 /* Enable clock for DMA1*/
	  RCC ->AHBENR |= RCC_AHBENR_DMA1EN;

	  /* disable channel, since this registers must not be written when the channel is enabled */
	 	 if(DMA_CCR1_EN == (DMA_CCR1_EN & DMA1_Channel5->CCR))
	 	  {
	 	    /* DMA 2 stream 5 is enabled, shall be disabled first */
	 		  DMA_ChannelDisable(DMA1_Channel5);

	 	    /* Wait until EN bit is cleared */
	 	  while(DMA_CCR1_EN == (DMA_CCR1_EN & DMA1_Channel5->CCR))
	 	    {
	 	      /* Do nothing until EN bit is cleared */
	 	    }
	 	  }
	 	  else
	 	  {
	 	    /* Do nothing, stream 5 is not enabled */
	 	  }

	  /* Set Peripheral size 8-bits (00)*/
	 	DMA1_Channel5 ->CCR   &= ~DMA_CCR1_PSIZE;

	  /* Set Memory size 8-bits (00)*/
	 	DMA1_Channel5 ->CCR   &= ~DMA_CCR1_MSIZE;

	  /* Set Channel priority Very high (11)*/
	 	DMA1_Channel5 ->CCR   |= DMA_CCR1_PL;

	  /* Disable Peripheral increment mode (0) */
	 	DMA1_Channel5 ->CCR   &= ~DMA_CCR1_PINC;

	  /* Enable memory increment mode (1)*/
	 	DMA1_Channel5 ->CCR   |= DMA_CCR1_MINC;

	  /* Disable Circular mode (0)*/
	 	DMA1_Channel5 ->CCR   &= ~DMA_CCR1_CIRC;

	  /* Diable M2M Mode (0) */
	 	DMA1_Channel5 ->CCR   &= ~DMA_CCR1_MEM2MEM;

	  /* Data transfer direction Read from peripheral (0)*/
	 	DMA1_Channel5 ->CCR   &= ~DMA_CCR1_DIR;

	  /* Enable Transfer complete interrupt */
	 	DMA1_Channel5 ->CCR   |= DMA_CCR1_TCIE;

	    /* Set address for peripheral */
	 	DMA1_Channel5 ->CPAR   = (uint32_t)&USART1->DR;

	    /* Set address for memory */
	 	DMA1_Channel5 ->CMAR   = (uint32_t)RxDMABuffer;

		/* Set number of data items */
	 	DMA1_Channel5 ->CNDTR  =  MAX_BUFFER_LENGTH;

		__ASM("NOP");
		__ASM("NOP");
		__ASM("NOP");
		__ASM("NOP");
}
/**
 * @brief   IRQ callback function
 * @note
 * @param   None
 * @retval  None
 */
void USART1_TX_DMA_IRQ_Callback(void)
{
	 /* Check transfer complete flag */
	if((DMA1->ISR & DMA_ISR_TCIF4) == DMA_ISR_TCIF4)
	{
		/* DMA transfer is complete */

	    /* Clear transfer complete flag */
		DMA1->IFCR |= DMA_IFCR_CTCIF4;

        while((USART1->SR& USART_SR_TC) != USART_SR_TC){}
	    /* Disable DMA 1 Channel 4 */
	    DMA_ChannelDisable(DMA1_Channel4);
	}

	 /* Clears the GIF, TEIF, HTIF and TCIF flags in the DMA_ISR register */
    	DMA1->IFCR |= DMA_IFCR_CGIF4;
}

/**
 * @brief   IRQ callback function
 * @note
 * @param   None
 * @retval  None
 */
void USART1_RX_DMA_IRQ_Callback(void)
{
  /* Check transfer complete flag */

	//if((DMA1->ISR & DMA_ISR_TCIF5) == DMA_ISR_TCIF5)
  //{
    /* Calculate amount of data received */
    RxMessageLength = MAX_BUFFER_LENGTH - DMA1_Channel5->CNDTR;
    /* Reset address for memory */
 	DMA1_Channel5 ->CMAR   = (uint32_t)RxDMABuffer;

	/* Reset number of data items */
 	DMA1_Channel5 ->CNDTR  =  MAX_BUFFER_LENGTH;

    /* Copy data into RX buffer */
    for(int idx = 0; idx < RxMessageLength; idx++)
    {
      RxBuffer[idx] = RxDMABuffer[idx];
    }

    /* Check error status */
    if(USART1_NO_ERROR != currentErrorStatus)
    {
      /* Error detected, discard the received data */
      RxMessageLength = 0;
    }
    else
    {
      /* No error detected */
    }


	 /* Clears the GIF, TEIF, HTIF and TCIF flags in the DMA_ISR register */
   	DMA1->IFCR |= DMA_IFCR_CGIF5;

    /* Enable DMA 1 Channel 5 */
   	DMA_ChannelEnable(DMA1_Channel5,1,5);
 // }
  //else
  //{
    /* Do nothing, this interrupt is not handled */
  //}
}


/**
 * @brief   String transmit
 * @note
 * @param   USARTX, str
 * @retval  None
 */
void USART_Send_String(USART_TypeDef *USARTx,const char *str)
{
 /* Check null pointers */
 if(NULL != str)
  {

	 /* Wait until DMA1 Channel 4 is disabled */
     while(DMA_CCR1_EN == (DMA_CCR1_EN & DMA1_Channel4->CCR))
	    {
	      /* Do nothing, the enable flag shall reset
	       * when DMA transfer complete */
	    }
     /* Set address for memory */
     DMA1_Channel4 ->CMAR   = (uint32_t)str;

     /* Set no. of data to transfer */
     int size =0;
     while(*str++ != '\0')size++;

     DMA1_Channel4 ->CNDTR  = ++size;
     /* Enable DMA */
    DMA_ChannelEnable(DMA1_Channel4,1,4);
  }
 else
 {
   /* Null pointers, do nothing */
 }
}



/**
 * @brief   USART1 GPIO initialization function
 * @note    PA9  -> USART1_TX
 *          PA10 -> USART1_RX
 *          PA11 -> USART1_CTS
 *          PA12 -> USART1_RTS
 *          "Table 5. Medium-density STM32F103xx pin definitions" in Datasheet
 *          "Table 24. USARTs" in Reference manual
 * @param   HFC if = 1 -> Init. CTS & RTS pins
 * @retval  None
 */
void USART1_GPIO_Init(uint8_t HFC)
{
 /* GPIOA clock enable */
  	RCC ->APB2ENR   |= RCC_APB2ENR_IOPAEN;

 /* PA9 TX: Output mode, max speed 2 MHz. */
	GPIOA ->CRH     &= ~GPIO_CRH_MODE9;
	GPIOA ->CRH     |=  GPIO_CRH_MODE9_1;

 /* PA9 TX: Alternate function output Push-pull */
  	GPIOA ->CRH     &= ~GPIO_CRH_CNF9;
    GPIOA ->CRH     |=  GPIO_CRH_CNF9_1;

 /* PA10 RX: Floating input */
  	GPIOA ->CRH     &= ~GPIO_CRH_CNF10;
    GPIOA ->CRH     |=  GPIO_CRH_CNF10_0;

 /* PA10 RX: Input mode */
  	GPIOA ->CRH     &= ~GPIO_CRH_MODE10;
 if(HFC == 1)
 {
 /* PA11 CTS: Floating input */
    GPIOA ->CRH     &= ~GPIO_CRH_CNF11;
  	GPIOA ->CRH     |=  GPIO_CRH_CNF11_0;

 /* PA11 CTS: Input mode */
  	GPIOA ->CRH     &= ~GPIO_CRH_MODE11;

 /* PA12 RTS: Output mode, max speed 2 MHz. */
  	GPIOA ->CRH     &= ~GPIO_CRH_MODE12;
  	GPIOA ->CRH     |=  GPIO_CRH_MODE12_1;

 /* PA12 RTS: Alternate function output Push-pull */
  	GPIOA ->CRH     &= ~GPIO_CRH_CNF12;
  	GPIOA ->CRH     |=  GPIO_CRH_CNF12_1;
 }


}

/**
 * @brief   USART BRR value calculation
 * @note    F_CK Input clock to the peripheral(PCLK1[APB1] for USART2, 3, 4, 5 or PCLK2[APB2] for USART1) & always  over-sampling by 16
 * @param   Baud_Rate:    Desired Baud Rate value
 *          F_CK:         Input clock to the peripheral in Hz
 * @retval  Value of BRR
 */
uint16_t Cal_USART_BRR_Val(uint32_t Baud_Rate, uint32_t F_CK)
{
	 double USARTDIV=0;
	 uint8_t Fraction;

	   /* Set baud rate = 115200 Bps
	    * USARTDIV = Fck / (16 * baud_rate)
	    *          = 72000000 / (16 * 115200) = 39.0625
	    *
	    * DIV_Fraction = 16 * 0.0625 = 1 = 0x1
	    * DIV_Mantissa = 39 = 0x27
	    *
	    * BRR          = 0x271 */

	  USARTDIV    = ( F_CK/(Baud_Rate*16.0) );
	  Fraction = round( (USARTDIV - ((uint16_t)USARTDIV) )* 16 ) ;
	  if(Fraction > 15)
		 {
		    Fraction=0;
		    USARTDIV++;
		 }
	  return ( ( ((uint16_t)USARTDIV) << 4 ) + Fraction) ;
}


/**
 * @brief   USART initialization function
 * @note    None
 * @param   BRR_Val:     Can be calculated using Cal_USART_BRR_Val function
 * @retval  None
 */
void USART1_Init(uint16_t BRR_Val)
{

	/* USART GPIO configuration -------------------------------------------------------*/

	  /* Configuration GPIOA TX & RX based on Reference manual Table 24 & Table 54	*/
		USART1_GPIO_Init(1);

	/* USART configuration -------------------------------------------------------*/
	   /*Enable USART1 clock */
	   RCC ->APB2ENR   |=  RCC_APB2ENR_USART1EN;

	   /* select 1 Start bit, 9 Data bits, n Stop bit  */
	   USART1 ->CR1    |= USART_CR1_M;

	   /* STOP bits, 00: 1 Stop bit */
	   USART1->CR2    &= ~USART_CR2_STOP;

	   /* Select odd parity */
	   USART1->CR1 |= USART_CR1_PS;

	   /* Enable parity control */
	   USART1->CR1 |= USART_CR1_PCE;

	   /* Set Baud Rate */
	   USART1->BRR = BRR_Val;

 	   /* DMA mode enabled for reception */
	   USART1->CR3  |= USART_CR3_DMAR;

       /* DMA mode enabled for transmitting */
	   USART1->CR3  |= USART_CR3_DMAT;

 	  /* Enable RTS flow control */
 	  //USART1->CR3 |= USART_CR3_RTSE;

       /* Enable CTS flow control */
 	  //USART1->CR3 |= USART_CR3_CTSE;

		__ASM("NOP");
		__ASM("NOP");
		__ASM("NOP");
		__ASM("NOP");

}

/**
 * @brief   Enable USART transmitter and receiver
 * @note
 * @param   USARTx ,where x=1 ..3
 * @retval  None
 */
void USART_Enable(USART_TypeDef *USARTx)
{
  /* Enable USART1 */
  USARTx->CR1 |= USART_CR1_UE;

  /* Enable transmitter */
  USARTx->CR1 |= USART_CR1_TE;

  /* Enable receiver */
  USARTx->CR1 |= USART_CR1_RE;

  /* Enable reception buffer not empty flag interrupt */
  USARTx->CR1 |= USART_CR1_RXNEIE;

  /* Enable parity error interrupt */
  USARTx->CR1 |= USART_CR1_PEIE;

  /* Enable idle line detection interrupt */
  USARTx->CR1 |= USART_CR1_IDLEIE;

}


/**
 * @brief   USART1 transmit and receive data
 * @note
 * @param   None
 * @retval  None
 */
void USART1_Process(void)
{
  /* Check error status */
  if(USART1_NO_ERROR == currentErrorStatus)
  {
    /* Check current USART state */
    switch (currentState)
    {
      case USART1_IDLE:
        /* Transmit data */
    	  USART_Send_String(USART1, hello_world);

        /* Go to next state */
        currentState = USART1_WAIT_FOR_RESPONCE;
        break;

      case USART1_WAIT_FOR_RESPONCE:
        /* Check if new message received */
        if(0 != RxMessageLength)
        {
          /* Reset message length */
          RxMessageLength = 0;

          /* Go to next state */
          currentState = USART1_ASK_FOR_NAME;
        }
        else
        {
          /* Nothing received yet */
        }
        break;

      case USART1_ASK_FOR_NAME:
        /* Transmit data */
    	  USART_Send_String(USART1, ask_for_name);

        /* Go to next state */
        currentState = USART1_WAIT_FOR_NAME;
        break;

      case USART1_WAIT_FOR_NAME:
        /* Check if new message received */
        if(0 != RxMessageLength)
        {
          /* Transmit data */
        	USART_Send_String(USART1, hi);
        	USART_Send_String(USART1, RxBuffer);
        	USART_Send_String(USART1,ask_for_command);
        	USART_Send_String(USART1,ask_for_command_ex);

          /* Reset message length */
          RxMessageLength = 0;

          /* Go to next state */
          currentState = USART1_WAIT_FOR_COMMAND;
        }
        else
        {
          /* Nothing received yet */
        }
        break;

      case USART1_WAIT_FOR_COMMAND:
        /* Check if new message received */
        if(0 != RxMessageLength)
        {
          /* Reset message length */
          RxMessageLength = 0;

          /* String compare results */
          strCmpReturnType isMatch_01 = STR_NOT_EQUAL;
          strCmpReturnType isMatch_02 = STR_NOT_EQUAL;
          strCmpReturnType isMatch_03 = STR_NOT_EQUAL;
          strCmpReturnType isMatch_04 = STR_NOT_EQUAL;

          /* Compare with turn on green led command */
          isMatch_01 = strCmp(turn_on_green_led, RxBuffer,
              sizeof(turn_on_green_led));

          /* Check return status */
          if(STR_EQUAL == isMatch_01)
          {
            /* Turn on green led */
           // GPIO_TurnON_LED(EVAL_GREEN_LED);

            /* Transmit data */
            USART_Send_String(USART1,done);
          }
          else
          {
            /* Compare with turn on red led command */
            isMatch_02 = strCmp(turn_on_red_led, RxBuffer,
                sizeof(turn_on_red_led));
          }

          /* Check return status */
          if(STR_EQUAL == isMatch_02)
          {
            /* Turn on red led */
           // GPIO_TurnON_LED(EVAL_RED_LED);

            /* Transmit data */
            USART_Send_String(USART1,done);
          }
          else if(STR_NOT_EQUAL == isMatch_01)
          {
            /* Compare with turn off green led command */
            isMatch_03 = strCmp(turn_off_green_led, RxBuffer,
                sizeof(turn_off_green_led));
          }
          else
          {
            /* Do nothing */
          }

          /* Check return status */
          if(STR_EQUAL == isMatch_03)
          {
            /* Turn off green led */
           // GPIO_TurnOFF_LED(EVAL_GREEN_LED);

            /* Transmit data */
            USART_Send_String(USART1,done);
          }
          else if((STR_NOT_EQUAL == isMatch_02)
              && (STR_NOT_EQUAL == isMatch_01))
          {
            /* Compare with turn off red led command */
            isMatch_04 = strCmp(turn_off_red_led, RxBuffer,
                sizeof(turn_off_red_led));
          }
          else
          {
            /* Do nothing */
          }

          /* Check return status */
          if(STR_EQUAL == isMatch_04)
          {
            /* Turn off red led */
            //GPIO_TurnOFF_LED(EVAL_RED_LED);

            /* Transmit data */
            USART_Send_String(USART1,done);
          }
          else if((STR_NOT_EQUAL == isMatch_03)
              && (STR_NOT_EQUAL == isMatch_02)
              && (STR_NOT_EQUAL == isMatch_01))
          {
            /* Transmit data */
        	  USART_Send_String(USART1,wrong_command);
          }
          else
          {
            /* Do nothing */
          }
        }
        else
        {
          /* Nothing received yet */
        }
        break;

      default:
        break;
    }
  }
  else if(USART1_PARITY_ERROR == currentErrorStatus)
  {
    /* Transmit parity error */
	  USART_Send_String(USART1,parity_error);

    /* Clear error status */
    currentErrorStatus = USART1_NO_ERROR;
  }
  else
  {
    /* No error detected */
  }
}

