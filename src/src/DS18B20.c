/*******************************************************************************
 * @file    DS18B20.c
 * @author  Rashad Shubita
 * @email   shubitarashad@gmail.com
 * @date    20.01.2019
 *
 * @brief   Interfacing temperature sensor DS18B20 using UART over one-wire
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
#include "SysTick.h"
#include "gpio.h"
#include "DS18B20.h"
#include "usart.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/**
 * @brief   Reset command
 */
#define DS18B20_RESET_CMD                    ((uint8_t) 0xF0)

/**
 * @brief   Logical bit values
 */
#define BIT_0                                ((uint8_t) 0x00)
#define BIT_1                                ((uint8_t) 0xFF)

/**
 * @brief   Conversion time in ms, from DS18B20 datasheet
 */
#define MAX_CONVERSION_TIME                  ((uint32_t) 750)

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/**
 * @brief   Temperature convert, {Skip ROM = 0xCC, Convert = 0x44}
 */
static const uint8_t temp_convert[] =
{
  BIT_0, BIT_0, BIT_1, BIT_1, BIT_0, BIT_0, BIT_1, BIT_1,
  BIT_0, BIT_0, BIT_1, BIT_0, BIT_0, BIT_0, BIT_1, BIT_0
};

/**
 * @brief   Temperature data read, {Skip ROM = 0xCC, Scratch read = 0xBE}
 */
static const uint8_t temp_read[] =
{
  BIT_0, BIT_0, BIT_1, BIT_1, BIT_0, BIT_0, BIT_1, BIT_1,
  BIT_0, BIT_1, BIT_1, BIT_1, BIT_1, BIT_1, BIT_0, BIT_1,
  BIT_1, BIT_1, BIT_1, BIT_1, BIT_1, BIT_1, BIT_1, BIT_1,
  BIT_1, BIT_1, BIT_1, BIT_1, BIT_1, BIT_1, BIT_1, BIT_1
};

/* Private variables ---------------------------------------------------------*/
/**
 * @brief   Received temperature data using DMA
 */
static uint8_t temperatureData[sizeof(temp_read)];

/**
 * @brief   Temperature data received flag
 */
static uint8_t temperatureDataReceived = 0;

/**
 * @brief   Current temperature value in degree celsius
 */
static float currentTemperature = 0;

/* Private function prototypes -----------------------------------------------*/
/**
 * @brief   DMA command transmit
 * @note
 * @param   cmd, size
 * @retval  None
 */
static void cmdTransmit(const uint8_t * cmd, uint8_t size);

/**
 * @brief   DMA command receive
 * @note
 * @param   cmd, size
 * @retval  None
 */
static void cmdReceive(const uint8_t * cmd, uint8_t size);

/**
 * @brief   Send reset pulse to DS18B20
 * @note
 * @param   None
 * @retval  None
 */
static uint8_t cmdReset(void);

/* Private functions ---------------------------------------------------------*/

/**
 * @brief   DMA string transmit
 * @note    IMPORTANT: Since we send and receive the reset pulse without DMA,
 *          its necessary to clear any pending DMA requests before
 *          enable transmission DMA stream.
 * @param   cmd, size
 * @retval  None
 */
static void cmdTransmit(const uint8_t * cmd, uint8_t size)
{
	 /* Check null pointers */
	  if(NULL != cmd)
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
		  DMA1_Channel2 ->CMAR   = (uint32_t)cmd;

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

	    /* Enable DMA 1 channel 2 */
	    DMA1_Channel2->CCR |=  DMA_CCR1_EN;
	  }
	  else
	  {
	    /* Null pointers, do nothing */
	  }

}

/**
 * @brief   DMA string receive
 * @note    IMPORTANT: Since we send and receive the reset pulse without DMA,
 *          its necessary to clear any pending DMA requests before
 *          enable reception DMA stream.
 * @param   cmd, size
 * @retval  None
 */
static void cmdReceive(const uint8_t * cmd, uint8_t size)
{
  /* Check null pointers */
  if(NULL != cmd)
  {

	 /* Set address for memory */
	  DMA1_Channel3 ->CMAR   = (uint32_t)cmd;

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

    /* Enable DMA 1 channel 3 */
    DMA1_Channel3->CCR |=  DMA_CCR1_EN;
  }
  else
  {
    /* Null pointers, do nothing */
  }
}

/**
 * @brief   Send reset pulse to DS18B20
 * @note
 * @param   None
 * @retval  None
 */
static uint8_t cmdReset(void)
{
  uint8_t isSensorDetected = 0;

  /* Disable USART3 prescaler and outputs */
  USART3->CR1 &= ~USART_CR1_UE;

  /* Set baud rate = 9600 Bps */
  USART3->BRR = Cal_USART_BRR_Val(9600,8000000);

  /* Enable USART3 prescaler and outputs */
  USART3->CR1 |= USART_CR1_UE;

  /* Check USART status register */
  while(!(USART3->SR & USART_SR_TXE))
  {
    /* Wait for transmission buffer empty flag */
  }

  /* Write reset command */
  USART3->DR = DS18B20_RESET_CMD;

  /* Check USART status register */
  while(!(USART3->SR & USART_SR_TC))
  {
    /* Wait for transmission complete flag */
  }

  /* Read Rx Data */
  uint16_t Rx = USART3->DR;

  /* Check sensor presence */
  if((DS18B20_RESET_CMD != Rx) && ( BIT_0 != Rx))
  {
    /* Temp sensor was detected */
    isSensorDetected = 1;
  }
  else
  {
    /* Do nothing, No sensor was detected */
  }

  /* Disable USART3 prescaler and outputs */
  USART3->CR1 &= ~USART_CR1_UE;

  /* Set baud rate = 115200 Bps */
  USART3->BRR = Cal_USART_BRR_Val(115200, 8000000);

  /* Enable USART3 prescaler and outputs */
  USART3->CR1 |= USART_CR1_UE;

  return isSensorDetected;
}

/**
 * @}
 */

/**
 * @defgroup DS18B20_exported_functions
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
void DS18B20_GPIO_Init(void)
{
	 /* GPIOB clock enable */
	  	RCC ->APB2ENR   |= RCC_APB2ENR_IOPBEN;

	 /* PB10 TX: Output mode, max speed 2 MHz. */
		GPIOB ->CRH     &= ~GPIO_CRH_MODE10;
		GPIOB ->CRH     |=  GPIO_CRH_MODE10_1;

	 /* PB10 TX: Alternate function output Push-pull */
		GPIOB ->CRH     &= ~GPIO_CRH_CNF10;
		GPIOB ->CRH     |=  GPIO_CRH_CNF10_1;
}

/**
 * @brief   Configure DMA for USART TX
 * @note    USART3_TX -> DMA1_Channel2
 * @param   None
 * @retval  None
 */
void DS18B20_TX_DMA_Init(void)
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
void DS18B20_RX_DMA_Init(void)
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
 * @brief   Configure USART3 for DS18B20
 * @note
 * @param   None
 * @retval  None
 */
void DS18B20_USART3_Init(void)
{
  /* Enable USART3 clock */
  RCC-> APB1ENR |= RCC_APB1ENR_USART3EN;

  /* select 1 Start bit, 8 Data bits, n Stop bit  */
  USART3 ->CR1    &= ~USART_CR1_M;

  /* STOP bits, 00: 1 Stop bit */
  USART3->CR2    &= ~USART_CR2_STOP;

  /* Select Single-wire Half-duplex mode */
  USART3->CR3 |= USART_CR3_HDSEL;
}

/**
 * @brief   Enable communications with DS18B20
 * @note
 * @param   None
 * @retval  None
 */
void DS18B20_USART3_Enable(void)
{
  /* Enable USART3 */
  USART3->CR1 |= USART_CR1_UE;

  /* Enable transmitter */
  USART3->CR1 |= USART_CR1_TE;

  /* Enable receiver */
  USART3->CR1 |= USART_CR1_RE;
}

/**
 * @brief   DS18B20 process function
 * @note
 * @param   None
 * @retval  None
 */
void DS18B20_Process(void)
{
  /* Sensor detected flag */
  uint8_t isSensorDetected = 0;

  /* Send reset pulse */
  isSensorDetected = cmdReset();

  /* Check if the sensor was detected */
  if(1 == isSensorDetected)
  {
    /* Turn on green LED */
    //GPIO_TurnON_LED(EVAL_GREEN_LED);

    /* Send temperature conversion command */
    cmdTransmit(temp_convert, sizeof(temp_convert));

    /* Wait conversion time */
    SysTick_Delay(MAX_CONVERSION_TIME);

    /* Send reset pulse */
    cmdReset();

    /* Enable temperature data reception with DMA */
    cmdReceive(temperatureData, sizeof(temperatureData));

    /* Send temperature read command */
    cmdTransmit(temp_read, sizeof(temp_read));

    /* Check temperature data received flag */
    while (temperatureDataReceived == 0)
    {
      /* Wait until DMA receive temperature data */
    }

    /* Reset temperature data received flag */
    temperatureDataReceived = 0;

    /* Temporarily variable for extracting temperature data */
    uint16_t temperature = 0;

    /* Extract new temperature data */
    for (int idx = 16; idx < 32; idx++)
    {
      if (BIT_1 == temperatureData[idx])
      {
        /* Bit value is 1 */
        temperature = (temperature >> 1) | 0x8000;
      }
      else
      {
        /* Bit value is 0 */
        temperature = temperature >> 1;
      }
    }

    /* Copying new temperature data and divide by 16 for fraction part */
    currentTemperature = (float) temperature / (float) 16;

  }
  else
  {
    /* Turn on red LED, indicates sensor detection failed */
    //GPIO_TurnON_LED(EVAL_RED_LED);

    /* Temperature data not valid */
    currentTemperature = 0;
  }
}

/**
 * @brief   IRQ callback function
 * @note
 * @param   None
 * @retval  None
 */
void USART3_TX_DMA_IRQ_Callback(void)
{
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
void USART3_RX_DMA_IRQ_Callback(void)
{

	 /* Check transfer complete flag */
	if((DMA1->ISR & DMA_ISR_TCIF3) == DMA_ISR_TCIF3)
	{
		/* DMA transfer is complete */

	    /* Clear transfer complete flag */
		DMA1->IFCR |= DMA_IFCR_CTCIF3;

		 /* Clears the GIF, TEIF, HTIF and TCIF flags in the DMA_ISR register */
	   	DMA1->IFCR |= DMA_IFCR_CGIF3;
	    /* Set transfer complete flag */
	    temperatureDataReceived = 1;
	}

	else
	 {
	    /* Do nothing, this interrupt is not handled */
	 }

}




