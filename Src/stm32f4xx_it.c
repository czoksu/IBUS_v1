/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @date    15/10/2014 22:57:44
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2014 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

/* External variables --------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

extern uint8_t dataReceived;
extern UART_HandleTypeDef huart1;
extern uint8_t* rcvMsg;
extern uint8_t byteCounter;
extern unsigned short IBUSBuffer[32];
extern uint8_t dataReceived;
extern uint8_t msgLength;
extern uint8_t data;


/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

void USART1_IRQHandler(void)
{
  if (USART1->SR & UART_FLAG_RXNE) {                  // read interrupt
		dataReceived = 1;
		if(byteCounter < (2 + IBUSBuffer[1])) {
			IBUSBuffer[byteCounter] = USART1->DR;
			byteCounter++;
			if(byteCounter == (IBUSBuffer[1] + 2)) {
				msgLength = byteCounter;
				data = msgLength;
				byteCounter = 0;
			}
		}
	}

  HAL_NVIC_ClearPendingIRQ(USART1_IRQn);
  HAL_UART_IRQHandler(&huart1);
}



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
