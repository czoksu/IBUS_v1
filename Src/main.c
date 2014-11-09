/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 15/10/2014 22:57:46
  * Description        : Main program body
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
#include "ff.h"
#include "ff_gen_drv.h"
#include "sd_diskio.h" /* defines SD_Driver as external */
#include "string.h"
#include "stm32f4xx_hal_uart.h"
#include "IBUS.h"

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

SD_HandleTypeDef hsd;

HAL_SD_CardInfoTypedef SDCardInfo;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

SDRAM_HandleTypeDef hsdram1;

uint8_t SD_DriverNum;      /* FatFS SD part */
char SD_Path[4];           /* SD logical drive path */

/* USER CODE BEGIN 0 */
// AN_312 FT800 header
#include "FT800.h"

// AN_312 FT800 pin definitions
#define FT800_PD_N	GPIO_PIN_0
#define FT800_CS_N	GPIO_PIN_1

// AN312 FT800 screen size - uncomment the appropriate size
//#define LCD_QVGA																			// VM800B/C 3.5" QVGA - 320x240
#define LCD_WQVGA																				// VM800B/C 4.3" & 5.0" WQVGA - 480x272

// AN_312 Colors - fully saturated colors defined here
#define RED					0xFF0000UL													// Red
#define GREEN				0x00FF00UL													// Green
#define BLUE				0x0000FFUL													// Blue
#define WHITE				0xFFFFFFUL													// White
#define BLACK				0x000000UL													// Black

// Global Variables

// AN_312 FT800 pins
unsigned int ft800irqPin;																// Interrupt from FT800 to Arduino - not used here
unsigned int ft800pwrPin;																// PD_N from Arduino to FT800 - effectively FT800 reset
unsigned int ft800csPin;																// SPI chip select - defined separately since it's manipulated with GPIO calls

// AN_312 LCD display parameters
unsigned int lcdWidth;																	// Active width of LCD display
unsigned int lcdHeight;																	// Active height of LCD display
unsigned int lcdHcycle;																	// Total number of clocks per line
unsigned int lcdHoffset;																// Start of active line
unsigned int lcdHsync0;																	// Start of horizontal sync pulse
unsigned int lcdHsync1;																	// End of horizontal sync pulse
unsigned int lcdVcycle;																	// Total number of lines per screen
unsigned int lcdVoffset;																// Start of active screen
unsigned int lcdVsync0;																	// Start of vertical sync pulse
unsigned int lcdVsync1;																	// End of vertical sync pulse
unsigned char lcdPclk;																	// Pixel Clock
unsigned char lcdSwizzle;																// Define RGB output pins
unsigned char lcdPclkpol;																// Define active edge of PCLK

unsigned long ramDisplayList = RAM_DL;									// Set beginning of display list memory 
unsigned long ramCommandBuffer = RAM_CMD;								// Set beginning of graphics command memory

unsigned int cmdBufferRd = 0x0000;											// Used to navigate command ring buffer
unsigned int cmdBufferWr = 0x0000;											// Used to navigate command ring buffer
unsigned int cmdOffset = 0x0000;												// Used to navigate command rung buffer
unsigned int point_size = 0x0100;												// Define a default dot size
unsigned long point_x = (96 * 16);											// Define a default point x-location (1/16 anti-aliased)
unsigned long point_y = (136 * 16);											// Define a default point y-location (1/16 anti-aliased)
unsigned long color;																		// Variable for chanign colors
unsigned char ft800Gpio;																// Used for FT800 GPIO register
uint8_t speed = 0;
int revs = 900;
uint8_t IBUSByte = 0;
uint8_t byteCounter = 0;



/* USER CODE END 0 */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FMC_Init(void);
static void MX_IWDG_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USB_OTG_HS_USB_Init(void);

int main(void)
{

  /* USER CODE BEGIN 1 */
void ft800cmdWrite(unsigned char);
void ft800memWrite8(unsigned long, unsigned char);
void ft800memWrite16(unsigned long, unsigned int);
void ft800memWrite32(unsigned long, unsigned long);
unsigned char ft800memRead8(unsigned long);
unsigned int ft800memRead16(unsigned long);
unsigned long ft800memRead32(unsigned long);
unsigned int incCMDOffset(unsigned int, unsigned char);
void cmd_button(int16_t x, int16_t y, int16_t w, int16_t h, int16_t font, uint16_t options, const char* s );
void cmd_fgcolor(uint32_t c);
void cmd_bgcolor(uint32_t c);
void cmd_keys( int16_t x, int16_t y, int16_t w, int16_t h, int16_t font, uint16_t options, const char* s );
void cmd_slider( int16_t x, int16_t y, int16_t w, int16_t h, uint16_t options, uint16_t val, uint16_t range );
void cmd_toggle( int16_t x, int16_t y, int16_t w, int16_t font, uint16_t options, uint16_t state, const char* s );
void cmd_number( int16_t x, int16_t y, int16_t font, uint16_t options, int32_t n );
void cmd_calibrate( uint32_t result );
void cmd_track( int16_t x, int16_t y, int16_t w, int16_t h, int16_t tag );
void cmd_text( int16_t x, int16_t y, int16_t font, uint16_t option, const char* text );
void cmd_gauge( uint16_t x, uint16_t y, uint16_t r, uint16_t flat, uint16_t large, uint16_t small, uint16_t pointer, uint16_t max_pointer );

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* System interrupt init*/
  /* Sets the priority grouping field */

	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
	HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);	

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FMC_Init();
  MX_IWDG_Init();
  MX_SDIO_SD_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USB_OTG_HS_USB_Init();

  /* USER CODE BEGIN 2 */

// LCD display parameters
#ifdef LCD_QVGA																					// QVGA display parameters
  lcdWidth   = 320;																			// Active width of LCD display
  lcdHeight  = 240;																			// Active height of LCD display
  lcdHcycle  = 408;																			// Total number of clocks per line
  lcdHoffset = 70;																			// Start of active line
  lcdHsync0  = 0;																				// Start of horizontal sync pulse
  lcdHsync1  = 10;																			// End of horizontal sync pulse
  lcdVcycle  = 263;																			// Total number of lines per screen
  lcdVoffset = 13;																			// Start of active screen
  lcdVsync0  = 0;																				// Start of vertical sync pulse
  lcdVsync1  = 2;																				// End of vertical sync pulse
  lcdPclk    = 8;																				// Pixel Clock
  lcdSwizzle = 2;																				// Define RGB output pins
  lcdPclkpol = 0;																				// Define active edge of PCLK
#endif

#ifdef LCD_WQVGA																				// WQVGA display parameters
  lcdWidth   = 480;																			// Active width of LCD display
  lcdHeight  = 272;																			// Active height of LCD display
  lcdHcycle  = 548;																			// Total number of clocks per line
  lcdHoffset = 43;																			// Start of active line
  lcdHsync0  = 0;																				// Start of horizontal sync pulse
  lcdHsync1  = 41;																			// End of horizontal sync pulse
  lcdVcycle  = 292;																			// Total number of lines per screen
  lcdVoffset = 12;																			// Start of active screen
  lcdVsync0  = 0;																				// Start of vertical sync pulse
  lcdVsync1  = 10;																			// End of vertical sync pulse
  lcdPclk    = 5;																				// Pixel Clock
  lcdSwizzle = 0;																				// Define RGB output pins
  lcdPclkpol = 1;																				// Define active edge of PCLK
#endif

	
	HAL_GPIO_WritePin(GPIOB, FT800_PD_N, GPIO_PIN_SET); 	// Initial state of PD_N - high
	HAL_GPIO_WritePin(GPIOB, FT800_CS_N, GPIO_PIN_SET);		// Initial state of SPI CS - high
	HAL_Delay(20);																				// Wait 20ms
	HAL_GPIO_WritePin(GPIOB, FT800_PD_N, GPIO_PIN_RESET); // Reset FT800
	HAL_Delay(20);																				// Wait 20ms
	HAL_GPIO_WritePin(GPIOB, FT800_PD_N, GPIO_PIN_SET); 	// FT800 is awake
	HAL_Delay(20);																				// Wait 20ms - required
	
  ft800cmdWrite(FT800_CLKEXT);													// Start FT800
  HAL_Delay(5);																					// Give some time to process
  ft800cmdWrite(FT800_ACTIVE);													// Set FT800 for external clock
  HAL_Delay(5);																					// Give some time to process
  ft800cmdWrite(FT800_CLK48M);													// Set FT800 for 48MHz PLL
  HAL_Delay(5);																					// Give some time to process
  						// Now FT800 can accept commands at up to 30MHz clock on SPI bus
  						//   This application leaves the SPI bus at 8MHz
///////////////////////////////
//Getting stuck in the while loop here
//////////////////////////////////
if (ft800memRead8(REG_ID) != 0x7C)											// Read ID register - is it 0x7C?
  {
    while(1);	// If we don't get 0x7C, the ineface isn't working - halt with infinite loop
  }  
  
  ft800memWrite8(REG_PCLK, ZERO);												// Set PCLK to zero - don't clock the LCD until later
  ft800memWrite8(REG_PWM_DUTY, ZERO);										// Turn off backlight

// End of Wake-up FT800
	
//***************************************
// Initialize Display
  ft800memWrite16(REG_HSIZE,   lcdWidth);								// active display width
  ft800memWrite16(REG_HCYCLE,  lcdHcycle);							// total number of clocks per line, incl front/back porch
  ft800memWrite16(REG_HOFFSET, lcdHoffset);							// start of active line
  ft800memWrite16(REG_HSYNC0,  lcdHsync0);							// start of horizontal sync pulse
  ft800memWrite16(REG_HSYNC1,  lcdHsync1);							// end of horizontal sync pulse
  ft800memWrite16(REG_VSIZE,   lcdHeight);							// active display height
  ft800memWrite16(REG_VCYCLE,  lcdVcycle);							// total number of lines per screen, incl pre/post
  ft800memWrite16(REG_VOFFSET, lcdVoffset);							// start of active screen
  ft800memWrite16(REG_VSYNC0,  lcdVsync0);							// start of vertical sync pulse
  ft800memWrite16(REG_VSYNC1,  lcdVsync1);							// end of vertical sync pulse
  ft800memWrite8(REG_SWIZZLE,  lcdSwizzle);							// FT800 output to LCD - pin order
  ft800memWrite8(REG_PCLK_POL, lcdPclkpol);							// LCD data is clocked in on this PCLK edge
							// Don't set PCLK yet - wait for just after the first display list
// End of Initialize Display
//***************************************

//***************************************
// Configure Touch and Audio - not used in this example, so disable both
  ft800memWrite8(REG_TOUCH_MODE, ZERO);									// Disable touch
  ft800memWrite16(REG_TOUCH_RZTHRESH, ZERO);						// Eliminate any false touches
  
  ft800memWrite8(REG_VOL_PB, ZERO);											// turn recorded audio volume down
  ft800memWrite8(REG_VOL_SOUND, ZERO);									// turn synthesizer volume down
  ft800memWrite16(REG_SOUND, 0x6000);										// set synthesizer to mute
  
// End of Configure Touch and Audio
//***************************************

//***************************************
// Write Initial Display List & Enable Display

  ramDisplayList = RAM_DL;															// start of Display List
  ft800memWrite32(ramDisplayList, DL_CLEAR_RGB); 				// Clear Color RGB   00000010 RRRRRRRR GGGGGGGG BBBBBBBB  (R/G/B = Colour values) default zero / black
  ramDisplayList += 4;																	// point to next location
  ft800memWrite32(ramDisplayList, (DL_CLEAR | CLR_COL | CLR_STN | CLR_TAG));	
																												// Clear 00100110 -------- -------- -----CST  (C/S/T define which parameters to clear)
  ramDisplayList += 4;																	// point to next location
  ft800memWrite32(ramDisplayList, DL_DISPLAY);					// DISPLAY command 00000000 00000000 00000000 00000000 (end of display list)

  ft800memWrite32(REG_DLSWAP, DLSWAP_FRAME);						// 00000000 00000000 00000000 000000SS  (SS bits define when render occurs)
																												// Nothing is being displayed yet... the pixel clock is still 0x00
  ramDisplayList = RAM_DL;															// Reset Display List pointer for next list
  
  ft800Gpio = ft800memRead8(REG_GPIO);									// Read the FT800 GPIO register for a read/modify/write operation
  ft800Gpio = ft800Gpio | 0x80;													// set bit 7 of FT800 GPIO register (DISP) - others are inputs
  ft800memWrite8(REG_GPIO, ft800Gpio);									// Enable the DISP signal to the LCD panel
  ft800memWrite8(REG_PCLK, lcdPclk);										// Now start clocking data to the LCD panel
  for(int duty = 0; duty <= 128; duty++)
  {
    ft800memWrite8(REG_PWM_DUTY, duty);									// Turn on backlight - ramp up slowly to full brighness
    HAL_Delay(10);
  }

// End of Write Initial Display List & Enable Display
//***************************************	
	
/* USER CODE END 2 */

  /*## FatFS: Link the SD disk I/O driver ###############################*/
  SD_DriverNum = FATFS_LinkDriver(&SD_Driver, SD_Path);

  /* USER CODE BEGIN 3 */
  /* Infinite loop */
  while (1)
  {
		// Wait for graphics processor to complete executing the current command list
		// This happens when REG_CMD_READ matches REG_CMD_WRITE, indicating that all commands
		// have been executed.  The next commands get executed when REG_CMD_WRITE is updated again...
		// then REG_CMD_READ again catches up to REG_CMD_WRITE
		// This is a 4Kbyte ring buffer, so keep pointers within the 4K roll-over
		do
		{
			cmdBufferRd = ft800memRead16(REG_CMD_READ);					// Read the graphics processor read pointer
			cmdBufferWr = ft800memRead16(REG_CMD_WRITE); 				// Read the graphics processor write pointer
		}while (cmdBufferWr != cmdBufferRd);									// Wait until the two registers match
  
		cmdOffset = cmdBufferWr;															// The new starting point the first location after the last command
    
		ft800memWrite32(RAM_CMD + cmdOffset, (CMD_DLSTART));// Start the display list
		cmdOffset = incCMDOffset(cmdOffset, 4);								// Update the command pointer
  
		ft800memWrite32(RAM_CMD + cmdOffset, (DL_CLEAR_RGB | BLACK));
																												// Set the default clear color to black
		cmdOffset = incCMDOffset(cmdOffset, 4);								// Update the command pointer


		ft800memWrite32(RAM_CMD + cmdOffset, (DL_CLEAR | CLR_COL | CLR_STN | CLR_TAG));
																												// Clear the screen - this and the previous prevent artifacts between lists
																												// Attributes are the color, stencil and tag buffers
		cmdOffset = incCMDOffset(cmdOffset, 4);								// Update the command pointer

		cmd_gauge(114, 142, 60, OPT_FLAT, 10, 2, speed, 250);
		cmd_gauge(357, 142, 60, OPT_FLAT, 8, 2, revs, 7000);
		cmd_text(240, 22, 28, OPT_CENTER, "IBUSMONITOR");
		
		ft800memWrite32(RAM_CMD + cmdOffset, (DL_DISPLAY));		// Instruct the graphics processor to show the list
		cmdOffset = incCMDOffset(cmdOffset, 4);								// Update the command pointer
	

		ft800memWrite32(RAM_CMD + cmdOffset, (CMD_SWAP));			// Make this list active
		cmdOffset = incCMDOffset(cmdOffset, 4);								// Update the command pointer


		ft800memWrite16(REG_CMD_WRITE, (cmdOffset));					// Update the ring buffer pointer so the graphics processor starts executing
		
		revs += 100;
		HAL_Delay(50);	
		if(speed < 250) {
			speed++;
			revs -= 100;
		}
		else
			speed = 0;

// Wait a half-second to observe the changing color

 // End of loop()

		
  }
/* USER CODE END 3 */

}

/** System Clock Configuration
*/
//void SystemClock_Config(void)
//{
//  RCC_ClkInitTypeDef RCC_ClkInitStruct;
//  RCC_OscInitTypeDef RCC_OscInitStruct;
//  
//  /* Enable Power Control clock */
//  __PWR_CLK_ENABLE();
//  
//  /* The voltage scaling allows optimizing the power consumption when the device is 
//     clocked below the maximum system frequency, to update the voltage scaling value 
//     regarding system frequency refer to product datasheet.  */
//  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
//  
//  /* Enable HSE Oscillator and activate PLL with HSE as source */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
//  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//  RCC_OscInitStruct.PLL.PLLM = 8;
//  RCC_OscInitStruct.PLL.PLLN = 360;
//  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
//  RCC_OscInitStruct.PLL.PLLQ = 8;
//  HAL_RCC_OscConfig(&RCC_OscInitStruct);
//  
//  /* Activate the Over-Drive mode */
//  HAL_PWREx_ActivateOverDrive();
//  
//  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
//  clocks dividers */
//  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
//  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
//}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 6;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

}

/* IWDG init function */
void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Reload = 50;
  HAL_IWDG_Init(&hiwdg);

}

/* SDIO init function */
void MX_SDIO_SD_Init(void)
{

  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  HAL_SD_Init(&hsd, &SDCardInfo);

}

/* SPI5 init function */
void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;//SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  HAL_SPI_Init(&hspi1);

}




/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);

}

/* USB_OTG_HS init function */
void MX_USB_OTG_HS_USB_Init(void)
{

}
/* FMC initialization function */
void MX_FMC_Init(void)
{
  FMC_SDRAM_TimingTypeDef SdramTiming;

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK2;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_11;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_1;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_DISABLE;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 16;
  SdramTiming.ExitSelfRefreshDelay = 16;
  SdramTiming.SelfRefreshTime = 16;
  SdramTiming.RowCycleDelay = 16;
  SdramTiming.WriteRecoveryTime = 16;
  SdramTiming.RPDelay = 16;
  SdramTiming.RCDDelay = 16;

  HAL_SDRAM_Init(&hsdram1, &SdramTiming);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PB12   ------> USB_OTG_HS_ID
     PB13   ------> USB_OTG_HS_VBUS
     PB14   ------> USB_OTG_HS_DM
     PB15   ------> USB_OTG_HS_DP
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOF_CLK_ENABLE();
  __GPIOH_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOG_CLK_ENABLE();
  __GPIOE_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();

  /*Configure GPIO pins : PC4 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

//  /*Configure GPIO pin : PC5 */
//  GPIO_InitStruct.Pin = GPIO_PIN_5;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
//  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins :  PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF12_OTG_HS_FS;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PG9 PG10 PG11 PG12 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/******************************************************************************
 * Function:        void ft800cmdWrite(ftCommand)
 * PreCondition:    None
 * Input:           ftCommand
 * Output:          None
 * Side Effects:    None
 * Overview:        Sends FT800 command
 * Note:            None
 *****************************************************************************/
void ft800cmdWrite(unsigned char ftCommand)
{
	unsigned char cZero = 0x00;														// Filler value for command
	HAL_GPIO_WritePin(GPIOB, FT800_CS_N, GPIO_PIN_RESET);	// Set chip select low
	HAL_SPI_Transmit(&hspi1, &ftCommand, 1, 0);						// Send command
	HAL_SPI_Transmit(&hspi1, &cZero, 1, 0);								// Send first filler byte
	HAL_SPI_Transmit(&hspi1, &cZero, 1, 0);								// Send second filler byte
	HAL_GPIO_WritePin(GPIOB, FT800_CS_N, GPIO_PIN_SET);		// Set chip select high
}

/******************************************************************************
 * Function:        void ft800memWritexx(ftAddress, ftDataxx, ftLength)
 * PreCondition:    None
 * Input:           ftAddress = FT800 memory space address
 *                  ftDataxx = a byte, int or long to send
 * Output:          None
 * Side Effects:    None
 * Overview:        Writes FT800 internal address space
 * Note:            "xx" is one of 8, 16 or 32
 *****************************************************************************/
void ft800memWrite8(unsigned long ftAddress, unsigned char ftData8)
{
	unsigned char cTempAddr[3];														// FT800 Memory Address

	cTempAddr[2] = (char) (ftAddress >> 16) | MEM_WRITE;	// Compose the command and address to send
	cTempAddr[1] = (char) (ftAddress >> 8);								// middle byte
	cTempAddr[0] = (char) (ftAddress);										// low byte
	
	HAL_GPIO_WritePin(GPIOB, FT800_CS_N, GPIO_PIN_RESET);	// Set chip select low
  
  for (int i = 2; i >= 0; i--)
	{
		HAL_SPI_Transmit(&hspi1, &cTempAddr[i], 1, 0); 			// Send Memory Write plus high address byte
	}
	
	for (int j = 0; j < sizeof(ftData8); j++)
	{
		HAL_SPI_Transmit(&hspi1, &ftData8, 1, 0);						// Send data byte	
	}
	HAL_GPIO_WritePin(GPIOB, FT800_CS_N, GPIO_PIN_SET);		// Set chip select high
}

void ft800memWrite16(unsigned long ftAddress, unsigned int ftData16)
{
	unsigned char cTempAddr[3];														// FT800 Memory Address
	unsigned char cTempData[2];														// 16-bit data to write

	cTempAddr[2] = (char) (ftAddress >> 16) | MEM_WRITE;	// Compose the command and address to send
	cTempAddr[1] = (char) (ftAddress >> 8);								// middle byte
	cTempAddr[0] = (char) (ftAddress);										// low byte
	
	cTempData[1] = (char) (ftData16 >> 8);								// Compose data to be sent - high byte
	cTempData[0] = (char) (ftData16);											// low byte
	
	HAL_GPIO_WritePin(GPIOB, FT800_CS_N, GPIO_PIN_RESET);	// Set chip select low
  
  for (int i = 2; i >= 0; i--)
	{
		HAL_SPI_Transmit(&hspi1, &cTempAddr[i], 1, 0); 			// Send Memory Write plus high address byte
	}

	for (int j = 0; j < sizeof(cTempData); j++)						// Start with least significant byte
	{
		HAL_SPI_Transmit(&hspi1, &cTempData[j], 1, 0);			// Send data byte	
	}
	HAL_GPIO_WritePin(GPIOB, FT800_CS_N, GPIO_PIN_SET);		// Set chip select high
}

void ft800memWrite32(unsigned long ftAddress, unsigned long ftData32)
{
	unsigned char cTempAddr[3];														// FT800 Memory Address
	unsigned char cTempData[4];														// 32-bit data to write

	cTempAddr[2] = (char) (ftAddress >> 16) | MEM_WRITE;	// Compose the command and address to send
	cTempAddr[1] = (char) (ftAddress >> 8);								// middle byte
	cTempAddr[0] = (char) (ftAddress);										// low byte
	
	cTempData[3] = (char) (ftData32 >> 24);								// Compose data to be sent - high byte
	cTempData[2] = (char) (ftData32 >> 16);								// 
	cTempData[1] = (char) (ftData32 >> 8);								//
	cTempData[0] = (char) (ftData32);											// low byte
	
	HAL_GPIO_WritePin(GPIOB, FT800_CS_N, GPIO_PIN_RESET);	// Set chip select low
  
  for (int i = 2; i >= 0; i--)
	{
		HAL_SPI_Transmit(&hspi1, &cTempAddr[i], 1, 0); 			// Send Memory Write plus high address byte
	}
	
	for (int j = 0; j < sizeof(cTempData); j++)						// Start with least significant byte
	{
		HAL_SPI_Transmit(&hspi1, &cTempData[j], 1, 0);			// Send SPI byte
	}
	HAL_GPIO_WritePin(GPIOB, FT800_CS_N, GPIO_PIN_SET);		// Set chip select high
}
/******************************************************************************
 * Function:        unsigned char ft800memReadxx(ftAddress, ftLength)
 * PreCondition:    None
 * Input:           ftAddress = FT800 memory space address
 * Output:          ftDataxx (byte, int or long)
 * Side Effects:    None
 * Overview:        Reads FT800 internal address space
 * Note:            "xx" is one of 8, 16 or 32
 *****************************************************************************/
unsigned char ft800memRead8(unsigned long ftAddress)
{
  unsigned char ftData8 = ZERO;													// Place-holder for 8-bits being read
	unsigned char cTempAddr[3];														// FT800 Memory Address
	unsigned char cZeroFill = ZERO;												// Dummy byte

	cTempAddr[2] = (char) (ftAddress >> 16) | MEM_READ;		// Compose the command and address to send
	cTempAddr[1] = (char) (ftAddress >> 8);								// middle byte
	cTempAddr[0] = (char) (ftAddress);										// low byte

	HAL_GPIO_WritePin(GPIOB, FT800_CS_N, GPIO_PIN_RESET);	// Set chip select low

  for (int i = 2; i >= 0; i--)
	{
		HAL_SPI_Transmit(&hspi1, &cTempAddr[i], 1, 0); 			// Send Memory Write plus high address byte
	}

  HAL_SPI_Transmit(&hspi1, &cZeroFill, 1, 0);						// Send dummy byte

	for (int j = 0; j < sizeof(ftData8); j++)							// Start with least significant byte
	{
		HAL_SPI_Receive(&hspi1, &ftData8, 1, 0);						// Receive data byte	
	}
	HAL_GPIO_WritePin(GPIOB, FT800_CS_N, GPIO_PIN_SET);		// Set chip select high

  return ftData8;																				// Return 8-bits
}

unsigned int ft800memRead16(unsigned long ftAddress)
{
  unsigned int ftData16;																// 16-bits to return
	unsigned char cTempAddr[3];														// FT800 Memory Address
	unsigned char cTempData[2];														// Place-holder for 16-bits being read
	unsigned char cZeroFill;

	cTempAddr[2] = (char) (ftAddress >> 16) | MEM_READ;		// Compose the command and address to send
	cTempAddr[1] = (char) (ftAddress >> 8);								// middle byte
	cTempAddr[0] = (char) (ftAddress);										// low byte

	HAL_GPIO_WritePin(GPIOB, FT800_CS_N, GPIO_PIN_RESET);	// Set chip select low

  for (int i = 2; i >= 0; i--)
	{
		HAL_SPI_Transmit(&hspi1, &cTempAddr[i], 1, 0); 			// Send Memory Write plus high address byte
	}

  HAL_SPI_Transmit(&hspi1, &cZeroFill, 1, 0);						// Send dummy byte

	for (int j = 0; j < sizeof(cTempData); j++)						// Start with least significant byte
	{
		HAL_SPI_Receive(&hspi1, &cTempData[j], 1, 0);				// Receive data byte
	}
	HAL_GPIO_WritePin(GPIOB, FT800_CS_N, GPIO_PIN_SET);		// Set chip select high

	ftData16 = (cTempData[1]<< 8) | 											// Compose value to return - high byte
					   (cTempData[0]); 														// low byte

  return ftData16;																			// Return 16-bits
}

unsigned long ft800memRead32(unsigned long ftAddress)
{
  unsigned long ftData32;																// 32-bits to return
	unsigned char cTempAddr[3];														// FT800 Memory Address
	unsigned char cTempData[4];														// Place holder for 32-bits being read
	unsigned char cZeroFill;															// Dummy byte

	cTempAddr[2] = (char) (ftAddress >> 16) | MEM_READ;		// Compose the command and address to send
	cTempAddr[1] = (char) (ftAddress >> 8);								// middle byte
	cTempAddr[0] = (char) (ftAddress);										// low byte

	HAL_GPIO_WritePin(GPIOB, FT800_CS_N, GPIO_PIN_RESET);	// Set chip select low

  for (int i = 2; i >= 0; i--)
	{
		HAL_SPI_Transmit(&hspi1, &cTempAddr[i], 1, 0); 			// Send Memory Write plus high address byte
	}
	
  HAL_SPI_Transmit(&hspi1, &cZeroFill, 1, 0);						// Send dummy byte

	for (int j = 0; j < sizeof(cTempData); j++)						// Start with least significatn byte
	{
		HAL_SPI_Receive(&hspi1, &cTempData[j], 1, 0);				// Receive data byte
	}
	HAL_GPIO_WritePin(GPIOB, FT800_CS_N, GPIO_PIN_SET);		// Set chip select high

	ftData32 = (cTempData[3]<< 24) | 											// Compose value to return - high byte
						 (cTempData[2]<< 16) | 
						 (cTempData[1]<< 8) | 
						 (cTempData[0]); 														// Low byte

  return ftData32;																			// Return 32-bits
}

/******************************************************************************
 * Function:        void incCMDOffset(currentOffset, commandSize)
 * PreCondition:    None
 *                    starting a command list
 * Input:           currentOffset = graphics processor command list pointer
 *                  commandSize = number of bytes to increment the offset
 * Output:          newOffset = the new ring buffer pointer after adding the command
 * Side Effects:    None
 * Overview:        Adds commandSize to the currentOffset.  
 *                  Checks for 4K ring-buffer offset roll-over 
 * Note:            None
 *****************************************************************************/
unsigned int incCMDOffset(unsigned int currentOffset, unsigned char commandSize)
{
    unsigned int newOffset;															// Used to hold new offset
    newOffset = currentOffset + commandSize;						// Calculate new offset
    if(newOffset > 4095)																// If new offset past boundary...
    {
        newOffset = (newOffset - 4096);									// ... roll over pointer
    }
    return newOffset;																		// Return new offset
}

void cmd_button(int16_t x, int16_t y, int16_t w, int16_t h, int16_t font, uint16_t options, const char* s ) {
  
  		int i;
  
		ft800memWrite32(RAM_CMD + cmdOffset, (CMD_BUTTON));																												// Select the size of the dot to draw
		cmdOffset = incCMDOffset(cmdOffset, 4);								// Update the command pointer

		ft800memWrite32(RAM_CMD + cmdOffset, (x));																												// Select the size of the dot to draw
		cmdOffset = incCMDOffset(cmdOffset, 2);								// Update the command pointer
		
		ft800memWrite32(RAM_CMD + cmdOffset, (y));																												// Select the size of the dot to draw
		cmdOffset = incCMDOffset(cmdOffset, 2);								// Update the command pointer
		
		ft800memWrite32(RAM_CMD + cmdOffset, (w));																												// Select the size of the dot to draw
		cmdOffset = incCMDOffset(cmdOffset, 2);								// Update the command pointer
		
		ft800memWrite32(RAM_CMD + cmdOffset, (h));																												// Select the size of the dot to draw
		cmdOffset = incCMDOffset(cmdOffset, 2);								// Update the command pointer

		ft800memWrite32(RAM_CMD + cmdOffset, (font));																												// Select the size of the dot to draw
		cmdOffset = incCMDOffset(cmdOffset, 2);	
		
		ft800memWrite32(RAM_CMD + cmdOffset, (options));																												// Select the size of the dot to draw
		cmdOffset = incCMDOffset(cmdOffset, 2);	
		
		for(i=0; i<strlen(s); i++) {
		  
			ft800memWrite8(RAM_CMD + cmdOffset, (s[i]));																												// Select the size of the dot to draw
			cmdOffset = incCMDOffset(cmdOffset, sizeof(s[i]));	 
		
		}
}


void cmd_fgcolor(uint32_t c) {
 
		ft800memWrite32(RAM_CMD + cmdOffset, (CMD_FGCOLOR));																												// Select the size of the dot to draw
		cmdOffset = incCMDOffset(cmdOffset, 4);								// Update the command pointer

		ft800memWrite32(RAM_CMD + cmdOffset, (c));																												// Select the size of the dot to draw
		cmdOffset = incCMDOffset(cmdOffset, 4);								// Update the command pointer  
  
}

void cmd_bgcolor(uint32_t c) {
 
		ft800memWrite32(RAM_CMD + cmdOffset, (CMD_BGCOLOR));																												// Select the size of the dot to draw
		cmdOffset = incCMDOffset(cmdOffset, 4);								// Update the command pointer

		ft800memWrite32(RAM_CMD + cmdOffset, (c));																												// Select the size of the dot to draw
		cmdOffset = incCMDOffset(cmdOffset, 4);								// Update the command pointer  
  
}

void cmd_keys( int16_t x, int16_t y, int16_t w, int16_t h, int16_t font, uint16_t options, const char* s ) {
  
  		int i;
  
		ft800memWrite32(RAM_CMD + cmdOffset, (CMD_KEYS));																												// Select the size of the dot to draw
		cmdOffset = incCMDOffset(cmdOffset, 4);								// Update the command pointer

		ft800memWrite32(RAM_CMD + cmdOffset, (x));																												// Select the size of the dot to draw
		cmdOffset = incCMDOffset(cmdOffset, 2);								// Update the command pointer
		
		ft800memWrite32(RAM_CMD + cmdOffset, (y));																												// Select the size of the dot to draw
		cmdOffset = incCMDOffset(cmdOffset, 2);								// Update the command pointer
		
		ft800memWrite32(RAM_CMD + cmdOffset, (w));																												// Select the size of the dot to draw
		cmdOffset = incCMDOffset(cmdOffset, 2);								// Update the command pointer
		
		ft800memWrite32(RAM_CMD + cmdOffset, (h));																												// Select the size of the dot to draw
		cmdOffset = incCMDOffset(cmdOffset, 2);								// Update the command pointer

		ft800memWrite32(RAM_CMD + cmdOffset, (font));																												// Select the size of the dot to draw
		cmdOffset = incCMDOffset(cmdOffset, 2);	
		
		ft800memWrite32(RAM_CMD + cmdOffset, (options));																												// Select the size of the dot to draw
		cmdOffset = incCMDOffset(cmdOffset, 2);	
		
		for(i=0; i<strlen(s); i++) {
		  
			ft800memWrite8(RAM_CMD + cmdOffset, (s[i]));																												// Select the size of the dot to draw
			cmdOffset = incCMDOffset(cmdOffset, sizeof(s[i]));	 
		
		}		
  
}


void cmd_slider( int16_t x, int16_t y, int16_t w, int16_t h, uint16_t options, uint16_t val, uint16_t range ) {
  

		ft800memWrite32(RAM_CMD + cmdOffset, (CMD_SLIDER));																												// Select the size of the dot to draw
		cmdOffset = incCMDOffset(cmdOffset, 4);								// Update the command pointer

		ft800memWrite32(RAM_CMD + cmdOffset, (x));																												// Select the size of the dot to draw
		cmdOffset = incCMDOffset(cmdOffset, 2);								// Update the command pointer
		
		ft800memWrite32(RAM_CMD + cmdOffset, (y));																												// Select the size of the dot to draw
		cmdOffset = incCMDOffset(cmdOffset, 2);								// Update the command pointer
		
		ft800memWrite32(RAM_CMD + cmdOffset, (w));																												// Select the size of the dot to draw
		cmdOffset = incCMDOffset(cmdOffset, 2);								// Update the command pointer
		
		ft800memWrite32(RAM_CMD + cmdOffset, (h));																												// Select the size of the dot to draw
		cmdOffset = incCMDOffset(cmdOffset, 2);								// Update the command pointer

		ft800memWrite32(RAM_CMD + cmdOffset, (options));																												// Select the size of the dot to draw
		cmdOffset = incCMDOffset(cmdOffset, 2);	
		
		ft800memWrite32(RAM_CMD + cmdOffset, (val));																												// Select the size of the dot to draw
		cmdOffset = incCMDOffset(cmdOffset, 2);	

		ft800memWrite32(RAM_CMD + cmdOffset, (range));																												// Select the size of the dot to draw
		cmdOffset = incCMDOffset(cmdOffset, 2);	
  
}


void cmd_toggle( int16_t x, int16_t y, int16_t w, int16_t font, uint16_t options, uint16_t state, const char* s ) {
  
		int i;
		
		ft800memWrite32(RAM_CMD + cmdOffset, (CMD_TOGGLE));																												// Select the size of the dot to draw
		cmdOffset = incCMDOffset(cmdOffset, 4);								// Update the command pointer

		ft800memWrite32(RAM_CMD + cmdOffset, (x));																												// Select the size of the dot to draw
		cmdOffset = incCMDOffset(cmdOffset, 2);								// Update the command pointer
		
		ft800memWrite32(RAM_CMD + cmdOffset, (y));																												// Select the size of the dot to draw
		cmdOffset = incCMDOffset(cmdOffset, 2);								// Update the command pointer
		
		ft800memWrite32(RAM_CMD + cmdOffset, (w));																												// Select the size of the dot to draw
		cmdOffset = incCMDOffset(cmdOffset, 2);								// Update the command pointer
		
		ft800memWrite32(RAM_CMD + cmdOffset, (font));																												// Select the size of the dot to draw
		cmdOffset = incCMDOffset(cmdOffset, 2);								// Update the command pointer

		ft800memWrite32(RAM_CMD + cmdOffset, (options));																												// Select the size of the dot to draw
		cmdOffset = incCMDOffset(cmdOffset, 2);	
		
		ft800memWrite32(RAM_CMD + cmdOffset, (state));																												// Select the size of the dot to draw
		cmdOffset = incCMDOffset(cmdOffset, 2);	

		for(i=0; i<strlen(s); i++) {
		  
			ft800memWrite8(RAM_CMD + cmdOffset, (s[i]));																												// Select the size of the dot to draw
			cmdOffset = incCMDOffset(cmdOffset, sizeof(s[i]));	 
		
		}	

  
  
}

void cmd_number( int16_t x, int16_t y, int16_t font, uint16_t options, int32_t n ) {
  
  
		ft800memWrite32(RAM_CMD + cmdOffset, (CMD_NUMBER));																												// Select the size of the dot to draw
		cmdOffset = incCMDOffset(cmdOffset, 4);								// Update the command pointer

		ft800memWrite16(RAM_CMD + cmdOffset, (x));																												// Select the size of the dot to draw
		cmdOffset = incCMDOffset(cmdOffset, 2);								// Update the command pointer
		
		ft800memWrite16(RAM_CMD + cmdOffset, (y));																												// Select the size of the dot to draw
		cmdOffset = incCMDOffset(cmdOffset, 2);								// Update the command pointer
		
		ft800memWrite16(RAM_CMD + cmdOffset, (font));																												// Select the size of the dot to draw
		cmdOffset = incCMDOffset(cmdOffset, 2);								// Update the command pointer
		
		ft800memWrite16(RAM_CMD + cmdOffset, (options));																												// Select the size of the dot to draw
		cmdOffset = incCMDOffset(cmdOffset, 2);								// Update the command pointer
   
		ft800memWrite32(RAM_CMD +cmdOffset, n);
		cmdOffset = incCMDOffset(cmdOffset, 4);								// Update the command pointer
		
}


void cmd_calibrate( uint32_t result ) {
  
		ft800memWrite32(RAM_CMD +cmdOffset, CMD_CALIBRATE);
		cmdOffset = incCMDOffset(cmdOffset, 4);								// Update the command pointer
  
}

void cmd_track( int16_t x, int16_t y, int16_t w, int16_t h, int16_t tag ) {
  
		ft800memWrite32(RAM_CMD + cmdOffset, CMD_TRACK);																												// Select the size of the dot to draw
		cmdOffset = incCMDOffset(cmdOffset, 4);								// Update the command pointer

		ft800memWrite16(RAM_CMD + cmdOffset, x);																												// Select the size of the dot to draw
		cmdOffset = incCMDOffset(cmdOffset, 2);								// Update the command pointer
		
		ft800memWrite16(RAM_CMD + cmdOffset, y);																												// Select the size of the dot to draw
		cmdOffset = incCMDOffset(cmdOffset, 2);								// Update the command pointer
		
		ft800memWrite16(RAM_CMD + cmdOffset, w);																												// Select the size of the dot to draw
		cmdOffset = incCMDOffset(cmdOffset, 2);								// Update the command pointer
		
		ft800memWrite16(RAM_CMD + cmdOffset, h);																												// Select the size of the dot to draw
		cmdOffset = incCMDOffset(cmdOffset, 2);								// Update the command pointer
   
		ft800memWrite16(RAM_CMD +cmdOffset, tag);
		cmdOffset = incCMDOffset(cmdOffset, 2);								// Update the command pointer
    
}
void cmd_gauge(uint16_t x, uint16_t y, uint16_t r, uint16_t flat, uint16_t large, uint16_t small, uint16_t pointer, uint16_t max_pointer) {
		ft800memWrite32(RAM_CMD + cmdOffset, (CMD_GAUGE));		// Instruct the graphics processor to show the list
		cmdOffset = incCMDOffset(cmdOffset, 4);								// Update the command pointer
		
		ft800memWrite32(RAM_CMD + cmdOffset, (x));		// Instruct the graphics processor to show the list
		cmdOffset = incCMDOffset(cmdOffset, 2);								// Update the command pointer
		
		ft800memWrite32(RAM_CMD + cmdOffset, (y));		// Instruct the graphics processor to show the list
		cmdOffset = incCMDOffset(cmdOffset, 2);								// Update the command pointer
		
		ft800memWrite32(RAM_CMD + cmdOffset, (r));		// Instruct the graphics processor to show the list
		cmdOffset = incCMDOffset(cmdOffset, 2);								// Update the command pointer
		
		ft800memWrite32(RAM_CMD + cmdOffset, (flat));		// Instruct the graphics processor to show the list
		cmdOffset = incCMDOffset(cmdOffset, 2);								// Update the command pointer
		
		ft800memWrite32(RAM_CMD + cmdOffset, (large));		// Instruct the graphics processor to show the list
		cmdOffset = incCMDOffset(cmdOffset, 2);								// Update the command pointer
		
		ft800memWrite32(RAM_CMD + cmdOffset, (small));		// Instruct the graphics processor to show the list
		cmdOffset = incCMDOffset(cmdOffset, 2);								// Update the command pointer
		
		ft800memWrite32(RAM_CMD + cmdOffset, (pointer));		// Instruct the graphics processor to show the list
		cmdOffset = incCMDOffset(cmdOffset, 2);								// Update the command pointer
		
		ft800memWrite32(RAM_CMD + cmdOffset, (max_pointer));		// Instruct the graphics processor to show the list
		cmdOffset = incCMDOffset(cmdOffset, 2);								// Update the command pointer
}

void cmd_text(int16_t x, int16_t y, int16_t font, uint16_t option, const char* text) {
		
		int i;	
	
		ft800memWrite32(RAM_CMD + cmdOffset, (CMD_TEXT));		// Instruct the graphics processor to show the list
		cmdOffset = incCMDOffset(cmdOffset, 4);								// Update the command pointer
		
		ft800memWrite16(RAM_CMD + cmdOffset, (x));		// Instruct the graphics processor to show the list
		cmdOffset = incCMDOffset(cmdOffset, 2);								// Update the command pointer
		
		ft800memWrite16(RAM_CMD + cmdOffset, (y));		// Instruct the graphics processor to show the list
		cmdOffset = incCMDOffset(cmdOffset, 2);								// Update the command pointer
		
		ft800memWrite16(RAM_CMD + cmdOffset, (font));		// Instruct the graphics processor to show the list
		cmdOffset = incCMDOffset(cmdOffset, 2);								// Update the command pointer
		
		ft800memWrite16(RAM_CMD + cmdOffset, (option));		// Instruct the graphics processor to show the list
		cmdOffset = incCMDOffset(cmdOffset, 2);								// Update the command pointer
		
		for( i=0; i <= strlen(text); i++) {
				ft800memWrite16(RAM_CMD + cmdOffset, text[i]);		// Instruct the graphics processor to show the list
				cmdOffset = incCMDOffset(cmdOffset, sizeof(text[i]));						// Update the command pointer
		}
			
}

void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
	HAL_UART_Receive_IT(&huart1, &IBUSByte, 1);
  /* USER CODE END USART1_IRQn 0 */
  HAL_NVIC_ClearPendingIRQ(USART1_IRQn);
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE END 4 */


#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
