#include "FT800.h"

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
		
		uint32_t i, Count;	
		uint32_t NBytes = strlen((const char *)text) + 1;
		NBytes = (NBytes + 3)&(~3);
		
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
		
		while(NBytes) {
			Count = NBytes;
			
			for(i = 0;i<Count;i++) {
				ft800memWrite8(RAM_CMD + cmdOffset, *text++);
				cmdOffset = incCMDOffset(cmdOffset, 1);
			}
			
			NBytes -= Count;
			
			
		}
}

