//------------------------------------------------------------------------------------------------
//---- KeyDecoder.c - 2026 Dave Gaunt                                                         ----
//------------------------------------------------------------------------------------------------
//---- v1.0 - Establish sync and revieve scan codes from an amiga 500 keyboard                ----
//------------------------------------------------------------------------------------------------
#include <stdio.h>
#include "types.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/spi.h"

#include "hsync.pio.h"
#include "vsync.pio.h"
#include "rgb.pio.h"

#include "VicChars.h"
#include "Keyboard_A500.h"
#include "KeyPositions.h"

#define VGA_RESOLUTION_X    	(640)
#define VGA_RESOLUTION_Y  		(480)
#define TERMINAL_CHARS_WIDE		(VGA_RESOLUTION_X >> 3)
#define TERMINAL_CHARS_HIGH		(VGA_RESOLUTION_Y >> 3)

#define KEYBOARD_COL_FGND		(RGB_BLUE)
#define KEYBOARD_COL_BGND		(RGB_BLACK)
#define KEYBOARD_COL_PRESS		(RGB_WHITE)
#define KEYBOARD_COL_RELEASE	(RGB_GREEN)

enum keyboardCodes {KBCODE_RESET_WARNING = 0x78, KBCODE_LAST_KEY_BAD = 0xF9, KBCODE_BUFFER_OVERFLOW, KBCODE_HARDWARE_FAILURE, KBCODE_SELFTEST_FAIL, KBCODE_INITIATE_POWERUP, KBCODE_TERMINATE_POWERUP, KBCODE_INTERRUPT};
enum vga_pins {PIN_RED = 0, PIN_GREEN, PIN_BLUE, PIN_HSYNC = 8, PIN_VSYNC, PIN_KEYBOARD_CLOCK, PIN_KEYBOARD_DATA, PIN_KEYBOARD_RESET};
enum rgbColours {RGB_BLACK, RGB_RED, RGB_GREEN, RGB_YELLOW, RGB_BLUE, RGB_MAGENTA, RGB_CYAN, RGB_WHITE};

u8 aVGAScreenBuffer[(VGA_RESOLUTION_X * VGA_RESOLUTION_Y) >> 1];
u8* address_pointer = aVGAScreenBuffer;
static volatile u32 s_uKeyCode = KBCODE_LAST_KEY_BAD;

//------------------------------------------------------------------------------------------------
//----                                                                                        ----
//------------------------------------------------------------------------------------------------
void initVGA()
{
	// Choose which PIO instance to use (there are two instances, each with 4 state machines)
	PIO pio = pio0;
	const uint hsync_offset = pio_add_program(pio, &hsync_program);
	const uint vsync_offset = pio_add_program(pio, &vsync_program);
	const uint rgb_offset = pio_add_program(pio, &rgb_program);

	// Manually select a few state machines from pio instance pio0.
	uint hsync_sm = 0;
	uint vsync_sm = 1;
	uint rgb_sm = 2;
	hsync_program_init(pio, hsync_sm, hsync_offset, PIN_HSYNC);
	vsync_program_init(pio, vsync_sm, vsync_offset, PIN_VSYNC);
	rgb_program_init(pio, rgb_sm, rgb_offset, PIN_RED);

	/////////////////////////////////////////////////////////////////////////////////////////////////////
	// ============================== PIO DMA Channels =================================================
	/////////////////////////////////////////////////////////////////////////////////////////////////////

	// DMA channels - 0 sends color data, 1 reconfigures and restarts 0
	int rgb_chan_0 = 0;
	int rgb_chan_1 = 1;

	// Channel Zero (sends color data to PIO VGA machine)
	dma_channel_config c0 = dma_channel_get_default_config(rgb_chan_0);  	// default configs
	channel_config_set_transfer_data_size(&c0, DMA_SIZE_8);              	// 8-bit txfers
	channel_config_set_read_increment(&c0, true);                        	// yes read incrementing
	channel_config_set_write_increment(&c0, false);                      	// no write incrementing
	channel_config_set_dreq(&c0, DREQ_PIO0_TX2) ;                        	// DREQ_PIO0_TX2 pacing (FIFO)
	channel_config_set_chain_to(&c0, rgb_chan_1);                        	// chain to other channel

	dma_channel_configure
	(
		rgb_chan_0,                                                        	// Channel to be configured
		&c0,                                                               	// The configuration we just created
		&pio->txf[rgb_sm],                                                 	// write address (RGB PIO TX FIFO)
		&aVGAScreenBuffer,                                                 	// The initial read address (pixel color array)
		(VGA_RESOLUTION_X * VGA_RESOLUTION_Y) >> 1,                        	// Number of transfers; in this case each is 1 byte.
		false                                                              	// Don't start immediately.
	);

	// Channel One (reconfigures the first channel)
	dma_channel_config c1 = dma_channel_get_default_config(rgb_chan_1);  	// default configs
	channel_config_set_transfer_data_size(&c1, DMA_SIZE_32);             	// 32-bit txfers
	channel_config_set_read_increment(&c1, false);                       	// no read incrementing
	channel_config_set_write_increment(&c1, false);                      	// no write incrementing
	channel_config_set_chain_to(&c1, rgb_chan_0);                        	// chain to other channel

	dma_channel_configure
	(
		rgb_chan_1,                         	// Channel to be configured
		&c1,                                	// The configuration we just created
		&dma_hw->ch[rgb_chan_0].read_addr,  	// Write address (channel 0 read address)
		&address_pointer,                   	// Read address (POINTER TO AN ADDRESS)
		1,                                 	 	// Number of transfers, in this case each is 4 byte
		false                               	// Don't start immediately.
	);

  /////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////////////

	// Initialize PIO state machine counters. This passes the information to the state machines
	// that they retrieve in the first 'pull' instructions, before the .wrap_target directive
	// in the assembly. Each uses these values to initialize some counting registers.
	#define H_ACTIVE   655    // (active + frontporch - 1) - one cycle delay for mov
	#define V_ACTIVE   479    // (active - 1)
	#define RGB_ACTIVE 319    // (horizontal active)/2 - 1
	// #define RGB_ACTIVE 639 // change to this if 1 pixel/byte
	pio_sm_put_blocking(pio, hsync_sm, H_ACTIVE);
	pio_sm_put_blocking(pio, vsync_sm, V_ACTIVE);
	pio_sm_put_blocking(pio, rgb_sm, RGB_ACTIVE);

	// Start the two pio machine IN SYNC
	// Note that the RGB state machine is running at full speed,
	// so synchronization doesn't matter for that one. But, we'll
	// start them all simultaneously anyway.
	pio_enable_sm_mask_in_sync(pio, ((1u << hsync_sm) | (1u << vsync_sm) | (1u << rgb_sm)));

	// Start DMA channel 0. Once started, the contents of the pixel color array
	// will be continously DMA's to the PIO machines that are driving the screen.
	// To change the contents of the screen, we need only change the contents
	// of that array.
	dma_start_channel_mask((1u << rgb_chan_0)) ;
}

//------------------------------------------------------------------------------------------------
//----                                                                                        ----
//------------------------------------------------------------------------------------------------
void FilledRectangle(u32 uPositionX, u32 uPositionY, u32 uWidth, u32 uHeight, u32 uColour)
{
	if (uPositionX + uWidth >= VGA_RESOLUTION_X)
		uWidth = VGA_RESOLUTION_X - uPositionX;

	if (uPositionY + uHeight >= VGA_RESOLUTION_Y)
		uHeight = VGA_RESOLUTION_Y - uPositionY;

	if ((uWidth > 0) && (uHeight > 0))
	{
		u32 uPixelOffset = ((uPositionY * VGA_RESOLUTION_X) + uPositionX) >> 1;

		if (uPositionX & 1)
		{
			u32 uOffset = uPixelOffset++;
			--uWidth;

			for(u32 y=0; y<uHeight; ++y)
			{
				aVGAScreenBuffer[uOffset] = (aVGAScreenBuffer[uOffset] & 0b11000111) | (uColour << 3);
				uOffset += VGA_RESOLUTION_X >> 1;
			}
		}

		while (uWidth > 1)
		{
			u32 uOffset = uPixelOffset++;
			uWidth -= 2;

			for(u32 y=0; y<uHeight; ++y)
			{
				aVGAScreenBuffer[uOffset] = (uColour << 3) | uColour;
				uOffset += VGA_RESOLUTION_X >> 1;
			}
		}

		if (1 == uWidth)
		{
			for(u32 y=0; y<uHeight; ++y)
			{
				aVGAScreenBuffer[uPixelOffset] = (aVGAScreenBuffer[uPixelOffset] & 0b11111000) | uColour;
				uPixelOffset += VGA_RESOLUTION_X >> 1;
			}
		}
	}
}

//------------------------------------------------------------------------------------------------
//----                                                                                        ----
//------------------------------------------------------------------------------------------------
void DrawPetsciiChar(const u32 uXPos, const u32 uYPos, const u8 uChar, const u8 uColour)
{
	for (u32 uLine=0; uLine<8; ++uLine)
	{
		u32 uPixelOffset = ((((uYPos + uLine) * VGA_RESOLUTION_X) + uXPos) >> 1) + 3;
		u32 uCharLine = VicChars901460_03[2048 + (uChar << 3) + uLine];

		for (u32 x=0; x<4; ++x)
		{
			u8 uPixelPair = 0;

			if (uCharLine & 2)
				uPixelPair = uColour;

			if (uCharLine & 1)
				uPixelPair |= (uColour << 3);

			aVGAScreenBuffer[uPixelOffset--] = uPixelPair;
			uCharLine >>= 2;
		}
	}
}

//------------------------------------------------------------------------------------------------
//----                                                                                        ----
//------------------------------------------------------------------------------------------------
void DrawString(uint32_t uCharX, uint32_t uCharY, const char* pszString, const uint8_t uColour)
{
	while (*pszString)
	{
		if (uCharX >= (TERMINAL_CHARS_WIDE-1))
		{
			uCharX = 1;
			++uCharY;
		}

		if (uCharY >= (TERMINAL_CHARS_HIGH-1))
			return;

		uint8_t c = *pszString++;

		if (c >= '`')
			c -= '`';

		DrawPetsciiChar(uCharX << 3, uCharY << 3, c, uColour);
		++uCharX;
	}
}

//------------------------------------------------------------------------------------------------
//---- Send 20 microsecond acknowledge pulse                                                  ----
//------------------------------------------------------------------------------------------------
static void KeybaordAcknowledge(void)
{
	gpio_set_dir(PIN_KEYBOARD_DATA, GPIO_OUT);
	gpio_put(PIN_KEYBOARD_DATA, false);
	sleep_us(85);
	gpio_set_dir(PIN_KEYBOARD_DATA, GPIO_IN);
}

//------------------------------------------------------------------------------------------------
//---- Sync Reciever with the Keyboard data stream                                            ----
//------------------------------------------------------------------------------------------------
static void KeyboardSync(void)
{
	u32 uLow32Pins = gpioc_lo_in_get();

	// Wait For Keyboard Clock To Assert LOW
	while (((uLow32Pins >> PIN_KEYBOARD_CLOCK) & 1) == 1)
		uLow32Pins = gpioc_lo_in_get();

	// Wait For Keyboard Clock To Return HI	(Should Be 20 microseconds)
	while (((uLow32Pins >> PIN_KEYBOARD_CLOCK) & 1) == 0)
		uLow32Pins = gpioc_lo_in_get();

	// Wait 20 microseconds for the keyboard to be ready for data.
	sleep_us(20);
	KeybaordAcknowledge();
}

//------------------------------------------------------------------------------------------------
//---- Main Keyboard read function                                                            ----
//------------------------------------------------------------------------------------------------
#define __scratch_x_func(func_name)   __scratch_x(__STRING(func_name)) func_name
static void __scratch_x_func(function_core1)(void)
{
	save_and_disable_interrupts();

	KeyboardSync();

 	while(true)
 	{
		u32 uKeyCode = 0;

		for (u32 i=0; i<8; ++i)
		{
			u32 uLow32Pins = gpioc_lo_in_get();

			while (((uLow32Pins >> PIN_KEYBOARD_CLOCK) & 1) == 1)
				uLow32Pins = gpioc_lo_in_get();

			uKeyCode |= ((~uLow32Pins >> PIN_KEYBOARD_DATA) & 1) << (7 - i);

			while (((uLow32Pins >> PIN_KEYBOARD_CLOCK) & 1) == 0)
				uLow32Pins = gpioc_lo_in_get();
		}

		// Wait 20 microseconds for the keyboard to be ready to recieve data.
		sleep_us(20);
		KeybaordAcknowledge();
		s_uKeyCode = uKeyCode;
	}
}

//------------------------------------------------------------------------------------------------
//---- Caution - No Clipping!                                                                 ----
//------------------------------------------------------------------------------------------------
void CopyRectangle(const u32 uXPos, const u32 uYPos, const u32 uWidth, const u32 uHeight, const u8* pColourTable)
{
	u32 uBufferOffset = ((uYPos * VGA_RESOLUTION_X) + uXPos) >> 1;
	u32 uCopyWidth = uWidth >> 1;

	for (u32 y=0; y<uHeight; ++y)
	{
		for (u32 x=0; x<uCopyWidth; ++x)
		{
			u8 uPixelPair =  pColourTable[Keyboard_A500[((uBufferOffset + x) << 1)    ]];
			uPixelPair   |= (pColourTable[Keyboard_A500[((uBufferOffset + x) << 1) + 1]] << 3);
			aVGAScreenBuffer[uBufferOffset + x] = uPixelPair;
		}

		uBufferOffset += (VGA_RESOLUTION_X >> 1);
	}
}

//------------------------------------------------------------------------------------------------
//---- Yep it's main...                                                                       ----
//------------------------------------------------------------------------------------------------
int main()
{
	stdio_init_all();

	// Assert RESET until we are ready
	gpio_init(PIN_KEYBOARD_RESET);
	gpio_set_dir(PIN_KEYBOARD_RESET, GPIO_OUT);
	gpio_put(PIN_KEYBOARD_RESET, false);

	gpio_init(PIN_KEYBOARD_CLOCK);
	gpio_set_dir(PIN_KEYBOARD_CLOCK, GPIO_IN);

	gpio_init(PIN_KEYBOARD_DATA);
	gpio_set_dir(PIN_KEYBOARD_DATA, GPIO_IN);

	multicore_launch_core1(function_core1);

	initVGA();
	FilledRectangle(0, 0, VGA_RESOLUTION_X, VGA_RESOLUTION_Y, RGB_BLACK);

	u32 uLastKey = KBCODE_LAST_KEY_BAD;
	bool bRecievedKeyDown = false;
	bool bReset = true;

	const u8 aColourDefault[2] = {KEYBOARD_COL_BGND, KEYBOARD_COL_FGND};
	const u8 aColourPressed[2] = {KEYBOARD_COL_PRESS, KEYBOARD_COL_FGND};
	const u8 aColourReleased[2] = {KEYBOARD_COL_RELEASE, KEYBOARD_COL_FGND};

	char szTempString[128];
	u32 aErrorCodes[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	const char aErrorStrings[8][32] =
	{
		"Last Key Bad = %d\0",
		"Buffer Overflow = %d\0",
		"Controller Fail = %d (unused)\0",
		"Selftest Failed = %d\0",
		"Initiate PowerUp = %d\0",
		"Terminate PowerUp = %d\0",
		"Interrupt = %d (unused)\0",
		"\0"
	};

	// Release the keyboard from reset.
	gpio_put(PIN_KEYBOARD_RESET, true);
	gpio_set_dir(PIN_KEYBOARD_RESET, GPIO_IN);

	while(true)
	{
		DrawString(2, 40, "Reset Warning = 0", RGB_CYAN);

		if (gpio_get(PIN_KEYBOARD_RESET))
		{
			if (bReset)
			{
				// Draw the keyboard to the top of the screen.
				CopyRectangle(0, 0, 640, 200, aColourDefault);
				
				uLastKey = KBCODE_LAST_KEY_BAD;
				s_uKeyCode = KBCODE_LAST_KEY_BAD;
				bRecievedKeyDown = false;
				bReset = false;
			}

			DrawString(2, 32, "Keyboard State = Running", RGB_GREEN);
		}
		else
		{
			bReset = true;
			DrawString(2, 32, "Keyboard State = RESET!!", RGB_RED);
		}

		if (uLastKey != s_uKeyCode)
		{
			uLastKey = s_uKeyCode;

			if (0 == (s_uKeyCode & 1))
				bRecievedKeyDown = true;

			// Ignore any key release until we get at least 1 key pressed.
			if ((s_uKeyCode < KBCODE_LAST_KEY_BAD) && (bRecievedKeyDown == true))
			{
				CopyRectangle(	aKeyPosition[s_uKeyCode >> 1].uXPos,
								aKeyPosition[s_uKeyCode >> 1].uYPos,
								aKeyPosition[s_uKeyCode >> 1].uWidth,
								aKeyPosition[s_uKeyCode >> 1].uHeight,
								(s_uKeyCode & 1) ? aColourReleased : aColourPressed
							);

				// Hack to draw the L shaped Return key.
				if (0x44 == (s_uKeyCode >> 1))
					CopyRectangle(366, 100, 18, 21, (s_uKeyCode & 1) ? aColourReleased : aColourPressed);

				FilledRectangle(240, 32 * 8, VGA_RESOLUTION_X-240, 8, RGB_BLACK);
				sprintf(szTempString, "Key Scan Code = 0x%02X (%d) State %s", s_uKeyCode >> 1, s_uKeyCode >> 1, (s_uKeyCode & 1) ? "Released" : "Pressed ");
				DrawString(30, 32, szTempString, RGB_GREEN);
			}
			else
			{
				// Ignore 0xFF as this is just clocking in wait for Acknowledge bits.
				if (0xFF != s_uKeyCode)
				{
					const u8 uErrorCode = (s_uKeyCode >> 1) | 128;		// ROR
					aErrorCodes[uErrorCode - KBCODE_LAST_KEY_BAD]++;

					for(u32 i=0; i<7; ++i)
					{
						sprintf(szTempString, aErrorStrings[i], aErrorCodes[i]);
						DrawString(2, 42 + (i << 1), szTempString, RGB_CYAN);
					}
				}
			}
		}
	}
}
