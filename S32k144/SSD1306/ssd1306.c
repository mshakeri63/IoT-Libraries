/**
 * -------------------------------------------------------------------------------------+
 * @brief       SSD1306 OLED Driver for s32k144
 * -------------------------------------------------------------------------------------+
 *              Copyright (C) 2025 Mohammad Shakeri.
 *              Written by  Mohammad Shakeri
 *
 * @author       Mohammad Shakeri
 * @date        28.07.2025
 * @file        ssd1306.c
 * @version     1.0.0
 * @test        s32k144
 *
 * @depend      string.h, font.h, twi.h
 * -------------------------------------------------------------------------------------+
 * @brief       Version 1.0 -> applicable for 1 display

 * -------------------------------------------------------------------------------------+
 * @usage       Basic Setup for OLED Display
 */
 
// @includes
#include "ssd1306.h"
#include "I2C.h"
#include "derivative.h" /* include peripheral declarations S32K144 */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
//#include "logo.h"
uint16_t _counter;
extern const uint8_t my_logo_width;
 extern const uint8_t my_logo_height;
 extern const uint8_t my_logo[];
#define SSD1306_PAGES (END_COLUMN_ADDR / 8)
// @const List of init commands with arguments by Adafruit
// @link https://github.com/adafruit/Adafruit_SSD1306



// @var array Chache memory Lcd 8 * 128 = 1024
//static char cacheMemLcd[CACHE_SIZE_MEM];
static char cacheMemLcd[CACHE_SIZE_MEM] __attribute__((section("m_data")));
/**
 * +------------------------------------------------------------------------------------+
 * |== PRIVATE FUNCTIONS ===============================================================|
 * +------------------------------------------------------------------------------------+
 */

/**
 * @brief   SSD1306 Init
 *
 * @param   uint8_t address
 *
 * @return  uint8_t
 */
uint8_t SSD1306_Init(uint8_t address)
{ 
	TWI_Write(0x3C<<1, 0x00, 0xAE); // Display off
	TWI_Write(0x3C<<1, 0x00, 0x20); // Set Memory Addressing Mode
	TWI_Write(0x3C<<1, 0x00, 0x10); // Horizontal Addressing Mode
	TWI_Write(0x3C<<1, 0x00, 0xB0); // Set Page Start Address for Page Addressing Mode
	TWI_Write(0x3C<<1, 0x00, 0xC8); // COM Output Scan Direction
	TWI_Write(0x3C<<1, 0x00, 0x00); // Set low column address
	TWI_Write(0x3C<<1, 0x00, 0x10); // Set high column address
	TWI_Write(0x3C<<1, 0x00, 0x40); // Set start line address
	TWI_Write(0x3C<<1, 0x00, 0x81); // Set contrast control register
	TWI_Write(0x3C<<1, 0x00, 0xFF); // Contrast value
	TWI_Write(0x3C<<1, 0x00, 0xA1); // Set segment re-map 0 to 127
	TWI_Write(0x3C<<1, 0x00, 0xA6); // Set normal display
	TWI_Write(0x3C<<1, 0x00, 0xA8); // Set multiplex ratio (1 to 64)
	TWI_Write(0x3C<<1, 0x00, 0x3F); // 1/64 duty
	TWI_Write(0x3C<<1, 0x00, 0xA4); // Output follows RAM content
	TWI_Write(0x3C<<1, 0x00, 0xD3); // Set display offset
	TWI_Write(0x3C<<1, 0x00, 0x00); // No offset
	TWI_Write(0x3C<<1, 0x00, 0xD5); // Set display clock divide ratio/oscillator frequency
	TWI_Write(0x3C<<1, 0x00, 0xF0); // Default ratio + freq
	TWI_Write(0x3C<<1, 0x00, 0xD9); // Set pre-charge period
	TWI_Write(0x3C<<1, 0x00, 0x22); // Default pre-charge
	TWI_Write(0x3C<<1, 0x00, 0xDA); // Set com pins hardware configuration
	TWI_Write(0x3C<<1, 0x00, 0x12); // Alternative COM pin config
	TWI_Write(0x3C<<1, 0x00, 0xDB); // Set vcomh
	TWI_Write(0x3C<<1, 0x00, 0x22); // 0.77xVcc
	TWI_Write(0x3C<<1, 0x00, 0x8D); // Enable charge pump regulator
	TWI_Write(0x3C<<1, 0x00, 0x14); // Enable charge pump
	TWI_Write(0x3C<<1, 0x00, 0xAF); // Display ON


return 1;
}
uint8_t SSD1306_Init1(uint8_t address)
{

	TWI_Write(0x3C<<1, 0x00, 0xAF); // Display ON


return 1;
}


uint8_t SSD1306_Send_Command (uint8_t command)
{
  uint8_t status = INIT_STATUS;

  // send control byte
  // -------------------------------------------------------------------------------------   
  status = TWI_Write(0x3c<<1, 0x00, command);
  if (SSD1306_SUCCESS != status) {
    return status;
  }


  return SSD1306_SUCCESS;
}

/**
 * +------------------------------------------------------------------------------------+
 * |== PUBLIC FUNCTIONS ================================================================|
 * +------------------------------------------------------------------------------------+
 */


uint8_t SSD1306_NormalScreen(uint8_t address)
{
    uint8_t status = INIT_STATUS;

    // 2. Send command to set normal display mode
    status = SSD1306_Send_Command(SSD1306_DIS_NORMAL);
    if (SSD1306_SUCCESS != status) {
    return status;
    }

    return status;
}

uint8_t SSD1306_InverseScreen (uint8_t address)
{
  uint8_t status = INIT_STATUS;

  // send command
  // -------------------------------------------------------------------------------------   
  status = SSD1306_Send_Command (SSD1306_DIS_INVERSE);
  if (SSD1306_SUCCESS != status) {
    return status;
  }
  return SSD1306_SUCCESS;
}

uint8_t SSD1306_UpdateScreen(uint8_t address)
{
    // Update all pages
    for (uint8_t page = START_PAGE_ADDR; page <= END_PAGE_ADDR; page++) {
    	SSD1306_UpdatePage(page);
    }
    return SSD1306_SUCCESS;
}

void SSD1306_ClearScreen(void)
{
    // Clear cache memory
    memset(cacheMemLcd, 0x00, CACHE_SIZE_MEM);
    // Push cleared buffer to display
    SSD1306_UpdateScreen(SSD1306_ADDR);
}

// Helper function to update a single page
void SSD1306_UpdatePage(uint8_t page)
{
    if (page > END_PAGE_ADDR) return;


    TWI_Write(SSD1306_ADDR, 0x80,0xB0+page);    // Set page address
    TWI_Write(SSD1306_ADDR, 0x80,0x00);        // Set low column address
    TWI_Write(SSD1306_ADDR, 0x00,0x10);       // Set high column address
    // Send page data
    uint16_t start_idx = page * (END_COLUMN_ADDR+1 );
    for (uint16_t col = 0; col <= END_COLUMN_ADDR; col++) {
    	LPI2C0_write(SSD1306_ADDR, 0x40,cacheMemLcd[start_idx + col]);
    }

}

void SSD1306_SetPosition (uint8_t x, uint8_t y) 
{
  _counter = x + (y << 7);                                        // update counter
}


uint8_t SSD1306_UpdatePosition (void) 
{
  uint8_t y = _counter >> 7;                                      // y / 8
  uint8_t x = _counter - (y << 7);                                // y % 8
  uint8_t x_new = x + CHARS_COLS_LENGTH + 1;                      // x + character length + 1
  
  if (x_new > END_COLUMN_ADDR) {                                  // check position
    if (y > END_PAGE_ADDR) {                                      // if more than allowable number of pages
      return SSD1306_ERROR;                                       // return out of range
    } else if (y < (END_PAGE_ADDR-1)) {                           // if x reach the end but page in range
      _counter = ((++y) << 7);                                    // update
    }
  }
 
  return SSD1306_SUCCESS;
}


uint8_t SSD1306_DrawChar(char character,uint8_t font_sel)
{
    uint8_t i = 0;

    if (SSD1306_UpdatePosition() == SSD1306_ERROR) {
        return SSD1306_ERROR;
    }
    switch (font_sel) {
            case 0:
            	 while (i < CHARS_COLS_LENGTH) {
            	        cacheMemLcd[_counter++] = FONTS[character - 32][i++];
            	    }
            	    _counter++;
                break;
            case 1:
                   while (i < CHARS_COLS_LENGTH5) {
                         cacheMemLcd[_counter++] = FONTS5[character - 32][i++];
                     }
                        _counter++;
             break;
            case 2:
                               while (i < CHARS_COLS_LENGTH6) {
                                     cacheMemLcd[_counter++] = FONTS6[character - 32][i++];
                                 }
                                    _counter++;
                         break;
            case 3:
                               while (i < CHARS_COLS_LENGTH8) {
                                     cacheMemLcd[_counter++] = FONTS8[character - 32][i++];
                                 }
                                    _counter++;
                         break;
            default:
                               while (i < CHARS_COLS_LENGTH) {
                                     cacheMemLcd[_counter++] = FONTS[character - 32][i++];
                                 }
                                    _counter++;
                         break;

    }

    return SSD1306_SUCCESS;
}


void SSD1306_DrawString (char *str, uint8_t font_sel)
{
	uint16_t i = 0;
  while (str[i] != '\0') {
    SSD1306_DrawChar (str[i++],font_sel);
  }
}


uint8_t SSD1306_DrawPixel (uint8_t x, uint8_t y)
{
  uint8_t page = 0;
  uint8_t pixel = 0;
  
  if ((x > MAX_X) || (y > MAX_Y)) {                               // if out of range
    return SSD1306_ERROR;                                         // out of range
  }
  page = y >> 3;                                                  // find page (y / 8)
  pixel = 1 << (y - (page << 3));                                 // which pixel (y % 8)
  _counter = x + (page << 7);                                     // update counter
  cacheMemLcd[_counter++] |= pixel;                               // save pixel

  return SSD1306_SUCCESS;
}


uint8_t SSD1306_DrawLine (uint8_t x1, uint8_t x2, uint8_t y1, uint8_t y2)
{
  int16_t D;                                                      // determinant
  int16_t delta_x, delta_y;                                       // deltas
  int16_t trace_x = 1, trace_y = 1;                               // steps

  delta_x = x2 - x1;                                              // delta x
  delta_y = y2 - y1;                                              // delta y
  
  if (delta_x < 0) {                                              // check if x2 > x1
    delta_x = -delta_x;                                           // negate delta x
    trace_x = -trace_x;                                           // negate step x
  }
  
  if (delta_y < 0) {                                              // check if y2 > y1
    delta_y = -delta_y;                                           // negate detla y
    trace_y = -trace_y;                                           // negate step y
  }

  // Bresenham condition for m < 1 (dy < dx)
  // -------------------------------------------------------------------------------------
  if (delta_y < delta_x) {
    D = (delta_y << 1) - delta_x;                                 // calculate determinant
    SSD1306_DrawPixel (x1, y1);                                   // draw first pixel
    while (x1 != x2) {                                            // check if x1 equal x2
      x1 += trace_x;                                              // update x1
      if (D >= 0) {                                               // check if determinant is positive
        y1 += trace_y;                                            // update y1
        D -= 2*delta_x;                                           // update determinant
      }
      D += 2*delta_y;                                             // update deteminant
      SSD1306_DrawPixel (x1, y1);                                 // draw next pixel
    }
  // for m > 1 (dy > dx)    
  // -------------------------------------------------------------------------------------
  } else {
    D = delta_y - (delta_x << 1);                                 // calculate determinant
    SSD1306_DrawPixel (x1, y1);                                   // draw first pixel
    while (y1 != y2) {                                            // check if y2 equal y1
      y1 += trace_y;                                              // update y1
      if (D <= 0) {                                               // check if determinant is positive
        x1 += trace_x;                                            // update y1
        D += 2*delta_y;                                           // update determinant
      }
      D -= 2*delta_x;                                             // update deteminant
      SSD1306_DrawPixel (x1, y1);                                 // draw next pixel
    }
  }

  return SSD1306_SUCCESS;
}
uint8_t SSD1306_DrawRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2) {
    // Draw top line
    SSD1306_DrawLine(x1, x2, y1, y1);
    // Draw bottom line
    SSD1306_DrawLine(x1, x2, y2, y2);
    // Draw left line
    SSD1306_DrawLine(x1, x1, y1, y2);
    // Draw right line
    SSD1306_DrawLine(x2, x2, y1, y2);

    return SSD1306_SUCCESS;
}
uint8_t SSD1306_DrawCircle(uint8_t center_x, uint8_t center_y, uint8_t radius){
    int16_t x = radius;
    int16_t y = 0;
    int16_t err = 0;

    while (x >= y) {
        // Draw pixels in all octants
        SSD1306_DrawPixel(center_x + x, center_y + y);
        SSD1306_DrawPixel(center_x + y, center_y + x);
        SSD1306_DrawPixel(center_x - y, center_y + x);
        SSD1306_DrawPixel(center_x - x, center_y + y);
        SSD1306_DrawPixel(center_x - x, center_y - y);
        SSD1306_DrawPixel(center_x - y, center_y - x);
        SSD1306_DrawPixel(center_x + y, center_y - x);
        SSD1306_DrawPixel(center_x + x, center_y - y);

        if (err <= 0) {
            y += 1;
            err += 2*y + 1;
        }
        if (err > 0) {
            x -= 1;
            err -= 2*x + 1;
        }
    }

    return SSD1306_SUCCESS;
}
uint8_t SSD1306_DrawFilledRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2) {
    // Ensure x1 < x2 and y1 < y2 for proper filling
    if (x1 > x2) { uint8_t tmp = x1; x1 = x2; x2 = tmp; }
    if (y1 > y2) { uint8_t tmp = y1; y1 = y2; y2 = tmp; }

    // Fill the rectangle line by line
    for (uint8_t y = y1; y <= y2; y++) {
        SSD1306_DrawLine(x1, x2, y, y); // Draw a horizontal line for each row
    }

    return SSD1306_SUCCESS;
}
uint8_t SSD1306_DrawFilledCircle(uint8_t center_x, uint8_t center_y, uint8_t radius) {
    int16_t x = radius;
    int16_t y = 0;
    int16_t err = 0;

    while (x >= y) {
        // Draw horizontal lines to fill the circle
        SSD1306_DrawLine(center_x - x, center_x + x, center_y + y, center_y + y); // Bottom octants
        SSD1306_DrawLine(center_x - x, center_x + x, center_y - y, center_y - y); // Top octants
        SSD1306_DrawLine(center_x - y, center_x + y, center_y + x, center_y + x); // Right octants
        SSD1306_DrawLine(center_x - y, center_x + y, center_y - x, center_y - x); // Left octants

        if (err <= 0) {
            y += 1;
            err += 2 * y + 1;
        }
        if (err > 0) {
            x -= 1;
            err -= 2 * x + 1;
        }
    }

    return SSD1306_SUCCESS;
}

// Function to draw the logo at (x, y)
// Function to draw a bitmap logo at position (x, y)
uint8_t SSD1306_DrawLogo(uint8_t x, uint8_t y, uint8_t width, uint8_t height, const uint8_t *logo) {
    // Input validation
    if (logo == NULL) {
        return SSD1306_ERROR;
    }

    // Check if the logo fits within display bounds
    if ((x + width > MAX_X + 1) || (y + height > MAX_Y + 1)) {
        return SSD1306_ERROR;
    }

    // Draw the logo pixel by pixel
    for (uint8_t row = 0; row < height; row++) {
        for (uint8_t col = 0; col < width; col++) {
            // Calculate which byte and bit position in the logo data
            uint16_t byte_index = (row * ((width + 7) / 8)) + (col / 8);
            uint8_t bit_position = 7 - (col % 8);  // MSB first

            // Check if the pixel should be drawn
            if (logo[byte_index] & (1 << bit_position)) {
                // Use your existing DrawPixel function with bounds checking
                if (SSD1306_DrawPixel(x + col, y + row) == SSD1306_ERROR) {
                    return SSD1306_ERROR;
                }
            }
        }
    }

    return SSD1306_SUCCESS;
}

// Alternative version for column-organized bitmap data (common in embedded displays)
uint8_t SSD1306_DrawLogo_ColumnFormat(uint8_t x, uint8_t y, uint8_t width, uint8_t height, const uint8_t *logo) {
    // Input validation
    if (logo == NULL) {
        return SSD1306_ERROR;
    }

    // Check bounds
    if ((x + width > MAX_X + 1) || (y + height > MAX_Y + 1)) {
        return SSD1306_ERROR;
    }

    // For column-organized data (each byte represents 8 vertical pixels)
    uint8_t pages = (height + 7) / 8;  // Number of 8-pixel pages needed

    for (uint8_t col = 0; col < width; col++) {
        for (uint8_t page = 0; page < pages; page++) {
            uint8_t byte_data = logo[col * pages + page];

            for (uint8_t bit = 0; bit < 8; bit++) {
                uint8_t pixel_y = y + (page * 8) + bit;

                // Make sure we don't draw outside the logo height
                if (pixel_y >= y + height) break;

                if (byte_data & (1 << bit)) {
                    if (SSD1306_DrawPixel(x + col, pixel_y) == SSD1306_ERROR) {
                        return SSD1306_ERROR;
                    }
                }
            }
        }
    }

    return SSD1306_SUCCESS;
}
