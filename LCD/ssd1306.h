/**
 * -------------------------------------------------------------------------------------+
 * @brief       SSD1306 OLED Driver for s32k144
 * -------------------------------------------------------------------------------------+
 *              Copyright (C) 2025 Mohammad Shakeri.
 *              Written by  Mohammad Shakeri
 *
 * @author       Mohammad Shakeri
 * @date        28.07.2025
 * @file        ssd1306.h
 * @version     1.0.0
 * @test        s32k144
 *
 * @depend      string.h, font.h, twi.h
 * -------------------------------------------------------------------------------------+
 * @brief       Version 1.0 -> applicable for 1 display

 * -------------------------------------------------------------------------------------+
 * @usage       Basic Setup for OLED Display
 */

#ifndef __SSD1306_H__
#define __SSD1306_H__

  // @includes
  #include <LCD_Logo.h>
  #include <string.h>                     // memset function
  #include "font.h"
  #include "font5x8.h"
  #include "font6x8.h"
  #include "font8x8.h"
  #include "twi.h"

  // Success / Error
  // ------------------------------------------------------------------------------------
  #define SSD1306_SUCCESS           0
  #define SSD1306_ERROR             1
  #define FONT        0
  #define FONT_5X8    1
  #define FONT_6X8    2
  #define FONT_8X8    3
  // Address definition
  // ------------------------------------------------------------------------------------
  #define SSD1306_ADDR              0x3C<<1

  // Command definition
  // ------------------------------------------------------------------------------------
  #define SSD1306_COMMAND           0x80  // Continuation bit=1, D/C=0; 1000 0000
  #define SSD1306_COMMAND_STREAM    0x00  // Continuation bit=0, D/C=0; 0000 0000
  #define SSD1306_DATA              0xC0  // Continuation bit=1, D/C=1; 1100 0000
  #define SSD1306_DATA_STREAM       0x40  // Continuation bit=0, D/C=1; 0100 0000

  #define SSD1306_SET_MUX_RATIO     0xA8  // Set MUX ratio to N+1 MUX, N=A[5:0] : from 16MUX to 64MUX
  #define SSD1306_DISPLAY_OFFSET    0xD3  // Set Display Offset
  #define SSD1306_DISPLAY_ON        0xAF  // Display ON in normal mode
  #define SSD1306_DISPLAY_OFF       0xAE  // Display OFF (sleep mode)
  #define SSD1306_DIS_ENT_DISP_ON   0xA4  // Entire Display ON, Output ignores RAM content
  #define SSD1306_DIS_IGNORE_RAM    0xA5  // Resume to RAM content display, Output follows RAM content
  #define SSD1306_DIS_NORMAL        0xA6  // Normal display, 0 in RAM: OFF in display panel, 1 in RAM: ON in display panel
  #define SSD1306_DIS_INVERSE       0xA7  // Inverse display, 0 in RAM: ON in display panel, 1 in RAM: OFF in display panel
  #define SSD1306_DEACT_SCROLL      0x2E  // Stop scrolling that is configured by command 26h/27h/29h/2Ah
  #define SSD1306_ACTIVE_SCROLL     0x2F  // Start scrolling that is configured by the scrolling setup commands:26h/27h/29h/2Ah
  #define SSD1306_SET_START_LINE    0x40  // Set Display Start Line
  #define SSD1306_MEMORY_ADDR_MODE  0x20  // Set Memory, Addressing Mode
  #define SSD1306_SET_COLUMN_ADDR   0x21  // Set Column Address
  #define SSD1306_SET_PAGE_ADDR     0x22  // Set Page Address 
  #define SSD1306_SEG_REMAP         0xA0  // Set Segment Re-map, X[0]=0b column address 0 is mapped to SEG0
  #define SSD1306_SEG_REMAP_OP      0xA1  // Set Segment Re-map, X[0]=1b: column address 127 is mapped to SEG0
  #define SSD1306_COM_SCAN_DIR      0xC0  // Set COM Output, X[3]=0b: normal mode (RESET) Scan from COM0 to COM[N –1], e N is the Multiplex ratio
  #define SSD1306_COM_SCAN_DIR_OP   0xC8  // Set COM Output, X[3]=1b: remapped mode. Scan from COM[N-1] to COM0, e N is the Multiplex ratio
  #define SSD1306_COM_PIN_CONF      0xDA  // Set COM Pins Hardware Configuration, 
                                          // A[4]=0b, Sequential COM pin configuration, A[4]=1b(RESET), Alternative COM pin configuration
                                          // A[5]=0b(RESET), Disable COM Left/Right remap, A[5]=1b, Enable COM Left/Right remap                                        
  #define SSD1306_SET_CONTRAST      0x81  // Set Contrast Control, Double byte command to select 1 to 256 contrast steps, increases as the value increases
  #define SSD1306_SET_OSC_FREQ      0xD5  // Set Display Clock Divide Ratio/Oscillator Frequency
                                          // A[3:0] : Define the divide ratio (D) of the  display clocks (DCLK): Divide ratio= A[3:0] + 1, RESET is 0000b (divide ratio = 1) 
                                          // A[7:4] : Set the Oscillator Frequency, FOSC. Oscillator Frequency increases with the value of A[7:4] and vice versa. RESET is 1000b
  #define SSD1306_SET_CHAR_REG      0x8D  // Charge Pump Setting, A[2] = 0b, Disable charge pump(RESET), A[2] = 1b, Enable charge pump during display on 
                                          // The Charge Pump must be enabled by the following command:
                                          // 8Dh ; Charge Pump Setting
                                          // 14h ; Enable Charge Pump
                                          // AFh; Display ON
  #define SSD1306_SET_PRECHARGE     0xD9  // Set Pre-charge Period
  #define SSD1306_VCOM_DESELECT     0xDB  // Set VCOMH Deselect Leve
  #define SSD1306_NOP               0xE3  // No operation
  #define SSD1306_RESET             0xE4  // Maybe SW RESET, @source https://github.com/SmingHub/Sming/issues/501

  // Clear Color
  // ------------------------------------------------------------------------------------
  #define CLEAR_COLOR               0x00

  // Init Status
  // ------------------------------------------------------------------------------------
  #define INIT_STATUS               0xFF

  // AREA definition
  // ------------------------------------------------------------------------------------
  #define START_PAGE_ADDR           0
  #define END_PAGE_ADDR             7    // 7 for 128x64, 3 for 128x32 version
  #define START_COLUMN_ADDR         0
  #define END_COLUMN_ADDR           127
  #define RAM_X_END                 END_COLUMN_ADDR + 1
  #define RAM_Y_END                 END_PAGE_ADDR + 1

  #define CACHE_SIZE_MEM            (1 + END_PAGE_ADDR) * (1 + END_COLUMN_ADDR)

  #define MAX_X                     END_COLUMN_ADDR
  #define MAX_Y                     (END_PAGE_ADDR + 1) * 8
  extern uint16_t _counter;

    /**
   * --------------------------------------------------------------------------------------------+
   * PRIVATE FUNCTIONS
   * --------------------------------------------------------------------------------------------+
   */
   uint8_t SSD1306_Init (uint8_t);
  uint8_t SSD1306_Init1 (uint8_t);
  uint8_t SSD1306_Send_StartAndSLAW (uint8_t);
  uint8_t SSD1306_Send_Command (uint8_t);
    /**
   * +------------------------------------------------------------------------------------+
   * |== PUBLIC FUNCTIONS ================================================================|
   * +------------------------------------------------------------------------------------+
   */
  void SSD1306_ClearScreen (void);
  uint8_t SSD1306_NormalScreen (uint8_t);
  uint8_t SSD1306_InverseScreen (uint8_t);
  uint8_t SSD1306_UpdateScreen (uint8_t);
  uint8_t SSD1306_UpdatePosition (void);
  void SSD1306_SetPosition (uint8_t, uint8_t);
  uint8_t SSD1306_DrawChar (char,uint8_t font_sel);
  void SSD1306_DrawString (char * str,uint8_t font_sel);
  uint8_t SSD1306_DrawPixel (uint8_t, uint8_t);
  uint8_t SSD1306_DrawLine (uint8_t, uint8_t, uint8_t, uint8_t);
  uint8_t SSD1306_DrawCircle(uint8_t center_x, uint8_t center_y, uint8_t radius);
  uint8_t SSD1306_DrawRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2);
  uint8_t SSD1306_DrawFilledRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2);
  uint8_t SSD1306_DrawFilledCircle(uint8_t center_x, uint8_t center_y, uint8_t radius);
  uint8_t SSD1306_DrawLogo(uint8_t x, uint8_t y, uint8_t width, uint8_t height, const uint8_t *logo);
  uint8_t SSD1306_DrawLogo_ColumnFormat(uint8_t x, uint8_t y, uint8_t width, uint8_t height, const uint8_t *logo);
 void SSD1306_UpdatePage(uint8_t page);
#endif
