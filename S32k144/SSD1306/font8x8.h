/**
 * -------------------------------------------------------------------------------------+
 * @brief       FONT for SSD1306 OLED Driver for s32k144
 * -------------------------------------------------------------------------------------+
 *              Copyright (C) 2025 Mohammad Shakeri.
 *              Written by  Mohammad Shakeri
 *
 * @author       Mohammad Shakeri
 * @date        28.07.2025
 * @file        Font8x8.h
 * @version     1.0.0
 * @test        s32k144
 *
 * @depend      string.h, font.h, twi.h
 * -------------------------------------------------------------------------------------+
 * @brief       Version 1.0 -> applicable for 1 display

 * -------------------------------------------------------------------------------------+
 * @usage       Basic Setup for OLED Display
 */

#ifndef __FONT8x8_H__
#define __FONT8x8_H__
#include <stdint.h>
  // includes
 // #include <avr/pgmspace.h>

  // Characters definition
  // -----------------------------------
  // number of columns for chars
  #define CHARS_COLS_LENGTH8  8

  // @author basti79
  // @source https://github.com/basti79/LCD-fonts/blob/master/8x8_vertikal_LSB_1.h
  static const uint8_t FONTS8[][CHARS_COLS_LENGTH8]  = {
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},	// 0x20
    {0x00,0x06,0x5F,0x5F,0x06,0x00,0x00,0x00},	// 0x21
    {0x00,0x07,0x07,0x00,0x07,0x07,0x00,0x00},	// 0x22
    {0x14,0x7F,0x7F,0x14,0x7F,0x7F,0x14,0x00},	// 0x23
    {0x24,0x2E,0x6B,0x6B,0x3A,0x12,0x00,0x00},	// 0x24
    {0x46,0x66,0x30,0x18,0x0C,0x66,0x62,0x00},	// 0x25
    {0x30,0x7A,0x4F,0x5D,0x37,0x7A,0x48,0x00},	// 0x26
    {0x04,0x07,0x03,0x00,0x00,0x00,0x00,0x00},	// 0x27
    {0x00,0x1C,0x3E,0x63,0x41,0x00,0x00,0x00},	// 0x28
    {0x00,0x41,0x63,0x3E,0x1C,0x00,0x00,0x00},	// 0x29
    {0x08,0x2A,0x3E,0x1C,0x1C,0x3E,0x2A,0x08},	// 0x2A
    {0x08,0x08,0x3E,0x3E,0x08,0x08,0x00,0x00},	// 0x2B
    {0x00,0xA0,0xE0,0x60,0x00,0x00,0x00,0x00},	// 0x2C
    {0x08,0x08,0x08,0x08,0x08,0x08,0x00,0x00},	// 0x2D
    {0x00,0x00,0x60,0x60,0x00,0x00,0x00,0x00},	// 0x2E
    {0x60,0x30,0x18,0x0C,0x06,0x03,0x01,0x00},	// 0x2F
    {0x3E,0x7F,0x59,0x4D,0x7F,0x3E,0x00,0x00},	// 0x30
    {0x42,0x42,0x7F,0x7F,0x40,0x40,0x00,0x00},	// 0x31
    {0x62,0x73,0x59,0x49,0x6F,0x66,0x00,0x00},	// 0x32
    {0x22,0x63,0x49,0x49,0x7F,0x36,0x00,0x00},	// 0x33
    {0x18,0x1C,0x16,0x13,0x7F,0x7F,0x10,0x00},	// 0x34
    {0x27,0x67,0x45,0x45,0x7D,0x39,0x00,0x00},	// 0x35
    {0x3C,0x7E,0x4B,0x49,0x79,0x30,0x00,0x00},	// 0x36
    {0x03,0x63,0x71,0x19,0x0F,0x07,0x00,0x00},	// 0x37
    {0x36,0x7F,0x49,0x49,0x7F,0x36,0x00,0x00},	// 0x38
    {0x06,0x4F,0x49,0x69,0x3F,0x1E,0x00,0x00},	// 0x39
    {0x00,0x00,0x6C,0x6C,0x00,0x00,0x00,0x00},	// 0x3A
    {0x00,0xA0,0xEC,0x6C,0x00,0x00,0x00,0x00},	// 0x3B
    {0x08,0x1C,0x36,0x63,0x41,0x00,0x00,0x00},	// 0x3C
    {0x14,0x14,0x14,0x14,0x14,0x14,0x00,0x00},	// 0x3D
    {0x00,0x41,0x63,0x36,0x1C,0x08,0x00,0x00},	// 0x3E
    {0x02,0x03,0x51,0x59,0x0F,0x06,0x00,0x00},	// 0x3F
    {0x3E,0x7F,0x41,0x5D,0x5D,0x1F,0x1E,0x00},	// 0x40
    {0x7C,0x7E,0x13,0x13,0x7E,0x7C,0x00,0x00},	// 0x41
    {0x41,0x7F,0x7F,0x49,0x49,0x7F,0x36,0x00},	// 0x42
    {0x1C,0x3E,0x63,0x41,0x41,0x63,0x22,0x00},	// 0x43
    {0x41,0x7F,0x7F,0x41,0x63,0x7F,0x1C,0x00},	// 0x44
    {0x41,0x7F,0x7F,0x49,0x5D,0x41,0x63,0x00},	// 0x45
    {0x41,0x7F,0x7F,0x49,0x1D,0x01,0x03,0x00},	// 0x46
    {0x1C,0x3E,0x63,0x41,0x51,0x73,0x72,0x00},	// 0x47
    {0x7F,0x7F,0x08,0x08,0x7F,0x7F,0x00,0x00},	// 0x48
    {0x00,0x41,0x7F,0x7F,0x41,0x00,0x00,0x00},	// 0x49
    {0x30,0x70,0x40,0x41,0x7F,0x3F,0x01,0x00},	// 0x4A
    {0x41,0x7F,0x7F,0x08,0x1C,0x77,0x63,0x00},	// 0x4B
    {0x41,0x7F,0x7F,0x41,0x40,0x60,0x70,0x00},	// 0x4C
    {0x7F,0x7F,0x06,0x0C,0x06,0x7F,0x7F,0x00},	// 0x4D
    {0x7F,0x7F,0x06,0x0C,0x18,0x7F,0x7F,0x00},	// 0x4E
    {0x1C,0x3E,0x63,0x41,0x63,0x3E,0x1C,0x00},	// 0x4F
    {0x41,0x7F,0x7F,0x49,0x09,0x0F,0x06,0x00},	// 0x50
    {0x1E,0x3F,0x21,0x71,0x7F,0x5E,0x00,0x00},	// 0x51
    {0x41,0x7F,0x7F,0x19,0x39,0x6F,0x46,0x00},	// 0x52
    {0x26,0x67,0x4D,0x59,0x7B,0x32,0x00,0x00},	// 0x53
    {0x03,0x41,0x7F,0x7F,0x41,0x03,0x00,0x00},	// 0x54
    {0x7F,0x7F,0x40,0x40,0x7F,0x7F,0x00,0x00},	// 0x55
    {0x1F,0x3F,0x60,0x60,0x3F,0x1F,0x00,0x00},	// 0x56
    {0x7F,0x7F,0x30,0x18,0x30,0x7F,0x7F,0x00},	// 0x57
    {0x63,0x77,0x1C,0x08,0x1C,0x77,0x63,0x00},	// 0x58
    {0x07,0x4F,0x78,0x78,0x4F,0x07,0x00,0x00},	// 0x59
    {0x67,0x73,0x59,0x4D,0x47,0x63,0x71,0x00},	// 0x5A
    {0x00,0x7F,0x7F,0x41,0x41,0x00,0x00,0x00},	// 0x5B
    {0x01,0x03,0x06,0x0C,0x18,0x30,0x60,0x00},	// 0x5C
    {0x00,0x41,0x41,0x7F,0x7F,0x00,0x00,0x00},	// 0x5D
    {0x08,0x0C,0x06,0x03,0x06,0x0C,0x08,0x00},	// 0x5E
    {0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80},	// 0x5F
    {0x00,0x00,0x03,0x07,0x04,0x00,0x00,0x00},	// 0x60
    {0x20,0x74,0x54,0x54,0x3C,0x78,0x40,0x00},	// 0x61
    {0x41,0x3F,0x7F,0x44,0x44,0x7C,0x38,0x00},	// 0x62
    {0x38,0x7C,0x44,0x44,0x6C,0x28,0x00,0x00},	// 0x63
    {0x30,0x78,0x48,0x49,0x3F,0x7F,0x40,0x00},	// 0x64
    {0x38,0x7C,0x54,0x54,0x5C,0x18,0x00,0x00},	// 0x65
    {0x48,0x7E,0x7F,0x49,0x03,0x02,0x00,0x00},	// 0x66
    {0x98,0xBC,0xA4,0xA4,0xF8,0x7C,0x04,0x00},	// 0x67
    {0x41,0x7F,0x7F,0x08,0x04,0x7C,0x78,0x00},	// 0x68
    {0x00,0x44,0x7D,0x7D,0x40,0x00,0x00,0x00},	// 0x69
    {0x40,0xC4,0x84,0xFD,0x7D,0x00,0x00,0x00},	// 0x6A
    {0x41,0x7F,0x7F,0x10,0x38,0x6C,0x44,0x00},	// 0x6B
    {0x00,0x41,0x7F,0x7F,0x40,0x00,0x00,0x00},	// 0x6C
    {0x7C,0x7C,0x0C,0x18,0x0C,0x7C,0x78,0x00},	// 0x6D
    {0x7C,0x7C,0x04,0x04,0x7C,0x78,0x00,0x00},	// 0x6E
    {0x38,0x7C,0x44,0x44,0x7C,0x38,0x00,0x00},	// 0x6F
    {0x84,0xFC,0xF8,0xA4,0x24,0x3C,0x18,0x00},	// 0x70
    {0x18,0x3C,0x24,0xA4,0xF8,0xFC,0x84,0x00},	// 0x71
    {0x44,0x7C,0x78,0x44,0x1C,0x18,0x00,0x00},	// 0x72
    {0x48,0x5C,0x54,0x54,0x74,0x24,0x00,0x00},	// 0x73
    {0x00,0x04,0x3E,0x7F,0x44,0x24,0x00,0x00},	// 0x74
    {0x3C,0x7C,0x40,0x40,0x3C,0x7C,0x40,0x00},	// 0x75
    {0x1C,0x3C,0x60,0x60,0x3C,0x1C,0x00,0x00},	// 0x76
    {0x3C,0x7C,0x60,0x30,0x60,0x7C,0x3C,0x00},	// 0x77
    {0x44,0x6C,0x38,0x10,0x38,0x6C,0x44,0x00},	// 0x78
    {0x9C,0xBC,0xA0,0xA0,0xFC,0x7C,0x00,0x00},	// 0x79
    {0x4C,0x64,0x74,0x5C,0x4C,0x64,0x00,0x00},	// 0x7A
    {0x08,0x08,0x3E,0x77,0x41,0x41,0x00,0x00},	// 0x7B
    {0x00,0x00,0x00,0x77,0x77,0x00,0x00,0x00},	// 0x7C
    {0x41,0x41,0x77,0x3E,0x08,0x08,0x00,0x00},	// 0x7D
    {0x02,0x03,0x01,0x03,0x02,0x03,0x01,0x00},	// 0x7E
    {0x78,0x7C,0x46,0x43,0x46,0x7C,0x78,0x00},	// 0x7F
  };

#endif
