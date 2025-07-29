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
#include "twi.h"
#include "ssd1306.h"
#include "I2C.h"

void TWI_Init(void)
{
    // Call the S32K144 specific initialization
    LPI2C0_init();
    LPI2C0_clock();
}

uint8_t TWI_Write(uint8_t s_w_address, uint8_t s_reg_address, uint8_t byte)
{
    return LPI2C0_write(s_w_address, s_reg_address, byte);
}

uint8_t TWI_MT_Send_Data(uint8_t data)
{
    // This will be handled internally by LPI2C0_write
    return LPI2C0_write(SSD1306_ADDR, 0x40, data);
}
uint8_t TWI_MR_Send_SLAR(uint8_t address)
{
    // This will be implemented when read functionality is needed
    return TWI_SUCCESS;
}

