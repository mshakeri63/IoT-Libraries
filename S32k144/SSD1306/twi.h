#ifndef __TWI_S32K144_H__
#define __TWI_S32K144_H__

#include "S32K144.h"

// Status codes
#define TWI_SUCCESS          0
#define TWI_ERROR           1
#define TWI_BUSY            2

// Function prototypes
void TWI_Init(void);
uint8_t TWI_MT_Start(void);
uint8_t TWI_MT_Send_SLAW(uint8_t address);
uint8_t TWI_MT_Send_Data(uint8_t data);
uint8_t TWI_MR_Send_SLAR(uint8_t address);
void TWI_Stop(void);

// Wrapper function to match S32K144 style
uint8_t TWI_Write(uint8_t s_w_address, uint8_t s_reg_address, uint8_t byte);

#endif
