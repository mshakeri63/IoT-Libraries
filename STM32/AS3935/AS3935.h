/*
 * AS3935.h
 *
 *  Created on: Feb 8, 2024
 *      Author: moham
 */

//#ifndef APPLICATION_USER_CORE_AS3935_H_
//#define APPLICATION_USER_CORE_AS3935_H_


/*
 * AS3935.h
 *
 *  Created on: Feb 8, 2024
 *      Author: moham
 */
#ifndef AS3935_H
#define AS3935_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdbool.h"
#include "stdint.h"


enum SF_AS3935_REGISTER_NAMES
{

    AFE_GAIN = 0x00,
    THRESHOLD,
    LIGHTNING_REG,
    INT_MASK_ANT,
    ENERGY_LIGHT_LSB,
    ENERGY_LIGHT_MSB,
    ENERGY_LIGHT_MMSB,
    DISTANCE,
    FREQ_DISP_IRQ,
    CALIB_TRCO = 0x3A,
    CALIB_SRCO = 0x3B,
    RESET_LIGHT = 0x3C,
    CALIB_RCO = 0x3D
};

// Masks for various registers, there are some redundant values that I kept
// for the sake of clarity.
enum SF_AS3935_REGSTER_MASKS
{

    WIPE_ALL = 0x0,
    INT_MASK = 0xF,
    ENERGY_MASK = 0x1F,
    SPI_READ_M = 0x40,
    CALIB_MASK = 0x40,
    OSC_MASK = 0x1F,
    DISTANCE_MASK = 0x3F,
    DIV_MASK = 0x3F,
    NOISE_FLOOR_MASK = 0x8F,
    GAIN_MASK = 0xC1,
    STAT_MASK = 0xBF,
    DISTURB_MASK = 0xDF,
    LIGHT_MASK = 0xCF,
    SPIKE_MASK = 0xF0,
    THRESH_MASK = 0xF0,
    CAP_MASK = 0xF0,
    POWER_MASK = 0xFE
};

typedef enum INTERRUPT_STATUS
{

    NOISE_TO_HIGH = 0x01,
    DISTURBER_DETECT = 0x04,
    LIGHTNING = 0x08

} lightningStatus;

//#define INDOOR 0x12
//#define OUTDOOR 0xE
#define INDOOR 18
#define OUTDOOR 14

#define DIRECT_COMMAND 0x96
#define UNKNOWN_ERROR 0xFF

// Function prototypes
	  void AS3935_WriteRegister(uint8_t _wReg, uint8_t _mask, uint8_t _bits, uint8_t _startPosition);
	  uint8_t AS3935_ReadRegister(uint8_t reg);
	  void AS3935_Init(void);
	  void AS3935_Reset(void);
	  void AS3935_SetNoiseFloorLevel(uint8_t level);
	  void AS3935_LightningIntHandler(void);
	  void UART_Log(char *message);
	  void powerDown();
	  bool wakeUp();
	  void setIndoorOutdoor(uint8_t _setting);
	  uint8_t readIndoorOutdoor();
	  void watchdogThreshold(uint8_t _sensitivity);
	  uint8_t readWatchdogThreshold();
	  void setNoiseLevel(uint8_t _floor);
	  uint8_t readNoiseLevel();
	  void spikeRejection(uint8_t _spSensitivity);
	  uint8_t readSpikeRejection();
	  void lightningThreshold(uint8_t _strikes);
	  uint8_t readLightningThreshold();
	  void clearStatistics(bool _clearStat);
	  uint8_t readInterruptReg();
	  void maskDisturber(bool _state);
	  void changeDivRatio(uint8_t _divisionRatio);
	  uint8_t distanceToStorm();
	  void displayOscillator(bool _state, uint8_t _osc);
	  uint8_t readDivRatio();
	  void tuneCap(uint8_t _farad);
	  uint8_t readTuneCap();
	  uint8_t readMaskDisturber();
	  uint32_t lightningEnergy();
	  bool calibrateOsc();
	  void resetSettings();
	  void _writeRegister(uint8_t _reg, uint8_t _mask, uint8_t _bits, uint8_t _startPosition);
	  uint8_t _readRegister(uint8_t _reg);
	  int sendvalue();

#ifdef __cplusplus
}
#endif
#endif /* APPLICATION_USER_CORE_AS3935_H_ */




