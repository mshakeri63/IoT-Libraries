/*
 * AS3935.c
 *
 *  Created on: Feb 8, 2024
 *      Author: moham
 */
#include "AS3935.h"
#include "stdint.h"
#include "stdio.h"
#include "stm32wbxx_hal.h"
#include "string.h"
#include "main.h"
#include "stm32wbxx_hal_uart.h"
#include "stm32wbxx_hal_spi.h"

 extern UART_HandleTypeDef huart1;
 extern SPI_HandleTypeDef  hspi1;


//MX_USART1_UART_Init();
//MX_SPI1_Init();
void powerDown()
{
    _writeRegister(AFE_GAIN, POWER_MASK, 1, 0);
}
bool wakeUp()
{

    _writeRegister(AFE_GAIN, POWER_MASK, 0, 0); // Set the power down bit to zero to wake it up

    if (calibrateOsc())
        return true;
    else
        return false;
}

// REG0x00, bits [5:1], manufacturer default: 10010 (INDOOR).
// This function changes toggles the chip's settings for Indoors and Outdoors.
void setIndoorOutdoor(uint8_t _setting)
{
    if (((_setting != INDOOR) && (_setting != OUTDOOR)))
        return

    _writeRegister(AFE_GAIN, GAIN_MASK, _setting, 1);
}

// REG0x00, bits [5:1], manufacturer default: 10010 (INDOOR).
// This function returns the indoor/outdoor settting.
uint8_t readIndoorOutdoor()
{

    uint8_t regVal = _readRegister(AFE_GAIN);
    //return ((regVal ));
    return ((regVal &= ~GAIN_MASK) >> 1);
    //return (regVal  >> 1);
}

// REG0x01, bits[3:0], manufacturer default: 0010 (2).
// This setting determines the threshold for events that trigger the
// IRQ Pin.
void watchdogThreshold(uint8_t _sensitivity)
{
    if (_sensitivity > 10) // 10 is the max sensitivity setting
        return;

    _writeRegister(THRESHOLD, THRESH_MASK, _sensitivity, 0);
}

// REG0x01, bits[3:0], manufacturer default: 0010 (2).
// This function returns the threshold for events that trigger the
// IRQ Pin.
uint8_t readWatchdogThreshold()
{

    uint8_t regVal = _readRegister(THRESHOLD);
    uint8_t _spiWrite;

        		    _spiWrite = _readRegister(THRESHOLD);       // Get the current value of the register
        		    _spiWrite &= ~THRESH_MASK;                     // Mask the position we want to write to
        		    _spiWrite |= (_spiWrite >> 0); // Write the given bits to the variable


        //return(_spiWrite);

    return (regVal &= (~THRESH_MASK));
   // return (regVal);
}

// REG0x01, bits [6:4], manufacturer default: 010 (2).
// The noise floor level is compared to a known reference voltage. If this
// level is exceeded the chip will issue an interrupt to the IRQ pin,
// broadcasting that it can not operate properly due to noise (INT_NH).
// Check datasheet for specific noise level tolerances when setting this register.
void setNoiseLevel(uint8_t _floor)
{
    if (_floor > 7)
        return;

    _writeRegister(THRESHOLD, NOISE_FLOOR_MASK, _floor, 4);
}

// REG0x01, bits [6:4], manufacturer default: 010 (2).
// This function will return the set noise level threshold: default is 2.
uint8_t readNoiseLevel()
{
    //uint8_t regVal = _readRegister(THRESHOLD);
    uint8_t _spiWrite=0;
    			_spiWrite = _readRegister(THRESHOLD);       // Get the current value of the register
			    _spiWrite &= ~NOISE_FLOOR_MASK;               // Mask the position we want to write to
    		    _spiWrite >>= 4; // Write the given bits to the variable


    return(_spiWrite);
    //return (regVal/8);
    //return (regVal & ~NOISE_FLOOR_MASK) >> 4;
}

// REG0x02, bits [3:0], manufacturer default: 0010 (2).
// This setting, like the watchdog threshold, can help determine between false
// events and actual lightning. The shape of the spike is analyzed during the
// chip's signal validation routine. Increasing this value increases robustness
// at the cost of sensitivity to distant events.
void spikeRejection(uint8_t _spSensitivity)
{
    if (_spSensitivity > 15)
        return;

    _writeRegister(LIGHTNING_REG, SPIKE_MASK, _spSensitivity, 0);
}

// REG0x02, bits [3:0], manufacturer default: 0010 (2).
// This function returns the value of the spike rejection register. This value
// helps to differentiate between events and acutal lightning, by analyzing the
// shape of the spike during  chip's signal validation routine.
// Increasing this value increases robustness at the cost of sensitivity to distant events.
uint8_t readSpikeRejection()
{

    uint8_t regVal = _readRegister(LIGHTNING_REG);
    //return (regVal &= ~SPIKE_MASK);
    return (regVal);
}
// REG0x02, bits [5:4], manufacturer default: 0 (single lightning strike).
// The number of lightning events before IRQ is set high. 15 minutes is The
// window of time before the number of detected lightning events is reset.
// The number of lightning strikes can be set to 1,5,9, or 16.
void lightningThreshold(uint8_t _strikes)
{

    uint8_t bits;

    if (_strikes == 1)
        bits = 0;
    else if (_strikes == 5)
        bits = 1;
    else if (_strikes == 9)
        bits = 2;
    else if (_strikes == 16)
        bits = 3;
    else
        return;

    _writeRegister(LIGHTNING_REG, LIGHT_MASK, bits, 4);
}

// REG0x02, bits [5:4], manufacturer default: 0 (single lightning strike).
// This function will return the number of lightning strikes must strike within
// a 15 minute window before it triggers an event on the IRQ pin. Default is 1.
uint8_t readLightningThreshold()
{

    uint8_t regVal = _readRegister(LIGHTNING_REG);

    regVal &= ~LIGHT_MASK;
    regVal >>= 4; // Front of the line.

    if (regVal == 0)
        return 1;
    else if (regVal == 1)
        return 5;
    else if (regVal == 2)
        return 9;
    else if (regVal == 3)
        return 16;
    else
        return regVal;
}

// REG0x02, bit [6], manufacturer default: 1.
// This register clears the number of lightning strikes that has been read in
// the last 15 minute block.
void clearStatistics(bool _clearStat)
{
    if (_clearStat != true)
        return;
    // Write high, then low, then high to clear.
    _writeRegister(LIGHTNING_REG, STAT_MASK, 1, 6);
    _writeRegister(LIGHTNING_REG, STAT_MASK, 0, 6);
    _writeRegister(LIGHTNING_REG, STAT_MASK, 1, 6);
}

// REG0x03, bits [3:0], manufacturer default: 0.
// When there is an event that exceeds the watchdog threshold, the register is written
// with the type of event. This consists of two messages: INT_D (disturber detected) and
// INT_L (Lightning detected). A third interrupt INT_NH (noise level too HIGH)
// indicates that the noise level has been exceeded and will persist until the
// noise has ended. Events are active HIGH. There is a one second window of time to
// read the interrupt register after lightning is detected, and 1.5 after
// disturber.
uint8_t readInterruptReg()
{
    // A 2ms delay is added to allow for the memory register to be populated
    // after the interrupt pin goes HIGH. See "Interrupt Management" in
    // datasheet.
    HAL_Delay(2);

    uint8_t _interValue;
    _interValue = _readRegister(INT_MASK_ANT);
    _interValue &= INT_MASK;

    return (_interValue);
}

// REG0x03, bit [5], manufacturere default: 0.
// This setting will change whether or not disturbers trigger the IRQ Pin.
void maskDisturber(bool _state)
{
    _writeRegister(INT_MASK_ANT, DISTURB_MASK, _state, 5);
}

// REG0x03, bit [5], manufacturere default: 0.
// This setting will return whether or not disturbers trigger the IRQ Pin.
uint8_t readMaskDisturber()
{

    uint8_t regVal = _readRegister(INT_MASK_ANT);

    return (regVal &= ~DISTURB_MASK) >> 5;
}

// REG0x03, bit [7:6], manufacturer default: 0 (16 division ratio).
// The antenna is designed to resonate at 500kHz and so can be tuned with the
// following setting. The accuracy of the antenna must be within 3.5 percent of
// that value for proper signal validation and distance estimation.
void changeDivRatio(uint8_t _divisionRatio)
{

    uint8_t bits;

    if (_divisionRatio == 16)
        bits = 0;
    else if (_divisionRatio == 32)
        bits = 1;
    else if (_divisionRatio == 64)
        bits = 2;
    else if (_divisionRatio == 128)
        bits = 3;
    else
        return;

    _writeRegister(INT_MASK_ANT, DIV_MASK, bits, 6);
}

// REG0x03, bit [7:6], manufacturer default: 0 (16 division ratio).
// This function returns the current division ratio of the resonance frequency.
// The antenna resonance frequency should be within 3.5 percent of 500kHz, and
// so when modifying the resonance frequency with the internal capacitors
// (tuneCap()) it's important to keep in mind that the displayed frequency on
// the IRQ pin is divided by this number.
uint8_t readDivRatio()
{

    uint8_t regVal = _readRegister(INT_MASK_ANT);
    regVal &= ~DIV_MASK;
    regVal >>= 6; // Front of the line.

    if (regVal == 0)
        return 16;
    else if (regVal == 1)
        return 32;
    else if (regVal == 2)
        return 64;
    else if (regVal == 3)
        return 128;
    else
        return UNKNOWN_ERROR;
}

// REG0x07, bit [5:0], manufacturer default: 0.
// This register holds the distance to the front of the storm and not the
// distance to a lightning strike.
uint8_t distanceToStorm()
{

    uint8_t _dist = _readRegister(DISTANCE);
    _dist &= DISTANCE_MASK;
    return (_dist);
}

// REG0x08, bits [5,6,7], manufacturer default: 0.
// This will send the frequency of the oscillators to the IRQ pin.
//  _osc 1, bit[5] = TRCO - System RCO at 32.768kHz
//  _osc 2, bit[6] = SRCO - Timer RCO Oscillators 1.1MHz
//  _osc 3, bit[7] = LCO - Frequency of the Antenna
void displayOscillator(bool _state, uint8_t _osc)
{
    if (_osc > 3)
        return;

    if (_state == true)
    {
        if (_osc == 1)
            _writeRegister(FREQ_DISP_IRQ, OSC_MASK, 1, 5);
        if (_osc == 2)
            _writeRegister(FREQ_DISP_IRQ, OSC_MASK, 1, 6);
        if (_osc == 3)
            _writeRegister(FREQ_DISP_IRQ, OSC_MASK, 1, 7);
    }

    if (_state == false)
    {
        if (_osc == 1)
            _writeRegister(FREQ_DISP_IRQ, OSC_MASK, 0, 5); // Demonstrative
        if (_osc == 2)
            _writeRegister(FREQ_DISP_IRQ, OSC_MASK, 0, 6);
        if (_osc == 3)
            _writeRegister(FREQ_DISP_IRQ, OSC_MASK, 0, 7);
    }
}

// REG0x08, bits [3:0], manufacturer default: 0.
// This setting will add capacitance to the series RLC antenna on the product
// to help tune its resonance. The datasheet specifies being within 3.5 percent
// of 500kHz to get optimal lightning detection and distance sensing.
// It's possible to add up to 120pF in steps of 8pF to the antenna.
void tuneCap(uint8_t farad)
{
    if (farad > 120)
        return;
    else if (farad % 8 != 0)
        return;
    else
        farad /= 8;

    _writeRegister(FREQ_DISP_IRQ, CAP_MASK, farad, 0);
}

// REG0x08, bits [3:0], manufacturer default: 0.
// This setting will return the capacitance of the internal capacitors. It will
// return a value from one to 15 multiplied by the 8pF steps of the internal
// capacitance.
uint8_t readTuneCap()
{

    uint8_t regVal = _readRegister(FREQ_DISP_IRQ);
    return ((regVal &= ~CAP_MASK) * 8); // Multiplied by 8pF
}

// LSB =  REG0x04, bits[7:0]
// MSB =  REG0x05, bits[7:0]
// MMSB = REG0x06, bits[4:0]
// This returns a 20 bit value that is the 'energy' of the lightning strike.
// According to the datasheet this is only a pure value that doesn't have any
// physical meaning.
uint32_t lightningEnergy()
{

    uint32_t _pureLight = _readRegister(ENERGY_LIGHT_MMSB);
    _pureLight &= ENERGY_MASK;
    _pureLight <<= 8;
    _pureLight |= _readRegister(ENERGY_LIGHT_MSB);
    _pureLight <<= 8;
    _pureLight |= _readRegister(ENERGY_LIGHT_LSB);
    return _pureLight;
}

// REG0x3D, bits[7:0]
// This function calibrates both internal oscillators The oscillators are tuned
// based on the resonance frequency of the antenna and so it should be trimmed
// before the calibration is done.
bool calibrateOsc()
{

    _writeRegister(CALIB_RCO, WIPE_ALL, DIRECT_COMMAND, 0); // Send command to calibrate the oscillators
   // Serial.println("Calibrating Oscillators");
    UART_Log("Calibrating Oscillators\r\n");

    displayOscillator(true, 2);
    HAL_Delay(2); // Give time for the internal oscillators to start up.
    displayOscillator(false, 2);

    // Check it they were calibrated successfully.
    uint8_t regValSrco = _readRegister(CALIB_SRCO);
    uint8_t regValTrco = _readRegister(CALIB_TRCO);

    regValSrco &= CALIB_MASK;
    regValSrco >>= 6;
    regValTrco &= CALIB_MASK;
    regValTrco >>= 6;

    if (!regValSrco && !regValTrco) // Zero upon success
        return true;
    else
        return false;
}

// REG0x3C, bits[7:0]
// This function resets all settings to their default values.
void resetSettings()
{

    _writeRegister(RESET_LIGHT, WIPE_ALL, DIRECT_COMMAND, 0);
}

// This function handles all I2C write commands. It takes the register to write
// to, then will mask the part of the register that coincides with the
// given register, and then write the given bits to the register starting at
// the given start position.
void _writeRegister(uint8_t _wReg, uint8_t _mask, uint8_t _bits, uint8_t _startPosition)
{

	      //uint8_t bits;
          uint8_t _spiWrite;

		    _spiWrite = _readRegister(_wReg);       // Get the current value of the register
		    _spiWrite &= _mask;                     // Mask the position we want to write to
		    _spiWrite |= (_bits << _startPosition); // Write the given bits to the variable


		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
	    uint8_t data[2] = {_wReg , _spiWrite}; // 0x00 to indicate write operation
	    HAL_SPI_Transmit(&hspi1, data, 2, HAL_MAX_DELAY);
		//HAL_SPI_Transmit(&hspi1, &_wReg, 1, HAL_MAX_DELAY);
		//HAL_SPI_Transmit(&hspi1, &_spiWrite, 1, HAL_MAX_DELAY);
	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
	    //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
	   // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);

      /*  _spiWrite = _readRegister(_wReg);       // Get the current value of the register
        _spiWrite &= _mask;                     // Mask the position we want to write to
        _spiWrite |= (_bits << _startPosition); // Write the given bits to the variable
        _spiPort->beginTransaction(mySpiSettings);
        digitalWrite(_cs, LOW);        // Start communication
        _spiPort->transfer(_wReg);     // Start write command at given register
        _spiPort->transfer(_spiWrite); // Write to register
        digitalWrite(_cs, HIGH);       // End communcation
        _spiPort->endTransaction();*/
	    /*char uart_buf3[50];
	    int uart_buf_len3;
	    uart_buf_len3 = sprintf(uart_buf3,"SPI spi write Register: %u\r\n",_spiWrite);
	    	    HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf3, uart_buf_len3, 100);
	    	    return (value);*/
}

// This function reads the given register.
uint8_t _readRegister(uint8_t _reg)
{

		 _reg |= SPI_READ_M; // Adjust for read operation, check datasheet for correct operation
		//extern uint8_t value;
	    //char uart_buf1[50];
	    //int uart_buf_len1;
	    //uint8_t data1[1] = {_reg};
	    //uart_buf_len1 = sprintf(uart_buf1,"SPI Read _reg: %x\r\n",_reg);
	   // HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf1, uart_buf_len1, 100);


	    //char spi_buf1[20];
	    //char spi_read1[20];
	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
	    HAL_SPI_Transmit(&hspi1, &_reg, 1, HAL_MAX_DELAY);
	    HAL_SPI_Receive(&hspi1, &value, 2, HAL_MAX_DELAY);
	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
	    //uart_buf_len = sprintf(value, "SPI Read Register\r\n");
	    //HAL_UART_Transmit(&huart1, (uint8_t *)value, uart_buf_len, 100);
	    //return (value);
	    //uart_buf_len1 = sprintf(uart_buf1,"SPI Read Register: %x\r\n",value);
	    //HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf1, uart_buf_len1, 100);
	    return (value);


     /*   _spiPort->beginTransaction(mySpiSettings);
        digitalWrite(_cs, LOW);                 // Start communication.
        _spiPort->transfer(_reg |= SPI_READ_M); // Register OR'ed with SPI read command.
        _regValue = _spiPort->transfer(0);      // Get data from register.
        // According to datsheet, the chip select must be written HIGH, LOW, HIGH
        // to correctly end the READ command.
        digitalWrite(_cs, HIGH);
        digitalWrite(_cs, LOW);
        digitalWrite(_cs, HIGH);
        _spiPort->endTransaction();
        return (_regValue);*/
}

void UART_Log(char *message) {
    HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), 100);
}

int sendvalue(){
	return(34);
}

