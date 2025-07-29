/**
 ******************************************************************************
 * @file    ASM330LHHSensor.h
 * @author  SRA
 * @version V1.0.0
 * @date    March 2020
 * @brief   Abstract Class of an ASM330LHH Automotive IMU 6 axes
 *          sensor.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2020 STMicroelectronics</center></h2>
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


/* Prevent recursive inclusion -----------------------------------------------*/

#ifndef __ASM330LHHSensor_H__
#define __ASM330LHHSensor_H__


/* Includes ------------------------------------------------------------------*/

//#include "Wire.h"
//#include "SPI.h"
#include "asm330lhh_reg.h"
#include "stdbool.h"
//#include "peripherals_lpi2c_config_1.h"
//#include "lpi2c_driver.h"

/* Defines -------------------------------------------------------------------*/

#define ASM330LHH_ACC_SENSITIVITY_FS_2G   0.061f
#define ASM330LHH_ACC_SENSITIVITY_FS_4G   0.122f
#define ASM330LHH_ACC_SENSITIVITY_FS_8G   0.244f
#define ASM330LHH_ACC_SENSITIVITY_FS_16G  0.488f

#define ASM330LHH_GYRO_SENSITIVITY_FS_125DPS    4.370f
#define ASM330LHH_GYRO_SENSITIVITY_FS_250DPS    8.750f
#define ASM330LHH_GYRO_SENSITIVITY_FS_500DPS   17.500f
#define ASM330LHH_GYRO_SENSITIVITY_FS_1000DPS  35.000f
#define ASM330LHH_GYRO_SENSITIVITY_FS_2000DPS  70.000f
#define ASM330LHH_GYRO_SENSITIVITY_FS_4000DPS 140.000f


/* Typedefs ------------------------------------------------------------------*/

typedef enum
{
  ASM330LHH_OK = 0,
  ASM330LHH_ERROR =-1
} ASM330LHHStatusTypeDef;


/* Class Declaration ---------------------------------------------------------*/
   
/**
 * Abstract class of an ASM330LHH Inertial Measurement Unit (IMU) 3 axes
 * sensor.
 */

	ASM330LHHStatusTypeDef ASM330LHHSensor_ASM330LHHSensor(uint8_t address);
  //  ASM330LHHSensor(SPIClass *spi, int cs_pin, uint32_t spi_speed=2000000);
    ASM330LHHStatusTypeDef ASM330LHHSensor_begin();
    ASM330LHHStatusTypeDef ASM330LHHSensor_end();
    void ASM330LHHSensor_ReadID(uint8_t *Id);
    ASM330LHHStatusTypeDef ASM330LHHSensor_Enable_X();
    ASM330LHHStatusTypeDef ASM330LHHSensor_Disable_X();
    void ASM330LHHSensor_Get_X_Sensitivity(float *Sensitivity);
    ASM330LHHStatusTypeDef ASM330LHHSensor_Get_X_ODR(float *Odr);
    ASM330LHHStatusTypeDef ASM330LHHSensor_Set_X_ODR(float Odr);
    ASM330LHHStatusTypeDef ASM330LHHSensor_Get_X_FS(int32_t *FullScale);
    ASM330LHHStatusTypeDef ASM330LHHSensor_Set_X_FS(int32_t FullScale);
    ASM330LHHStatusTypeDef ASM330LHHSensor_Get_X_AxesRaw(int16_t *Value);
    void ASM330LHHSensor_Get_X_Axes(int32_t *Acceleration);
    ASM330LHHStatusTypeDef ASM330LHHSensor_Get_X_DRDY_Status(uint8_t *Status);
    
    ASM330LHHStatusTypeDef ASM330LHHSensor_Enable_G();
    ASM330LHHStatusTypeDef ASM330LHHSensor_Disable_G();
    ASM330LHHStatusTypeDef ASM330LHHSensor_Get_G_Sensitivity(float *Sensitivity);
    ASM330LHHStatusTypeDef ASM330LHHSensor_Get_G_ODR(float *Odr);
    ASM330LHHStatusTypeDef ASM330LHHSensor_Set_G_ODR(float Odr);
    ASM330LHHStatusTypeDef ASM330LHHSensor_Get_G_FS(int32_t *FullScale);
    ASM330LHHStatusTypeDef ASM330LHHSensor_Set_G_FS(int32_t FullScale);
    ASM330LHHStatusTypeDef ASM330LHHSensor_Get_G_AxesRaw(int16_t *Value);
    ASM330LHHStatusTypeDef ASM330LHHSensor_Get_G_Axes(int32_t *AngularRate);
    ASM330LHHStatusTypeDef ASM330LHHSensor_Get_G_DRDY_Status(uint8_t *Status);
    
    ASM330LHHStatusTypeDef ASM330LHHSensor_Read_Reg(uint8_t reg, uint8_t *Data);
    ASM330LHHStatusTypeDef ASM330LHHSensor_Write_Reg(uint8_t reg, uint8_t Data);
    void ASM330LHHSensor_ReadID1(uint8_t *Id);
    void Temp_read(int16_t *tempd);
    /**
     * @brief Utility function to read data.
     * @param  pBuffer: pointer to data to be read.
     * @param  RegisterAddr: specifies internal address register to be read.
     * @param  NumByteToRead: number of bytes to be read.
     * @retval 0 if ok, an error code otherwise.
     */
    uint8_t IO_Read(uint8_t* pBuffer, uint8_t RegisterAddr, uint16_t NumByteToRead);

    uint8_t IO_Write(uint8_t* pBuffer, uint8_t RegisterAddr, uint16_t NumByteToWrite);



void ASM330LHHSensor_Get_X_Axes1(int32_t *Acceleration);
#ifdef __cplusplus
 extern "C" {
#endif
int32_t ASM330LHH_io_write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite );
int32_t ASM330LHH_io_read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead );
#ifdef __cplusplus
  }
#endif

#endif
