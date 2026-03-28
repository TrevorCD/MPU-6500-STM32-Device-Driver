/* mpu6500.h
 *
 * Device driver header for MPU-6500 Accelerometer. Depends on stm32's HAL.
 *
 *-----------------------------------------------------------------------------
 *
 *   Copyright 2026 Trevor B. Calderwood
 *
 *   Licensed under the Apache License, Version 2.0 (the "License");
 *   you may not use this file except in compliance with the License.
 *   You may obtain a copy of the License at
 *
 *       http://www.apache.org/licenses/LICENSE-2.0
 *
 *   Unless required by applicable law or agreed to in writing, software
 *   distributed under the License is distributed on an "AS IS" BASIS,
 *   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *   See the License for the specific language governing permissions and
 *   limitations under the License.
 *
 *----------------------------------------------------------------------------*/

/* Hardware Includes */
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

typedef struct {
	uint16_t accel_xout;
	uint16_t accel_yout;
	uint16_t accel_zout;
	uint16_t temp_out;
	uint16_t gyro_xout;
	uint16_t gyro_yout;
	uint16_t gyro_zout;

} MPU_OutputTypeDef;

typedef struct {

	I2C_HandleTypeDef *hi2c;
	
	uint32_t initialized;

	uint8_t data_ready;
	
} MPU_HandleTypeDef;

/* Public Prototypes */
int MPU_Init(MPU_HandleTypeDef *dev);
int MPU_GetAccel(MPU_HandleTypeDef *dev, MPU_OutputTypeDef *out);
int MPU_GetGyro(MPU_HandleTypeDef *dev, MPU_OutputTypeDef *out);
int MPU_GetTemp(MPU_HandleTypeDef *dev, MPU_OutputTypeDef *out);
