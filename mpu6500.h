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

} MPU6500_OutputTypeDef;

typedef struct {

	I2C_HandleTypeDef *hi2c;
	
	uint32_t initialized;

	volatile uint8_t data_ready; /* set to 1 by IntCallback. Must be set back to
									0 by whatever handles reading new data */
} MPU6500_HandleTypeDef;

/* Public Prototypes */
int MPU6500_Init(MPU6500_HandleTypeDef *dev);
int MPU6500_GetAccel(MPU6500_HandleTypeDef *dev, MPU6500_OutputTypeDef *out);
int MPU6500_GetGyro(MPU6500_HandleTypeDef *dev, MPU6500_OutputTypeDef *out);
int MPU6500_GetTemp(MPU6500_HandleTypeDef *dev, MPU6500_OutputTypeDef *out);
void MPU6500_IntCallback(MPU6500_HandleTypeDef * dev);
