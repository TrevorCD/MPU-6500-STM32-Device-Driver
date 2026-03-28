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

	I2C_HandleTypeDef *hi2c;

} MPU6500_HandleTypeDef;

/* Public Prototypes */
int MPU_Init(MPU_HandleTypeDef *dev);
int MPU_Read(MPU_HandleTypeDef *dev, uint8_t *data);
int MPU_Write(MPU_HandleTypeDef *dev, uint8_t *data);
int MPU_Ioctl(MPU_HandleTypeDef *dev, uint8_t *data, uint8_t type);
