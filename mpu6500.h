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

#ifndef MPU_H
#define MPU_H

typedef struct {
	float accel_xout;
	float accel_yout;
	float accel_zout;
	float temp_out;
	float gyro_xout;
	float gyro_yout;
	float gyro_zout;

} MPU6500_OutputTypeDef;

typedef struct {

	I2C_HandleTypeDef *hi2c;
	
	uint32_t initialized;

	volatile uint8_t data_ready; /* set to 1 by IntCallback. Must be set back to
									0 by whatever handles reading new data. */
	uint8_t gyro_config; /* Contains test bits, fs select and filter bypass */

	uint8_t accel_config;
	
	uint8_t sample_rate_div; /* The value of SMPLRT_DIV. Defaults to 0.
								Internal sample rate = 1kHz/(1+sample_rate_div)
								when fchoice_b = 0b00 and 0 < dlpf_cfg < 7. */
	uint8_t dlpf_cfg;        /* Digital low pass filter config.
								Initialized to 4. */
	uint8_t pwr_mgmt_1; /* 0x01 (Awake, PLL) on reset */
} MPU6500_HandleTypeDef;

/* Public Functions --------------------------------------------------------- */

/* MPU-6500 Initialize:
 *
 * Prerequisites:
 * - dev must be allocated and zeroed
 * - hi2c field must be set
 *
 * Initializes device context, checks if device responds, checks if device is a
 * BME-6000, and resets device.
 *
 * Note: On init, device is awake, using PLL, does not generate interrupts, has
 *       a sample rate of 1kHz, and the digital low pass filter set to 4.
 */
int MPU6500_Init(MPU6500_HandleTypeDef *dev);

/* MPU6500 Enable Interrupts:
 *
 * Enables raw data ready interrupts
 */
int MPU6500_EnableInterrupts(MPU6500_HandleTypeDef *dev);

/* MPU6500 Disable Interrupts:
 *
 * Disables all interrupts from the MPU6500
 */
int MPU6500_DisableInterrupts(MPU6500_HandleTypeDef *dev);

/* MPU6500 Interrupt Callback:
 *
 * Hook this into HAL_GPIO_EXTI_Callback() for the GPIO pin INT is connected to.
 *
 * Sets data_ready in the device context to 1.
 *
 * The code that handles reading data from the MPU6500 (after the interrupt)
 * must handle setting data_ready back to 0.
 *
 * This function fails silently!!!
 */
void MPU6500_IntCallback(MPU6500_HandleTypeDef * dev);

/* MPU6500 Awake:
 *
 * Sets the power management mode to awake
 */
int MPU6500_Awake(MPU6500_HandleTypeDef *dev);

/* MPU6500 Sleep:
 *
 * Sets the power management mode to sleep
 */
int MPU6500_Sleep(MPU6500_HandleTypeDef *dev);

/* MPU6500 Temperature Disable:
 *
 * Disables the temperature sensor by writing to the PWR_MGMT_1 register
 */
int MPU6500_TempDisable(MPU6500_HandleTypeDef *dev);

/* MPU6500 Temperature Enables:
 *
 * Enables the temperature sensor by writing to the PWR_MGMT_1 register
 */
int MPU6500_TempEnable(MPU6500_HandleTypeDef *dev);

/* MPU6500 Set Sample Rate Divider:
 *
 * Sets the sample rate where sample rate = 1kHz / ( 1 + div )
 * This is only effective when FCHOICE_B = 0b00 and 0 < DLPF_CFG < 7
 */
int MPU6500_SetSampleRateDiv(MPU6500_HandleTypeDef *dev, uint8_t div);

int MPU6500_SetAccelScale(MPU6500_HandleTypeDef *dev, uint8_t selection);
int MPU6500_GetAccel(MPU6500_HandleTypeDef *dev, MPU6500_OutputTypeDef *out);

int MPU6500_SetGyroScale(MPU6500_HandleTypeDef *dev, uint8_t selection);
int MPU6500_GetGyro(MPU6500_HandleTypeDef *dev, MPU6500_OutputTypeDef *out);

/* MPU6500 Get Temperature:
 *
 * Sets the temp_out field out to the device temperature in degrees celsius.
 *
 * Returns 0 on success, -1 on failure.
 *
 * Fails if:
 *   - dev is NULL
 *   - dev is not initialized
 *   - out is NULL
 *   - temperature sensor is disabled
 *   - I2C communication fails
 */
int MPU6500_GetTemp(MPU6500_HandleTypeDef *dev, MPU6500_OutputTypeDef *out);

#endif
