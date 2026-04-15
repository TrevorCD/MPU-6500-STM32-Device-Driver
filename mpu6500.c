/* mpu6500.c
 *
 * Device driver source for MPU-6500 Accelerometer in I2C mode.
 * Depends on stm32's HAL.
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

#include "mpu6500.h"

/* MPU-6500 Config -----------------------------------------------------------*/
#define MPU6500_TIMEOUT             (1000) /* timeout for i2c read in ms */

/* MPU-6500 Constants --------------------------------------------------------*/
#define ROOM_TEMP_OFFSET            (0.0f)
#define TEMP_SENSITIVITY            (333.87f)

/* MPU-6500 Registers --------------------------------------------------------*/
#define MPU6500_SELF_TEST_X_GYRO    (0x00)
#define MPU6500_SELF_TEST_Y_GYRO    (0x01)
#define MPU6500_SELF_TEST_Z_GYRO    (0x02)
#define MPU6500_SELF_TEST_X_ACCEL   (0x0D)
#define MPU6500_SELF_TEST_Y_ACCEL   (0x0E)
#define MPU6500_SELF_TEST_Z_ACCEL   (0x0F)
#define MPU6500_XG_OFFSET_H         (0x13) /* X_OFFS_USR        [15:8] */
#define MPU6500_XG_OFFSET_L         (0x14) /* X_OFFS_USR        [7:0]  */
#define MPU6500_YG_OFFSET_H         (0x15) /* Y_OFFS_USR        [15:8] */
#define MPU6500_YG_OFFSET_L         (0x16) /* Y_OFFS_USR        [7:0]  */
#define MPU6500_ZG_OFFSET_H         (0x17) /* Z_OFFS_USR        [15:8] */
#define MPU6500_ZG_OFFSET_L         (0x18) /* Z_OFFS_USR        [7:0]  */
#define MPU6500_SMPLRT_DIV          (0x19)
#define MPU6500_CONFIG              (0x1A) /* FIFO_MODE         [6]
											  EXT_SYNC_SET      [4:3]
											  DLPF_CFG          [2:0]  */
#define MPU6500_GYRO_CONFIG         (0x1B) /* XG_ST             [7]
											  YG_ST             [6]
											  ZG_ST             [5]
											  GYRO_FS_SEL       [4:3]
											  FCHOICE_B         [1:0]  */
#define MPU6500_ACCEL_CONFIG        (0x1C) /* XA_ST             [7]
											  YA_ST             [6]
											  ZA_ST             [5]
											  ACCEL_FS_SEL      [4:3]  */
#define MPU6500_ACCEL_CONFIG2       (0x1D) /* ACCEL_FC_HOICE_B  [3]
											  A_DLPF_CFG        [2:0]  */
#define MPU6500_LP_ACCEL_ODR        (0x1E) /* LPOSC_CLKSEL      [3:0]  */
#define MPU6500_WOM_THR             (0x1F)
#define MPU6500_FIFO_EN             (0x23) /* TEMP_FIFO_EN      [7]
											  GYRO_XOUT         [6]
											  GYRO_YOUT         [5]
											  GYRO_ZOUT         [4]
											  ACCEL             [3]
											  SLV2              [2]
											  SLV1              [1]
											  SLV0              [0]    */
#define MPU6500_INT_PIN_CFG         (0x37) /* ACTL              [7]
											  OPEN              [6]
											  LATCH_INT_EN      [5]
											  INT_ANYRD_2CLEAR  [4]
											  ACTL_FSY_EN       [3]
											  FSYNC_INT_MODE_EN [2]
											  BYPASS_EN         [1]    */
#define MPU6500_INT_ENABLE          (0x38) /* WOM_EN            [6]
											  FIFO_OFLOW_EN     [4]
											  FSYNC_INT_EN      [3]
											  RAW_RDY_EN        [0]    */
#define MPU6500_INT_STATUS          (0x3A) /* WOM_INT           [6]
											  FIFO_OFLOW_INT    [4]
											  FSYNC_INT         [3]
											  DMP_INT           [1]
											  RAW_DATA_RDY_INT  [0]    */
#define MPU6500_ACCEL_XOUT_H        (0x3B)
#define MPU6500_ACCEL_XOUT_L        (0x3C)
#define MPU6500_ACCEL_YOUT_H        (0x3D)
#define MPU6500_ACCEL_YOUT_L        (0x3E)
#define MPU6500_ACCEL_ZOUT_H        (0x3F)
#define MPU6500_ACCEL_ZOUT_L        (0x40)
#define MPU6500_TEMP_OUT_H          (0x41)
#define MPU6500_TEMP_OUT_L          (0x42)
#define MPU6500_GYRO_XOUT_H         (0x43)
#define MPU6500_GYRO_XOUT_L         (0x44)
#define MPU6500_GYRO_YOUT_H         (0x45)
#define MPU6500_GYRO_YOUT_L         (0x46)
#define MPU6500_GYRO_ZOUT_H         (0x47)
#define MPU6500_GYRO_ZOUT_L         (0x48)
#define MPU6500_SIGNAL_PATH_REST    (0x68) /* GYRO_RST          [2]
											  ACCEL_RST         [1]
											  TEMP_RST          [0]    */
#define MPU6500_ACCEL_INTEL_CTRL    (0x69) /* ACCEL_INT_EL_EN   [7]
											  ACCEL_INT_EL_MODE [6]    */
#define MPU6500_USER_CTRL           (0x6A) /* DMP_EN            [7]
											  FIFO_EN           [6]
											  I2C_MST_EN        [5]
											  I2C_IF_DIS        [4]
											  DMP_RST           [3]
											  FIFO_RST          [2]
											  I2C_MST_RST       [1]
											  SIG_COND_RST      [0]    */
/*----------------------------------------------------------------------------*/
#define MPU6500_PWR_MGMT_1          (0x6B) /* DEVICE_RESET      [7]
											  SLEEP             [6]
											  CYCLE             [5]
											  GYRO_STANDBY      [4]
											  TEMP_DIS          [3]
											  CLKSEL            [2:0]  */
/* PWR_MGMT_1 Bits: */
#define MPU6500_DEVICE_RESET        (0x80)
#define MPU6500_SLEEP               (0x40)
#define MPU6500_CYCLE               (0x20)
#define MPU6500_GYRO_STANDBY        (0x10)
#define MPU6500_TEMP_DIS            (0x08)
/*----------------------------------------------------------------------------*/
#define MPU6500_PWR_MGMT_2          (0x6C) /* DIS_XA            [5]
											  DIS_YA            [4]
											  DIS_ZA            [3]
											  DIS_XG            [2]
											  DIS_YG            [1]
											  DIS_ZG            [0]    */
#define MPU6500_FIFO_COUNT_H        (0x72) /* FIFO_CNT[12:8]    [4:0]  */
#define MPU6500_FIFO_COUNT_L        (0x73) /* FIFO_CNT[7:0]     [7:0]  */
#define MPU6500_FIFO_R_W            (0x74)
#define MPU6500_WHO_AM_I            (0x75)
#define MPU6500_XA_OFFSET_H         (0x77)
#define MPU6500_XA_OFFSET_L         (0x78)
#define MPU6500_YA_OFFSET_H         (0x7A)
#define MPU6500_YA_OFFSET_L         (0x7B)
#define MPU6500_ZA_OFFSET_H         (0x7D)
#define MPU6500_ZA_OFFSET_L         (0x7E)

/* Public Prototypes ---------------------------------------------------------*/
int MPU6500_Init_I2C(MPU6500_handle_t *dev, I2C_HandleTypeDef *hi2c,
					 MPU6500_addr_t addr);
int  MPU6500_Init_SPI(MPU6500_handle_t *dev, SPI_HandleTypeDef *hspi);
int  MPU6500_EnableInterrupts(MPU6500_handle_t *dev);
int  MPU6500_DisableInterrupts(MPU6500_handle_t *dev);
void MPU6500_IntCallback(MPU6500_handle_t * dev);
int  MPU6500_Awake(MPU6500_handle_t *dev);
int  MPU6500_Sleep(MPU6500_handle_t *dev);
int  MPU6500_TempDisable(MPU6500_handle_t *dev);
int  MPU6500_TempEnable(MPU6500_handle_t *dev);
int  MPU6500_SetSampleRateDiv(MPU6500_handle_t *dev, uint8_t div);
int  MPU6500_SetAccelScale(MPU6500_handle_t *dev, uint8_t selection);
int  MPU6500_GetAccel(MPU6500_handle_t *dev, MPU6500_output_t *out);
int  MPU6500_SetGyroScale(MPU6500_handle_t *dev, uint8_t selection);
int  MPU6500_GetGyro(MPU6500_handle_t *dev, MPU6500_output_t *out);
int  MPU6500_GetTemp(MPU6500_handle_t *dev, MPU6500_output_t *out);

/* Private Prototypes --------------------------------------------------------*/
static int MPU6500_Init(MPU6500_handle_t *dev);
static int MPU6500_Read_I2C(MPU6500_handle_t *dev, uint8_t reg, uint8_t *data);
static int MPU6500_Write_I2C(MPU6500_handle_t *dev, uint8_t reg, uint8_t data);
static int MPU6500_Read_SPI(MPU6500_handle_t *dev, uint8_t reg, uint8_t *data);
static int MPU6500_Write_SPI(MPU6500_handle_t *dev, uint8_t reg, uint8_t data);

/* Public Functions ----------------------------------------------------------*/

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
int MPU6500_Init_I2C(MPU6500_handle_t *dev, I2C_HandleTypeDef *hi2c,
					 MPU6500_addr_t addr) {

	HAL_StatusTypeDef status;

	/* Validate arguments */
	if(dev == NULL) return -1;
	if(dev->initialized != 0) return -1;
	if(hi2c == NULL) return -1;
	if(addr != MPU6500_ADDR_LOW && addr != MPU6500_ADDR_HIGH) return -1;
	
	/* Set communication fields in dev context */
	dev->hi2c = hi2c;
	dev->addr = addr;
	dev->comm_ops.write = MPU6500_Write_I2C;
	dev->comm_ops.read = MPU6500_Read_I2C;

	/* Check for ACK from device */
	status = HAL_I2C_IsDeviceReady(dev->hi2c, addr, 10, MPU6500_TIMEOUT);
	if(status != HAL_OK) return -1;

	/* Finish initialization */
	return MPU6500_Init(dev);
}

int MPU6500_Init_SPI(MPU6500_handle_t *dev, SPI_HandleTypeDef *hspi) {
	
	/* Validate arguments */
	if(dev == NULL) return -1;
	if(dev->initialized != 0) return -1;
	if(hspi == NULL) return -1;
	
	/* Set communication fields in dev context */
	dev->hspi = hspi;
	/* device address is 0 on SPI mode */
	dev->comm_ops.write = MPU6500_Write_SPI;
	dev->comm_ops.read = MPU6500_Read_SPI;

	/* Finish initialization */
	return MPU6500_Init(dev);
}

/* Interrupt Management and Callback -----------------------------------------*/

/* MPU6500 Enable Interrupts:
 *
 * Enables raw data ready interrupts
 */
int MPU6500_EnableInterrupts(MPU6500_handle_t *dev) {
	if(dev == NULL) return -1;
	if(dev->initialized != 1) return -1;
	/* Enable raw data ready interrupts (MPU6500_INT_ENABLE[bit 0] == 1) */
	if(MPU6500_Write_I2C(dev, MPU6500_INT_ENABLE, 1) != 0) return -1;
	return 0;
}

/* MPU6500 Disable Interrupts:
 *
 * Disables all interrupts from the MPU6500
 */
int MPU6500_DisableInterrupts(MPU6500_handle_t *dev) {
	if(dev == NULL) return -1;
	if(dev->initialized != 1) return -1;
	/* Disable all interrupts */
	if(MPU6500_Write_I2C(dev, MPU6500_INT_ENABLE, 0) != 0) return -1;
	return 0;
}

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
inline void MPU6500_IntCallback(MPU6500_handle_t *dev) {
	
	uint8_t int_status = 0;
	if(dev == NULL) return;
	if(dev->initialized != 1) return;
	/* Interrupt is cleared when INT_STATUS register is read */
	/* If INT_PIN_CFG[4] (INT_ANYRD_2CLEAR) is 1, any read clears interrupt */
	if(dev->comm_ops.read(dev, MPU6500_INT_STATUS, &int_status) == 0) {
		/// this switch statement may be a bad idea if multiple bits can be on
		switch(int_status) {
		case 1:
			dev->data_ready = 1;
			break;
		default:
			break;
		}
	}
}

/* Power Management ----------------------------------------------------------*/

/* MPU6500 Awake:
 *
 * Sets the power management mode to awake
 */
int MPU6500_Awake(MPU6500_handle_t *dev) {
	if(dev == NULL) return -1;
	if(dev->initialized != 1) return -1;
	/* Set sleep bit and cycle bit of PWR_MGMT_1 to 0 */
	dev->pwr_mgmt_1 &= ~(MPU6500_SLEEP | MPU6500_CYCLE);
	if(MPU6500_Write_I2C(dev, MPU6500_PWR_MGMT_1, dev->pwr_mgmt_1) != 0) {
		return -1;
	}
	return 0;
}

/* MPU6500 Sleep:
 *
 * Sets the power management mode to sleep
 */
int MPU6500_Sleep(MPU6500_handle_t *dev) {
	if(dev == NULL) return -1;
	if(dev->initialized != 1) return -1;
	/* Set sleep bit of PWR_MGMT_1 to 1 */
	dev->pwr_mgmt_1 |= 0x40;
	if(MPU6500_Write_I2C(dev, MPU6500_PWR_MGMT_1, dev->pwr_mgmt_1) != 0) {
		return -1;
	}
	return 0;
}

/* MPU6500 Temperature Disable:
 *
 * Disables the temperature sensor by writing to the PWR_MGMT_1 register
 */
int MPU6500_TempDisable(MPU6500_handle_t *dev) {
	if(dev == NULL) return -1;
	if(dev->initialized != 1) return -1;
	/* Set TEMP_DIS bit of power management register to 1 */
	dev->pwr_mgmt_1 |= MPU6500_TEMP_DIS;
	if(MPU6500_Write_I2C(dev, MPU6500_PWR_MGMT_1, dev->pwr_mgmt_1) != 0) {
		return -1;
	}
	return 0;
}

/* MPU6500 Temperature Enables:
 *
 * Enables the temperature sensor by writing to the PWR_MGMT_1 register
 */
int MPU6500_TempEnable(MPU6500_handle_t *dev) {
	if(dev == NULL) return -1;
	if(dev->initialized != 1) return -1;
	/* Set TEMP_DIS bit of power management register to 0  */
	dev->pwr_mgmt_1 &= ~((uint8_t)MPU6500_PWR_MGMT_1);
	if(MPU6500_Write_I2C(dev, MPU6500_PWR_MGMT_1, dev->pwr_mgmt_1) != 0) {
		return -1;
	}
	return 0;
}

/*----------------------------------------------------------------------------*/

/* MPU6500 Set Sample Rate Divider:
 *
 * Sets the sample rate where sample rate = 1kHz / ( 1 + div )
 * This is only effective when FCHOICE_B = 0b00 and 0 < DLPF_CFG < 7
 */
int MPU6500_SetSampleRateDiv(MPU6500_handle_t *dev, uint8_t div) {
	
	if(dev == NULL) return -1;
	if(dev->initialized != 1) return -1;

	/* If either of these conditions are true, SMPLRT_DIV register is not used
	   to calculate internal sample rate */
	/* gyro_config[1:0] is fchoice_b */
	if((dev->gyro_config & 0b11) != 0) return -1;
	if(dev->dlpf_cfg == 0 || dev->dlpf_cfg > 6) return -1;

	if(dev->sample_rate_div == div) return 0;

	if(MPU6500_Write_I2C(dev, MPU6500_SMPLRT_DIV, div) != 0) return -1;

	dev->sample_rate_div = div;
	return 0;
}

/* Accelerometer Functions ---------------------------------------------------*/

int MPU6500_SetAccelScale(MPU6500_handle_t *dev, uint8_t selection) {

	if(dev == NULL) return -1;
	if(dev->initialized != 1) return -1;

	if(selection > 3) return -1; /* Options: 0, 1, 2, 3 */

	/* ACCEL_FS_SEL is ACCEL_CONFIG[4:3] */
	/* Clear bits [4:3] */
	dev->accel_config &= ~0b11000;
	/* Set bits [4:3] to selection */
	dev->accel_config |= selection << 3;
	if(MPU6500_Write_I2C(dev, MPU6500_ACCEL_CONFIG, dev->accel_config) != 0)
		return -1;
	
	return 0;
}

int MPU6500_GetAccel(MPU6500_handle_t *dev, MPU6500_output_t *out) {

	uint8_t high, low;
	int16_t x, y, z;

	if(dev == NULL) return -1;
	if(dev->initialized != 1) return -1;
	if(out == NULL) return -1;

	/* accel_fs_sel is accel_config[4:3] */
	uint8_t accel_fs_sel = (dev->accel_config & 0b11000) >> 3;
	const float sensitivity_scale[] = { 16384.0f, 8192.0f, 4096.0f, 2048.0f };
	float accel_sensitivity = sensitivity_scale[accel_fs_sel];
	
	if(dev->comm_ops.read(dev, MPU6500_ACCEL_XOUT_H, &high) != 0) return -1;
	if(dev->comm_ops.read(dev, MPU6500_ACCEL_XOUT_L, &low) != 0) return -1;
	x = ((uint16_t)low) | (((uint16_t)high) << 8);
	
	if(dev->comm_ops.read(dev, MPU6500_ACCEL_YOUT_H, &high) != 0) return -1;
	if(dev->comm_ops.read(dev, MPU6500_ACCEL_YOUT_L, &low) != 0) return -1;
	y = ((uint16_t)low) | (((uint16_t)high) << 8);
	
	if(dev->comm_ops.read(dev, MPU6500_ACCEL_ZOUT_H, &high) != 0) return -1;
	if(dev->comm_ops.read(dev, MPU6500_ACCEL_ZOUT_L, &low) != 0) return -1;
	z = ((uint16_t)low) | (((uint16_t)high) << 8);

	out->accel_xout = ((float) x) / accel_sensitivity;
	out->accel_yout = ((float) y) / accel_sensitivity;
	out->accel_zout = ((float) z) / accel_sensitivity;

	return 0;
}

/* Gyroscope Functions -------------------------------------------------------*/

int MPU6500_SetGyroScale(MPU6500_handle_t *dev, uint8_t selection) {

	if(dev == NULL) return -1;
	if(dev->initialized != 1) return 01;
	
	if(selection > 3) return -1; /* Options: 0, 1, 2, 3 */
	
    /* GYRO_FS_SEL is GYRO_CONFIG[4:3] */
	/* Clear bits [4:3] */
	dev->gyro_config &= ~0b11000;
	/* Set bits [4:3] to selection */
	dev->gyro_config |= selection << 3;
	if(MPU6500_Write_I2C(dev, MPU6500_GYRO_CONFIG, dev->gyro_config) != 0)
		return -1;
	
	return 0;
}

int MPU6500_GetGyro(MPU6500_handle_t *dev, MPU6500_output_t *out) {

	uint8_t high, low;
	int16_t x, y, z;

	if(dev == NULL) return -1;
	if(dev->initialized != 1) return -1;
	if(out == NULL) return -1;

	/* gyro_fs_sel is gyro_config[4:3] */
	uint8_t gyro_fs_sel = (dev->gyro_config & 0b11000) >> 3;
	const float sensitivity_scale[] = { 131.0f, 65.5f, 32.8f, 16.4f };
	float gyro_sensitivity = sensitivity_scale[gyro_fs_sel];
	
	if(dev->comm_ops.read(dev, MPU6500_GYRO_XOUT_H, &high) != 0) return -1;
	if(dev->comm_ops.read(dev, MPU6500_GYRO_XOUT_L, &low) != 0) return -1;
	x = ((uint16_t)low) | (((uint16_t)high) << 8);
	
	if(dev->comm_ops.read(dev, MPU6500_GYRO_YOUT_H, &high) != 0) return -1;
	if(dev->comm_ops.read(dev, MPU6500_GYRO_YOUT_L, &low) != 0) return -1;
	y = ((uint16_t)low) | (((uint16_t)high) << 8);
	
	if(dev->comm_ops.read(dev, MPU6500_GYRO_ZOUT_H, &high) != 0) return -1;
	if(dev->comm_ops.read(dev, MPU6500_GYRO_ZOUT_L, &low) != 0) return -1;
	z = ((uint16_t)low) | (((uint16_t)high) << 8);

	out->gyro_xout = ((float) x) / gyro_sensitivity;
	out->gyro_yout = ((float) y) / gyro_sensitivity;
	out->gyro_zout = ((float) z) / gyro_sensitivity;
	
	return 0;
}

/* Temperature Functions -----------------------------------------------------*/

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
int MPU6500_GetTemp(MPU6500_handle_t *dev, MPU6500_output_t *out) {

	uint8_t high, low;

	if(dev == NULL) return -1;
	if(dev->initialized != 1) return -1;
	if(out == NULL) return -1;

	/* return -1 if temperature readings are disabled in power management reg */
	if(dev->pwr_mgmt_1 & MPU6500_TEMP_DIS) return -1;
	
	if(dev->comm_ops.read(dev, MPU6500_TEMP_OUT_H, &high) != 0) return -1;
	if(dev->comm_ops.read(dev, MPU6500_TEMP_OUT_L, &low) != 0) return -1;

	uint16_t temp_raw = ((uint16_t)low) | (((uint16_t)high) << 8);

	float temp_degC = ((temp_raw - ROOM_TEMP_OFFSET) / TEMP_SENSITIVITY) + 21;

	out->temp_out = temp_degC;
	
	return 0;
}

/* Private Functions ---------------------------------------------------------*/

/* Shared logic for MPU6500_Init_I2C and MPU6500_Init_SPI */
static int MPU6500_Init(MPU6500_handle_t *dev) {
	
    /* Ensure this is an MPU-6500 */
	uint8_t id = 0;
	if(dev->comm_ops.read(dev, MPU6500_WHO_AM_I, &id) != 0) return -1;
	if(id != 0x70) {
		/* This is not an MPU-6500! */
		return -1;
	}
	
	/* Device reset */
	if(MPU6500_Write_I2C(dev, MPU6500_PWR_MGMT_1, MPU6500_DEVICE_RESET) != 0) {
		return -1;
	}

	/* Give the device some time... */
	HAL_Delay(100);
	
	/* Set DLPF_CFG to 4. This sets internal sample rate to 1kHz, with a 9.9ms
	   delay. This is a medium digital low pass filter. */
	if(MPU6500_Write_I2C(dev, MPU6500_CONFIG, 4) != 0) return -1;
	HAL_Delay(10);
	
	/* driver state initialization */
	dev->initialized = 1;
	dev->gyro_config = 0; /* defaults to 0 */
	dev->accel_config = 0; /* defaults to 0 */
	dev->sample_rate_div = 0; /* defaults to 0 */
	dev->dlpf_cfg = 4; /* set to 4 above */
	dev->pwr_mgmt_1 = 1; /* 0x01 on reset */

	return 0;
}

/* Wrapper for HAL_I2C_Mem_Read */
static int MPU6500_Read_I2C(MPU6500_handle_t *dev, uint8_t reg, uint8_t *data) {
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Read(dev->hi2c, dev->addr, reg, 1, data, 1,
							  MPU6500_TIMEOUT);
	if(status != HAL_OK) return -1;
	return 0;
}

/* Wrapper for HAL_I2C_Master_Transmit */
static int MPU6500_Write_I2C(MPU6500_handle_t *dev, uint8_t reg, uint8_t data) {
	HAL_StatusTypeDef status;
	uint8_t msg[2] = {reg, data};
	status = HAL_I2C_Master_Transmit(dev->hi2c, dev->addr, msg, 2, 1000);
	if(status != HAL_OK) return -1;
	return 0;
}

static int MPU6500_Read_SPI(MPU6500_handle_t *dev, uint8_t reg, uint8_t *data) {
	/// not yet implemented
	return -1;
}
static int MPU6500_Write_SPI(MPU6500_handle_t *dev, uint8_t reg, uint8_t data) {
	/// not yet implemented
	return -1;
}
