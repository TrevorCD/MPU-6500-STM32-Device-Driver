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
#define MPU6500_AD0_TIED_LOW       (1)    /* Tie low for lower slave address */
#define MPU6500_READ_TIMEOUT       (1000) /* timeout for i2c read in ms */

/* MPU-6500 Registers --------------------------------------------------------*/
#define MPU6500_SELF_TEST_X_GYRO   (0x00)
#define MPU6500_SELF_TEST_Y_GYRO   (0x01)
#define MPU6500_SELF_TEST_Z_GYRO   (0x02)
#define MPU6500_SELF_TEST_X_ACCEL  (0x0D)
#define MPU6500_SELF_TEST_Y_ACCEL  (0x0E)
#define MPU6500_SELF_TEST_Z_ACCEL  (0x0F)
#define MPU6500_XG_OFFSET_H        (0x13) /* X_OFFS_USR        [15:8] */
#define MPU6500_XG_OFFSET_L        (0x14) /* X_OFFS_USR        [7:0]  */
#define MPU6500_YG_OFFSET_H        (0x15) /* Y_OFFS_USR        [15:8] */
#define MPU6500_YG_OFFSET_L        (0x16) /* Y_OFFS_USR        [7:0]  */
#define MPU6500_ZG_OFFSET_H        (0x17) /* Z_OFFS_USR        [15:8] */
#define MPU6500_ZG_OFFSET_L        (0x18) /* Z_OFFS_USR        [7:0]  */
#define MPU6500_SMPLRT_DIV         (0x19)
#define MPU6500_CONFIG             (0x1A) /* FIFO_MODE         [6]
											 EXT_SYNC_SET      [4:3]
											 DLPF_CFG          [2:0]  */
#define MPU6500_GYRO_CONFIG        (0x1B) /* XG_ST             [7]
											 YG_ST             [6]
											 ZG_ST             [5]
											 GYRO_FS_SEL       [4:3]
											 FCHOICE_B         [1:0]  */
#define MPU6500_ACCEL_CONFIG       (0x1C) /* XA_ST             [7]
											 YA_ST             [6]
											 ZA_ST             [5]
											 ACCEL_FS_SEL      [4:3]  */
#define MPU6500_ACCEL_CONFIG2      (0x1D) /* ACCEL_FC_HOICE_B  [3]
											 A_DLPF_CFG        [2:0]  */
#define MPU6500_LP_ACCEL_ODR       (0x1E) /* LPOSC_CLKSEL      [3:0]  */
#define MPU6500_WOM_THR            (0x1F)
#define MPU6500_FIFO_EN            (0x23) /* TEMP_FIFO_EN      [7]
											 GYRO_XOUT         [6]
											 GYRO_YOUT         [5]
											 GYRO_ZOUT         [4]
											 ACCEL             [3]
											 SLV2              [2]
											 SLV1              [1]
											 SLV0              [0]    */
#define MPU6500_I2C_MST_CTRL       (0x24) /* MULT_MST_EN       [7]
											 WAIT_FOR_ES       [6]
											 SLV3_FIFO_EN      [5]
											 I2C_MST_P_NSR     [4]
											 I2C_MST_CLK       [3:0]  */
#define MPU6500_I2C_SLV0_ADDR      (0x25)
#define MPU6500_I2C_SLV0_REG       (0x26)
#define MPU6500_I2C_SLV0_CTRL      (0x27)
#define MPU6500_I2C_SLV1_ADDR      (0x28)
#define MPU6500_I2C_SLV1_REG       (0x29)
#define MPU6500_I2C_SLV1_CTRL      (0x2A)
/* ... more I2C addrs ...*/
#define MPU6500_I2C_MST_STATUS     (0x36)
#define MPU6500_INT_PIN_CFG        (0x27) /* ACTL              [7]
											 OPEN              [6]
											 LATCH_INT_EN      [5]
											 INT_ANYRD_2CLEAR  [4]
											 ACTL_FSY_EN       [3]
											 FSYNC_INT_MODE_EN [2]
											 BYPASS_EN         [1]    */
#define MPU6500_INT_ENABLE         (0x38) /* WOM_EN            [6]
											 FIFO_OFLOW_EN     [4]
											 FSYNC_INT_EN      [3]
											 RAW_RDY_EN        [0]    */
#define MPU6500_INT_STATUS         (0x3A) /* WOM_INT           [6]
											 FIFO_OFLOW_INT    [4]
											 FSYNC_INT         [3]
											 DMP_INT           [1]
											 RAW_DATA_RDY_INT  [0]    */
#define MPU6500_ACCEL_XOUT_H       (0x3B)
#define MPU6500_ACCEL_XOUT_L       (0x3C)
#define MPU6500_ACCEL_YOUT_H       (0x3D)
#define MPU6500_ACCEL_YOUT_L       (0x3E)
#define MPU6500_ACCEL_ZOUT_H       (0x3F)
#define MPU6500_ACCEL_ZOUT_L       (0x40)
#define MPU6500_TEMP_OUT_H         (0x41)
#define MPU6500_TEMP_OUT_L         (0x42)
#define MPU6500_GYRO_XOUT_H        (0x43)
#define MPU6500_GYRO_XOUT_L        (0x44)
#define MPU6500_GYRO_YOUT_H        (0x45)
#define MPU6500_GYRO_YOUT_L        (0x46)
#define MPU6500_GYRO_ZOUT_H        (0x47)
#define MPU6500_GYRO_ZOUT_L        (0x48)
#define MPU6500_EXT_SENS_DATA_00   (0x49)
#define MPU6500_EXT_SENS_DATA_01   (0x4A)
#define MPU6500_EXT_SENS_DATA_02   (0x4B)
#define MPU6500_EXT_SENS_DATA_03   (0x4C)
#define MPU6500_EXT_SENS_DATA_04   (0x4D)
#define MPU6500_EXT_SENS_DATA_05   (0x4E)
#define MPU6500_EXT_SENS_DATA_06   (0x4F)
#define MPU6500_EXT_SENS_DATA_07   (0x50)
#define MPU6500_EXT_SENS_DATA_08   (0x51)
#define MPU6500_EXT_SENS_DATA_09   (0x52)
#define MPU6500_EXT_SENS_DATA_10   (0x53)
#define MPU6500_EXT_SENS_DATA_11   (0x54)
#define MPU6500_EXT_SENS_DATA_12   (0x55)
#define MPU6500_EXT_SENS_DATA_13   (0x56)
#define MPU6500_EXT_SENS_DATA_14   (0x57)
#define MPU6500_EXT_SENS_DATA_15   (0x58)
#define MPU6500_EXT_SENS_DATA_16   (0x59)
#define MPU6500_EXT_SENS_DATA_17   (0x5A)
#define MPU6500_EXT_SENS_DATA_18   (0x5B)
#define MPU6500_EXT_SENS_DATA_19   (0x5C)
#define MPU6500_EXT_SENS_DATA_20   (0x5D)
#define MPU6500_EXT_SENS_DATA_21   (0x5E)
#define MPU6500_EXT_SENS_DATA_22   (0x5F)
#define MPU6500_EXT_SENS_DATA_23   (0x60)
#define MPU6500_I2C_SLV0_DO        (0x63)
#define MPU6500_I2C_SLV1_DO        (0x64)
#define MPU6500_I2C_SLV2_DO        (0x65)
#define MPU6500_I2C_SLV3_DO        (0x66)
#define MPU6500_I2C_MST_DELAY_CTRL (0x67) /* DELAY_ES_SHADOW   [7]
											 I2C_SLV4_DLY_EN   [4]
											 I2C_SLV3_DLY_EN   [3]
											 I2C_SLV2_DLY_EN   [2]
											 I2C_SLV1_DLY_EN   [1]
											 I2C_SLV0_DLY_EN   [0]    */
#define MPU6500_SIGNAL_PATH_REST   (0x68) /* GYRO_RST          [2]
											 ACCEL_RST         [1]
											 TEMP_RST          [0]    */
#define MPU6500_ACCEL_INTEL_CTRL   (0x69) /* ACCEL_INT_EL_EN   [7]
											 ACCEL_INT_EL_MODE [6]    */
#define MPU6500_USER_CTRL          (0x6A) /* DMP_EN            [7]
											 FIFO_EN           [6]
											 I2C_MST_EN        [5]
											 I2C_IF_DIS        [4]
											 DMP_RST           [3]
											 FIFO_RST          [2]
											 I2C_MST_RST       [1]
											 SIG_COND_RST      [0]    */
#define MPU6500_PWR_MGMT_1         (0x6B) /* DEVICE_RESET      [7]
											 SLEEP             [6]
											 CYCLE             [5]
											 GYRO_STANDBY      [4]
											 TEMP_DIS          [3]
											 CLKSEL            [2:0]  */
#define MPU6500_PWR_MGMT_2         (0x6C) /* DIS_XA            [5]
											 DIS_YA            [4]
											 DIS_ZA            [3]
											 DIS_XG            [2]
											 DIS_YG            [1]
											 DIS_ZG            [0]    */
#define MPU6500_FIFO_COUNT_H       (0x72) /* FIFO_CNT[12:8]    [4:0]  */
#define MPU6500_FIFO_COUNT_L       (0x73) /* FIFO_CNT[7:0]     [7:0]  */
#define MPU6500_FIFO_R_W           (0x74)
#define MPU6500_WHO_AM_I           (0x75)
#define MPU6500_XA_OFFSET_H        (0x77)
#define MPU6500_XA_OFFSET_L        (0x78)
#define MPU6500_YA_OFFSET_H        (0x7A)
#define MPU6500_YA_OFFSET_L        (0x7B)
#define MPU6500_ZA_OFFSET_H        (0x7D)
#define MPU6500_ZA_OFFSET_L        (0x7E)

#if MPU6500_AD0_TIED_LOW
#define MPU6500_SLAVE_ADDR         (0b1101000)
#else
#define MPU6500_SLAVE_ADDR         (0b1101001)
#endif


/* Public Prototypes ---------------------------------------------------------*/
int MPU6500_Init(MPU6500_HandleTypeDef *dev);
int MPU6500_GetAccel(MPU6500_HandleTypeDef *dev, MPU6500_OutputTypeDef *out);
int MPU6500_GetGyro(MPU6500_HandleTypeDef *dev, MPU6500_OutputTypeDef *out);
int MPU6500_GetTemp(MPU6500_HandleTypeDef *dev, MPU6500_OutputTypeDef *out);
int MPU6500_DataReady(MPU6500_HandleTypeDef *dev);

/* Private Prototypes --------------------------------------------------------*/
static int MPU6500_Read(MPU6500_HandleTypeDef *dev, uint8_t *data);
static int MPU6500_Write(MPU6500_HandleTypeDef *dev, uint8_t data);

/* Public Functions ----------------------------------------------------------*/

int MPU6500_Init(MPU6500_HandleTypeDef *dev) {

	HAL_StatusTypeDef status;

	if(dev == NULL) return -1;
	if(dev->initialized != 0) return -1;
	
	status = HAL_I2C_IsDeviceReady(dev->hi2c, MPU6500_SLAVE_ADDR, 3, 50);
	if(status != HAL_OK) return -1;

	/* Enable raw data ready interrupts (MPU6500_INT_ENABLE[bit 0] == 1) */
	if(MPU6500_Write(dev, MPU6500_INT_ENABLE, 1) != 0) return -1;

	///
	
	dev->initialized = 1;
	
	return 0;
}

/*----------------------------------------------------------------------------*/

int MPU6500_GetAccel(MPU6500_HandleTypeDef *dev, MPU6500_OutputTypeDef *out) {

	uint8_t high, low;
	uint16_t x, y, z;

	if(dev == NULL) return -1;
	if(dev->initialized != 1) return -1;
	if(out == NULL) return -1;
	
	if(MPU6500_Read(dev, MPU6500_ACCEL_XOUT_H, &high) != 0) return -1;
	if(MPU6500_Read(dev, MPU6500_ACCEL_XOUT_L, &low) != 0) return -1;
	x = ((uint16_t)low) | (((uint16_t)high) << 8);
	
	if(MPU6500_Read(dev, MPU6500_ACCEL_YOUT_H, &high) != 0) return -1;
	if(MPU6500_Read(dev, MPU6500_ACCEL_YOUT_L, &low) != 0) return -1;
	y = ((uint16_t)low) | (((uint16_t)high) << 8);
	
	if(MPU6500_Read(dev, MPU6500_ACCEL_ZOUT_H, &high) != 0) return -1;
	if(MPU6500_Read(dev, MPU6500_ACCEL_ZOUT_L, &low) != 0) return -1;
	z = ((uint16_t)low) | (((uint16_t)high) << 8);

	out->accel_xout = x;
	out->accel_yout = y;
	out->accel_zout = z;
	
	return 0;
}

/*----------------------------------------------------------------------------*/

int MPU6500_GetGyro(MPU6500_HandleTypeDef *dev, MPU6500_OutputTypeDef *out) {

	uint8_t high, low;
	uint16_t x, y, z;

	if(dev == NULL) return -1;
	if(dev->initialized != 1) return -1;
	if(out == NULL) return -1;
	
	if(MPU6500_Read(dev, MPU6500_GYRO_XOUT_H, &high) != 0) return -1;
	if(MPU6500_Read(dev, MPU6500_GYRO_XOUT_L, &low) != 0) return -1;
	x = ((uint16_t)low) | (((uint16_t)high) << 8);
	
	if(MPU6500_Read(dev, MPU6500_GYRO_YOUT_H, &high) != 0) return -1;
	if(MPU6500_Read(dev, MPU6500_GYRO_YOUT_L, &low) != 0) return -1;
	y = ((uint16_t)low) | (((uint16_t)high) << 8);
	
	if(MPU6500_Read(dev, MPU6500_GYRO_ZOUT_H, &high) != 0) return -1;
	if(MPU6500_Read(dev, MPU6500_GYRO_ZOUT_L, &low) != 0) return -1;
	z = ((uint16_t)low) | (((uint16_t)high) << 8);

	out->gyro_xout = x;
	out->gyro_yout = y;
	out->gyro_zout = z;
	
	return 0;
}

/*----------------------------------------------------------------------------*/

int MPU6500_GetTemp(MPU6500_HandleTypeDef *dev, MPU6500_OutputTypeDef *out) {

	uint8_t high, low;

	if(dev == NULL) return -1;
	if(dev->initialized != 1) return -1;
	if(out == NULL) return -1;

	if(MPU6500_Read(dev, MPU6500_TEMP_OUT_H, &high) != 0) return -1;
	if(MPU6500_Read(dev, MPU6500_TEMP_OUT_L, &low) != 0) return -1;

	out->temp_out = ((uint16_t)low) | (((uint16_t)high) << 8);
	
	return 0;
}

/*----------------------------------------------------------------------------*/

/* returns 1 on data ready, 0 on not ready, -1 on failure */
int MPU6500_DataReady(MPU6500_HandleTypeDef *dev) {
	uint8_t data;
	if(dev == NULL) return -1;
	if(MPU6500_Read(dev, MPU6500_INT_STATUS, &data) != 0) return -1;
	if(data & 1) return 1;
	return 0;
}

/* Private Functions ---------------------------------------------------------*/

static int MPU6500_Read(MPU6500_HandleTypeDef *dev, uint8_t reg, uint8_t *data){
	HAL_StatusTypeDef status;
	if(dev == NULL) return -1;
	if(data == NULL) return -1;
	status = HAL_I2C_Mem_Read(dev->hi2c, MPU6500_SLAVE_ADDR, reg, 1, data, 1,
							  MPU6500_READ_TIMEOUT);
	if(status != HAL_OK) return -1;
	return 0;
}

/*----------------------------------------------------------------------------*/

static int MPU6500_Write(MPU6500_HandleTypeDef *dev, uint8_t reg, uint8_t data){
	HAL_StatusTypeDef status;
	uint8_t msg[2] = {reg, data};
	if(dev == NULL) return -1;
	status = HAL_I2C_Master_Transmit(dev->hi2c, MPU6500_SLAVE_ADDR,
									 msg, 2, 1000);
	if(status != HAL_OK) return -1;
	return 0;
}


