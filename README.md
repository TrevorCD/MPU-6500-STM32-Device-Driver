# MPU-6500-STM32-Device-Driver

An interrupt-driven device driver for the MPU-6500 gyroscope and accelerometer. This depends on the STM32 HAL for I2C and SPI communication.

## Usage

This driver can either use SPI or I2C communication. On driver initialization,
use either the MPU6500_Init_SPI or MPU6500_Init_I2C function to select the
communication protocol.



### Integration

- Add mpu6500.c to C_SOURCES in the stm32 makefile, and add a -I[path to mpu6500.h] as a flag.

- Enable interrupts on whichever line is connected to the MPU6500's INT pin.

- Hook the MPU6500_IntCallback into HAL_GPIO_EXTI_Callback for the aforementioned interrupt line.

### Initialization

- Initialize an MPU6500_HandleTypeDef struct as zeroed memory.

- Initialize an MPU6500_OutputTypeDef struct.

- Initialize either an SPI_HandleTypeDef or I2C_HandleTypeDef.

- Call either MPU6500_Init_SPI(), or MPU6500_Init_I2C(), depending on your setup,  with the device and protocol handles.

- Set the sample rate divider using MPU6500_SetSampleRateDiv(). The rate defaults to 1kHz, and is set at 1kHz/(1+div) where div is the value passed to SetSampleRateDiv.

- Enable interrupts from the MPU6500 with MPU6500_EnableInterrupts(). This is done after setting the sample rate divider to avoid interrupt starvation.

### Runtime usage

- Use MPU6500_GetAccel, GetGyro, and GetTemp when the data_ready field of the MPU6500 handle is 1. This value is set by the interrupt callback. Set data_ready back to 0 before reading.

- View the data read from the device through the MPU6500_OutputTypeDef struct.

- Due to the noise of the gyroscope and accelerometer, it is best practice to calculate an average of the first few samples, and subtract this average from each sample as an offset.
