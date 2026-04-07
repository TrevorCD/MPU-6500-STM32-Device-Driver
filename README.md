# MPU-6500-STM32-Device-Driver

An interrupt-driven device driver for the MPU-6500 gyroscope and accelerometer. This depends on the STM32 HAL for I2C communication.

## Usage

- Add mpu6500.c to C_SOURCES in the makefile, and add a -I[path to mpu6500.h] as a flag.
- Enable interrupts on whichever line is connected to the MPU6500's INT pin.
- Hook the MPU6500_IntCallback into HAL_GPIO_EXTI_Callback for the aforementioned interrupt line.
- Initialize an MPU6500_HandleTypeDef struct as 0s, and set the hi2c field to an initialized I2C handle (on which the MPU6500 is connected).
- Initialize an MPU6500_OutputTypeDef struct.
- Set the sample rate divider using MPU6500_SetSampleRateDiv(). The rate defaults to 1kHz, and is set at 1kHz/(1+div) where div is the value passed to SetSampleRateDiv.
- Enable interrupts from the MPU6500 with MPU6500_EnableInterrupts().
- Use MPU6500_GetAccel, GetGyro, and GetTemp when the data_ready field of the MPU6500 handle is 1. This value is set by the interrupt callback. Set data_ready back to 0 before reading.
- View the data read from the device through the MPU6500_OutputTypeDef struct.
- Due to the noise of the gyroscope and accelerometer, it is best practice to calculate an average of the first few samples, and subtract this average from each sample as an offset.
