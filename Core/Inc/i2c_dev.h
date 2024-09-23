/*
 * i2c_dev.h
 *
 *  Created on: May 8, 2024
 *      Author: ivanm
 */

#ifndef INC_I2C_DEV_H_
#define INC_I2C_DEV_H_

// custom MPU6050 defines

// addresses
#define MPU6050_ADR 0xD0
#define I2C_TIMEOUT 1000
#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_CONFIG_REG 0x1B
#define LOW_PASS_FILTER_REG 0x1A
#define GYRO_XOUT_H_REG 0x43

// values
#define SAMPLING_RATE_1KHZ 0b00000111
#define GYRO_500DS 0b00001000
#define ACCEL_4G 0b00001000
#define ACCEL_8G 0b00010000

//MPU6050 functions
void MPU6050_init(I2C_HandleTypeDef *hi2c, uint8_t sampling_rate, uint8_t guro_sens, uint8_t accel_sens);
void MPU6050_calibrate(uint16_t sample_amount);
//void MPU6050_getGyroValues(int16_t *angles);
//void MPU6050_getGyroRates(float *angles);
//void MPU6050_getAccelValues(int16_t *acceleration);
//void MPU6050_getAccelAngles(float *angles);
void MPU6050_getAnglesKalman(float *input_angle, uint32_t tik);

// custom BMP180 defines

// addresses
#define BMP180_ADR 0xEE
#define BMP180_CONTROL_REG 0xf4
#define BMP180_MSB_REG 0xf6
#define BMP180_LSB_REG 0xf7
#define BMP180_XLSB_REG 0xf8
#define BMP180_SOFT_RESET_REG 0xe0
#define BMP180_SOFT_RESET 0xb6
#define BMP180_CMD_TEMP 0x2E
#define BMP180_DELAY_TEMP 5


// values
typedef struct BMP180_epp {
	short BMP180_AC1;
	short BMP180_AC2;
	short BMP180_AC3;
	unsigned short BMP180_AC4;
	unsigned short BMP180_AC5;
	unsigned short BMP180_AC6;
	short BMP180_B1;
	short BMP180_B2;
	short BMP180_MB;
	short BMP180_MC;
	short BMP180_MD;
} BMP180_epp;

// functions
void BMP180_initCalibrate(I2C_HandleTypeDef *hi2c);
int32_t BMP180_GetRawTemperature(void);
float BMP180_GetTemperature(void);
int32_t BMP180_GetPressure(void);


#endif /* INC_I2C_DEV_H_ */
