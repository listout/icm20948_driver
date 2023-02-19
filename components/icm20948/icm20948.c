#include <stdio.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "esp_system.h"
#include "driver/i2c.h"

#include "icm20948.h"

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)                                                                                           \
	(byte & 0x80 ? '1' : '0'), (byte & 0x40 ? '1' : '0'), (byte & 0x20 ? '1' : '0'), (byte & 0x10 ? '1' : '0'),        \
	    (byte & 0x08 ? '1' : '0'), (byte & 0x04 ? '1' : '0'), (byte & 0x02 ? '1' : '0'), (byte & 0x01 ? '1' : '0')

#define ALPHA      0.99f        /*!< Weight of gyroscope */
#define RAD_TO_DEG 57.27272727f /*!< Radians to degrees */

/* MPU6050 register */
#define ICM20948_GYRO_CONFIG_1 0x01
#define ICM20948_ACCEL_CONFIG  0x14
#define ICM20948_INT_PIN_CFG   0x0F
#define ICM20948_INT_ENABLE    0x10
#define ICM20948_INT_ENABLE_1  0x11
#define ICM20948_INT_ENABLE_2  0x12
#define ICM20948_INT_ENABLE_3  0x13
#define ICM20948_INT_STATUS    0x19
#define ICM20948_INT_STATUS_1  0x1A
#define ICM20948_INT_STATUS_2  0x1B
#define ICM20948_INT_STATUS_3  0x1C
#define ICM20948_ACCEL_XOUT_H  0x2D
#define ICM20948_GYRO_XOUT_H   0x33
#define ICM20948_TEMP_XOUT_H   0x39
#define ICM20948_PWR_MGMT_1    0x06
#define ICM20948_WHO_AM_I      0x00
#define ICM20948_REG_BANK_SEL  0x7F

typedef struct {
	i2c_port_t bus;
	gpio_num_t int_pin;
	uint16_t dev_addr;
	uint32_t counter;
	float dt; /*!< delay time between two measurements, dt should be small (ms level) */
	struct timeval *timer;
} icm20948_dev_t;

static esp_err_t
icm20948_write(icm20948_handle_t sensor,
               const uint8_t reg_start_addr,
               const uint8_t *const data_buf,
               const uint8_t data_len)
{
	icm20948_dev_t *sens = (icm20948_dev_t *)sensor;
	esp_err_t ret;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	ret = i2c_master_start(cmd);
	assert(ESP_OK == ret);
	ret = i2c_master_write_byte(cmd, sens->dev_addr | I2C_MASTER_WRITE, true);
	assert(ESP_OK == ret);
	ret = i2c_master_write_byte(cmd, reg_start_addr, true);
	assert(ESP_OK == ret);
	ret = i2c_master_write(cmd, data_buf, data_len, true);
	assert(ESP_OK == ret);
	ret = i2c_master_stop(cmd);
	assert(ESP_OK == ret);
	ret = i2c_master_cmd_begin(sens->bus, cmd, 1000 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	return ret;
}

static esp_err_t
icm20948_read(icm20948_handle_t sensor, const uint8_t reg_start_addr, uint8_t *const data_buf, const uint8_t data_len)
{
	icm20948_dev_t *sens = (icm20948_dev_t *)sensor;
	esp_err_t ret;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	ret = i2c_master_start(cmd);
	assert(ESP_OK == ret);
	ret = i2c_master_write_byte(cmd, sens->dev_addr | I2C_MASTER_WRITE, true);
	assert(ESP_OK == ret);
	ret = i2c_master_write_byte(cmd, reg_start_addr, true);
	assert(ESP_OK == ret);
	ret = i2c_master_start(cmd);
	assert(ESP_OK == ret);
	ret = i2c_master_write_byte(cmd, sens->dev_addr | I2C_MASTER_READ, true);
	assert(ESP_OK == ret);
	ret = i2c_master_read(cmd, data_buf, data_len, I2C_MASTER_LAST_NACK);
	assert(ESP_OK == ret);
	ret = i2c_master_stop(cmd);
	assert(ESP_OK == ret);
	ret = i2c_master_cmd_begin(sens->bus, cmd, 1000 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	return ret;
}

icm20948_handle_t
icm20948_create(i2c_port_t port, const uint16_t dev_addr)
{
	icm20948_dev_t *sensor = (icm20948_dev_t *)calloc(1, sizeof(icm20948_dev_t));
	sensor->bus = port;
	sensor->dev_addr = dev_addr << 1;
	sensor->counter = 0;
	sensor->dt = 0;
	sensor->timer = (struct timeval *)calloc(1, sizeof(struct timeval));
	return (icm20948_handle_t)sensor;
}

void
icm20948_delete(icm20948_handle_t sensor)
{
	icm20948_dev_t *sens = (icm20948_dev_t *)sensor;
	free(sens);
}

esp_err_t
icm20948_get_deviceid(icm20948_handle_t sensor, uint8_t *const deviceid)
{
	return icm20948_read(sensor, ICM20948_WHO_AM_I, deviceid, 1);
}

esp_err_t
icm20948_wake_up(icm20948_handle_t sensor)
{
	esp_err_t ret;
	uint8_t tmp;
	ret = icm20948_read(sensor, ICM20948_PWR_MGMT_1, &tmp, 1);
	if (ESP_OK != ret) {
		return ret;
	}
	tmp &= (~BIT6);
	ret = icm20948_write(sensor, ICM20948_PWR_MGMT_1, &tmp, 1);
	return ret;
}

esp_err_t
icm20948_sleep(icm20948_handle_t sensor)
{
	esp_err_t ret;
	uint8_t tmp;
	ret = icm20948_read(sensor, ICM20948_PWR_MGMT_1, &tmp, 1);
	if (ESP_OK != ret) {
		return ret;
	}
	tmp |= BIT6;
	ret = icm20948_write(sensor, ICM20948_PWR_MGMT_1, &tmp, 1);
	return ret;
}

esp_err_t
icm20948_reset(icm20948_handle_t sensor)
{
	esp_err_t ret;
	uint8_t tmp;

	ret = icm20948_read(sensor, ICM20948_PWR_MGMT_1, &tmp, 1);
	if (ret != ESP_OK)
		return ret;
	tmp |= 0x80;
	ret = icm20948_write(sensor, ICM20948_PWR_MGMT_1, &tmp, 1);
	if (ret != ESP_OK)
		return ret;

	return ret;
}

esp_err_t
icm20948_set_bank(icm20948_handle_t sensor, uint8_t bank)
{
	esp_err_t ret;
	if (bank > 3)
		return ESP_FAIL;
	bank = (bank << 4) & 0x30;
	ret = icm20948_write(sensor, ICM20948_REG_BANK_SEL, &bank, 1);
	return ret;
}

esp_err_t
icm20948_set_gyro_fs(icm20948_handle_t sensor, icm20948_gyro_fs_t gyro_fs)
{
	esp_err_t ret;
	uint8_t tmp;

	ret = icm20948_set_bank(sensor, 2);
	if (ret != ESP_OK)
		return ret;

	ret = icm20948_read(sensor, ICM20948_GYRO_CONFIG_1, &tmp, 1);

#if CONFIG_LOG_DEFAULT_LEVEL == 4
	printf(BYTE_TO_BINARY_PATTERN "\n", BYTE_TO_BINARY(tmp));
#endif

	if (ret != ESP_OK)
		return ret;
	tmp &= 0x09;
	tmp |= (gyro_fs << 1);

#if CONFIG_LOG_DEFAULT_LEVEL == 4
	printf(BYTE_TO_BINARY_PATTERN "\n", BYTE_TO_BINARY(tmp));
#endif

	ret = icm20948_write(sensor, ICM20948_GYRO_CONFIG_1, &tmp, 1);
	return ret;
}

esp_err_t
icm20948_get_gyro_fs(icm20948_handle_t sensor, icm20948_gyro_fs_t *gyro_fs)
{
	esp_err_t ret;
	uint8_t tmp;

	ret = icm20948_set_bank(sensor, 2);
	if (ret != ESP_OK)
		return ret;

	ret = icm20948_read(sensor, ICM20948_GYRO_CONFIG_1, &tmp, 1);

#if CONFIG_LOG_DEFAULT_LEVEL == 4
	printf(BYTE_TO_BINARY_PATTERN "\n", BYTE_TO_BINARY(tmp));
#endif

	tmp &= 0x06;
	tmp >>= 1;
	*gyro_fs = tmp;
	return ret;
}

esp_err_t
icm20948_set_acce_fs(icm20948_handle_t sensor, icm20948_acce_fs_t acce_fs)
{
	esp_err_t ret;
	uint8_t tmp;

	ret = icm20948_set_bank(sensor, 2);
	if (ret != ESP_OK)
		return ret;

	ret = icm20948_read(sensor, ICM20948_ACCEL_CONFIG, &tmp, 1);

#if CONFIG_LOG_DEFAULT_LEVEL == 4
	printf(BYTE_TO_BINARY_PATTERN "\n", BYTE_TO_BINARY(tmp));
#endif

	if (ret != ESP_OK)
		return ret;
	tmp &= 0x09;
	tmp |= (acce_fs << 1);

#if CONFIG_LOG_DEFAULT_LEVEL == 4
	printf(BYTE_TO_BINARY_PATTERN "\n", BYTE_TO_BINARY(tmp));
#endif

	ret = icm20948_write(sensor, ICM20948_ACCEL_CONFIG, &tmp, 1);
	return ret;
}

esp_err_t
icm20948_get_acce_fs(icm20948_handle_t sensor, icm20948_acce_fs_t *acce_fs)
{
	esp_err_t ret;
	uint8_t tmp;

	ret = icm20948_set_bank(sensor, 2);
	if (ret != ESP_OK)
		return ret;

	ret = icm20948_read(sensor, ICM20948_ACCEL_CONFIG, &tmp, 1);

#if CONFIG_LOG_DEFAULT_LEVEL == 4
	printf(BYTE_TO_BINARY_PATTERN "\n", BYTE_TO_BINARY(tmp));
#endif

	tmp &= 0x06;
	tmp >>= 1;
	*acce_fs = tmp;
	return ret;
}

esp_err_t
icm20948_get_raw_acce(icm20948_handle_t sensor, icm20948_raw_acce_value_t *const raw_acce_value)
{
	uint8_t data_rd[6];
	esp_err_t ret = icm20948_read(sensor, ICM20948_ACCEL_XOUT_H, data_rd, sizeof(data_rd));

	raw_acce_value->raw_acce_x = (int16_t)((data_rd[0] << 8) + (data_rd[1]));
	raw_acce_value->raw_acce_y = (int16_t)((data_rd[2] << 8) + (data_rd[3]));
	raw_acce_value->raw_acce_z = (int16_t)((data_rd[4] << 8) + (data_rd[5]));
	return ret;
}

esp_err_t
icm20948_get_acce_sensitivity(icm20948_handle_t sensor, float *const acce_sensitivity)
{
	esp_err_t ret;
	icm20948_acce_fs_t acce_fs;
	ret = icm20948_get_acce_fs(sensor, &acce_fs);
	switch (acce_fs) {
	case ACCE_FS_2G:
		*acce_sensitivity = 16384;
		break;

	case ACCE_FS_4G:
		*acce_sensitivity = 8192;
		break;

	case ACCE_FS_8G:
		*acce_sensitivity = 4096;
		break;

	case ACCE_FS_16G:
		*acce_sensitivity = 2048;
		break;

	default:
		break;
	}
	return ret;
}

esp_err_t
icm20948_get_acce(icm20948_handle_t sensor, icm20948_acce_value_t *const acce_value)
{
	esp_err_t ret;
	float acce_sensitivity;
	icm20948_raw_acce_value_t raw_acce;

	ret = icm20948_get_acce_sensitivity(sensor, &acce_sensitivity);
	if (ret != ESP_OK) {
		return ret;
	}

	ret = icm20948_set_bank(sensor, 0);
	if (ret != ESP_OK)
		return ret;

	ret = icm20948_get_raw_acce(sensor, &raw_acce);
	if (ret != ESP_OK) {
		return ret;
	}

	acce_value->acce_x = raw_acce.raw_acce_x / acce_sensitivity;
	acce_value->acce_y = raw_acce.raw_acce_y / acce_sensitivity;
	acce_value->acce_z = raw_acce.raw_acce_z / acce_sensitivity;
	return ESP_OK;
}
