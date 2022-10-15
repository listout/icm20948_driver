#include <stdio.h>

#include "icm20948.h"

typedef struct {
	i2c_bus_device_handle_t i2c_dev;
	uint8_t dev_addr;
	uint32_t counter;
	float dt; /* delay time between twice measurement, dt should be small (ms
	             level) */
	struct timeval *timer;
} icm20948_dev_t;

icm20948_handle_t
icm20948_create(i2c_bus_handle_t bus, uint8_t dev_addr)
{
	if (bus == NULL) {
		return NULL;
	}
	icm20948_dev_t *sens = (icm20948_dev_t *)calloc(1, sizeof(icm20948_dev_t));
	sens->i2c_dev = i2c_bus_device_create(
	    bus, dev_addr, i2c_bus_get_current_clk_speed(bus));
	if (sens->i2c_dev == NULL) {
		free(sens);
		return NULL;
	}
	sens->dev_addr = dev_addr;
	sens->counter = 0;
	sens->dt = 0;
	sens->timer = (struct timeval *)calloc(1, sizeof(struct timeval));
	return (icm20948_handle_t)sens;
}

esp_err_t
icm20948_delete(icm20948_handle_t *sensor)
{
	if (*sensor == NULL) {
		return ESP_OK;
	}
	icm20948_dev_t *sens = (icm20948_dev_t *)(*sensor);
	i2c_bus_device_delete(&sens->i2c_dev);
	free(sens->timer);
	free(sens);
	*sensor = NULL;
	return ESP_OK;
}

esp_err_t
icm20948_get_deviceid(icm20948_handle_t sensor, uint8_t *deviceid)
{
	icm20948_dev_t *sens = (icm20948_dev_t *)sensor;
	esp_err_t ret;
	uint8_t tmp;
	ret = i2c_bus_read_byte(sens->i2c_dev, ICM20948_WHO_AM_I, &tmp);
	*deviceid = tmp;
	return ret;
}

esp_err_t
icm20948_wake_up(icm20948_handle_t sensor)
{
	icm20948_dev_t *sens = (icm20948_dev_t *)sensor;
	esp_err_t ret;
	uint8_t tmp;
	ret = i2c_bus_read_byte(sens->i2c_dev, ICM20948_PWR_MGMT_1, &tmp);
	if (ret != ESP_OK)
		return ret;
	tmp &= (~BIT6);
	ret = i2c_bus_write_byte(sens->i2c_dev, ICM20948_PWR_MGMT_1, tmp);
	return ret;
}

esp_err_t
icm20948_sleep(icm20948_handle_t sensor)
{
	icm20948_dev_t *sens = (icm20948_dev_t *)sensor;
	esp_err_t ret;
	uint8_t tmp;
	ret = i2c_bus_read_byte(sens->i2c_dev, ICM20948_PWR_MGMT_1, &tmp);
	if (ret != ESP_OK)
		return ret;
	tmp |= (BIT6);
	ret = i2c_bus_write_byte(sens->i2c_dev, ICM20948_PWR_MGMT_1, tmp);
	return ret;
}

esp_err_t
icm20948_reset(icm20948_handle_t sensor)
{
	icm20948_dev_t *sens = (icm20948_dev_t *)sensor;
	esp_err_t ret;
	ret = i2c_bus_write_bit(sens->i2c_dev, ICM20948_PWR_MGMT_1, 7, true);
	return ret;
}

esp_err_t
icm20948_bypass_enable(icm20948_handle_t sensor, bool enable)
{
	icm20948_dev_t *sens = (icm20948_dev_t *)sensor;
	esp_err_t ret;
	ret = i2c_bus_write_bit(sens->i2c_dev, ICM20948_INT_PIN_CFG, 1, enable);
	return ret;
}

esp_err_t
icm20948_switch_user_bank(icm20948_handle_t sensor, uint8_t bank)
{
	icm20948_dev_t *sens = (icm20948_dev_t *)sensor;
	esp_err_t ret;
	if (bank > 3)
		return ESP_FAIL;
	bank = (bank << 4) & 0x30; // use bin 5:4 of BANK_SEL registor
	ret = i2c_bus_write_byte(sens->i2c_dev, ICM20948_REG_BANK_SEL, bank);
	return ret;
}

esp_err_t
icm20948_get_gyro_fs(icm20948_handle_t sensor, icm20948_gyro_fs_t *gyro_fs)
{
	icm20948_dev_t *sens = (icm20948_dev_t *)sensor;
	esp_err_t ret;
	uint8_t tmp;
	ret = i2c_bus_read_byte(sens->i2c_dev, ICM20948_GYRO_CONFIG_1, &tmp);
	tmp = (tmp >> 1) & 0x03;
	*gyro_fs = tmp;
	return ret;
}

esp_err_t
icm20948_set_gyro_fs(icm20948_handle_t sensor, icm20948_gyro_fs_t gyro_fs)
{
	icm20948_dev_t *sens = (icm20948_dev_t *)sensor;
	esp_err_t ret;
	uint8_t tmp;
	ret = i2c_bus_read_byte(sens->i2c_dev, ICM20948_GYRO_CONFIG_1, &tmp);
	if (ret != ESP_OK)
		return ret;
	tmp &= (~BIT1);
	tmp &= (~BIT2);
	tmp |= (gyro_fs << 1);
	ret = i2c_bus_write_byte(sens->i2c_dev, ICM20948_GYRO_CONFIG_1, tmp);
	return ret;
}
