#include <stdio.h>
#include "icm20948.h"

typedef struct {
	i2c_bus_device_handle_t i2c_dev;
	uint8_t dev_addr;
	uint32_t counter;
	/* delay time between twice measurement, dt should be small (ms level) */
	float dt;
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
	ret = i2c_bus_read_byte(sens->i2c_dev, ICM_20948_WHOAMI, &tmp);
	*deviceid = tmp;
	return ret;
}
