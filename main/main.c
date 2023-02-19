#include <stdio.h>

#include "esp_log.h"
#include "driver/i2c.h"
#include "icm20948.h"

#define I2C_MASTER_SCL_IO  22        /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO  21        /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM     I2C_NUM_0 /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000    /*!< I2C master clock frequency */

static const char *TAG = "icm test";
static icm20948_handle_t icm20948 = NULL;

/**
 * @brief i2c master initialization
 */
static esp_err_t
i2c_bus_init(void)
{
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
	conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

	esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
	if (ret != ESP_OK)
		return ret;

	return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void
app_main(void)
{
	ESP_LOGI(TAG, "Starting ICM test");
	esp_err_t ret = i2c_bus_init();
	ESP_LOGI(TAG, "I2C bus initialization: %s", esp_err_to_name(ret));

	icm20948 = icm20948_create(I2C_MASTER_NUM, 0x69);
	if (icm20948 == NULL) {
		ESP_LOGE(TAG, "ICM20948 create returned NULL!");
		return;
	}
	ESP_LOGI(TAG, "ICM20948 creation successfull!");

	ret = icm20948_reset(icm20948);
	ESP_LOGI(TAG, "%s", esp_err_to_name(ret));

	vTaskDelay(10 / portTICK_PERIOD_MS);

	ret = icm20948_wake_up(icm20948);
	ESP_LOGI(TAG, "%s", esp_err_to_name(ret));

	ret = icm20948_set_bank(icm20948, 0);
	ESP_LOGI(TAG, "%s", esp_err_to_name(ret));

	uint8_t device_id;
	ret = icm20948_get_deviceid(icm20948, &device_id);
	ESP_LOGI(TAG, "%s", esp_err_to_name(ret));
	ESP_LOGI(TAG, "0x%02X", device_id);

	icm20948_gyro_fs_t fs;
	ret = icm20948_get_gyro_fs(icm20948, &fs);
	ESP_LOGI(TAG, "%s", esp_err_to_name(ret));

	ret = icm20948_set_gyro_fs(icm20948, GYRO_FS_1000DPS);
	ESP_LOGI(TAG, "%s", esp_err_to_name(ret));

	ret = icm20948_get_gyro_fs(icm20948, &fs);
	ESP_LOGI(TAG, "%s", esp_err_to_name(ret));
	ESP_LOGI(TAG, "%d", fs);

	icm20948_acce_fs_t as;
	ret = icm20948_get_acce_fs(icm20948, &as);
	ESP_LOGI(TAG, "%s", esp_err_to_name(ret));

	ret = icm20948_set_acce_fs(icm20948, ACCE_FS_4G);
	ESP_LOGI(TAG, "%s", esp_err_to_name(ret));

	ret = icm20948_get_acce_fs(icm20948, &as);
	ESP_LOGI(TAG, "%s", esp_err_to_name(ret));
	ESP_LOGI(TAG, "%d", as);

	float accel_sens;
	ret = icm20948_get_acce_sensitivity(icm20948, &accel_sens);
	ESP_LOGI(TAG, "%s", esp_err_to_name(ret));
	ESP_LOGI(TAG, "%lf", accel_sens);

	float gyro_sens;
	ret = icm20948_get_gyro_sensitivity(icm20948, &gyro_sens);
	ESP_LOGI(TAG, "%s", esp_err_to_name(ret));
	ESP_LOGI(TAG, "%lf", gyro_sens);

	icm20948_acce_value_t acce;
	icm20948_gyro_value_t gyro;

	for (size_t i = 0; i < 10; ++i) {
		ret = icm20948_get_acce(icm20948, &acce);
		ret = icm20948_get_gyro(icm20948, &gyro);
		ESP_LOGI(TAG, "%s", esp_err_to_name(ret));
		ESP_LOGI(TAG, "%lf %lf %lf", acce.acce_x, acce.acce_y, acce.acce_z);
		ESP_LOGI(TAG, "%s", esp_err_to_name(ret));
		ESP_LOGI(TAG, "%lf %lf %lf", gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}
