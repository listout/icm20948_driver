#include <stdio.h>

#include "esp_err.h"
#include "esp_log.h"
#include "i2c_bus.h"
#include "icm20948.h"

static const char *TAG = "icm main";

static i2c_bus_handle_t i2c_bus = NULL;
icm20948_handle_t icm20948 = NULL;

#define I2C_MASTER_SCL_IO 22      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 21      /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

void
app_main(void)
{

	i2c_config_t conf = {
	    .mode = I2C_MODE_MASTER,
	    .sda_io_num = I2C_MASTER_SDA_IO,
	    .sda_pullup_en = GPIO_PULLUP_ENABLE,
	    .scl_io_num = I2C_MASTER_SCL_IO,
	    .scl_pullup_en = GPIO_PULLUP_ENABLE,
	    .master.clk_speed = I2C_MASTER_FREQ_HZ,
	};
	i2c_bus = i2c_bus_create(I2C_MASTER_NUM, &conf);
	icm20948 = icm20948_create(i2c_bus, ICM_20948_I2C_ADDR_AD0);

	vTaskDelay(10 / portTICK_PERIOD_MS);

	esp_err_t ret;
	uint8_t icm20948_deviceid;
	ret = icm20948_get_deviceid(icm20948, &icm20948_deviceid);
	if (ret != ESP_OK)
		ESP_LOGE(TAG, "Opps, Something went wrong");
}
