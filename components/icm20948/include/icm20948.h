#ifndef __ICM20948_H__
#define __ICM20948_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "i2c_bus.h"
#define ICM_20948_I2C_ADDR_AD0 0x69 // Or 0x68 when AD0 is low
#define ICM_20948_WHOAMI 0xEA

typedef void *icm20948_handle_t;

icm20948_handle_t icm20948_create(i2c_bus_handle_t bus, uint8_t dev_addr);
esp_err_t icm20948_delete(icm20948_handle_t *sensor);
esp_err_t icm20948_get_deviceid(icm20948_handle_t sensor, uint8_t *deviceid);


#ifdef __cplusplus
}
#endif

#endif // __ICM20948_H__
