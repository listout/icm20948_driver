# ESP-IDF Driver for ICM20948

Much of the code is take from MPU6050 component from the esp-bsp library
found [here](https://github.com/espressif/esp-bsp/tree/master/components/mpu6050).

In development, for now can only read accelerometer data.

```
I (326) icm test: Starting ICM test
I (336) icm test: I2C bus initialization: ESP_OK
I (336) icm test: ICM20948 creation successfull!
I (346) icm test: ESP_OK
I (356) icm test: ESP_OK
I (356) icm test: ESP_OK
I (356) icm test: ESP_OK
I (356) icm test: 0xEA
I (356) icm test: ESP_OK
I (366) icm test: ESP_OK
I (366) icm test: ESP_OK
I (366) icm test: 2
I (376) icm test: ESP_OK
I (376) icm test: ESP_OK
I (386) icm test: ESP_OK
I (386) icm test: 1
I (386) icm test: ESP_OK
I (396) icm test: 8192.000000
I (396) icm test: ESP_OK
I (396) icm test: 0.120850 0.167480 0.852295
I (506) icm test: ESP_OK
I (506) icm test: 0.061523 0.385986 0.989990
I (606) icm test: ESP_OK
I (606) icm test: -0.020996 0.198975 1.000732
I (706) icm test: ESP_OK
I (706) icm test: -0.159668 -0.025391 1.037354
I (806) icm test: ESP_OK
I (806) icm test: -0.364258 -0.411865 0.826172
I (906) icm test: ESP_OK
I (906) icm test: -0.441162 -0.831543 0.340820
I (1006) icm test: ESP_OK
I (1006) icm test: -0.334717 -0.898926 0.097900
I (1106) icm test: ESP_OK
I (1106) icm test: -0.131592 -0.965820 0.170166
I (1206) icm test: ESP_OK
I (1206) icm test: 0.149170 -0.903564 0.441162
I (1306) icm test: ESP_OK
I (1306) icm test: 0.275635 -0.783691 0.679443
```
