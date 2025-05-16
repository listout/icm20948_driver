# This is no longer maintained

I've stopped working with ESP32 and ESP-IDF. Please do no use this
library and use something that's newer and actively maintainer.

# ESP-IDF Driver for ICM20948

Much of the code is take from MPU6050 component from the esp-bsp library
found [here](https://github.com/espressif/esp-bsp/tree/master/components/mpu6050).

In development, for now can only read accelerometer and gyroscope data.

```
I (325) icm test: Starting ICM test
I (335) icm test: I2C bus initialization: ESP_OK
I (335) icm test: ICM20948 creation successfull!
I (345) icm test: ESP_OK
I (355) icm test: ESP_OK
I (355) icm test: ESP_OK
I (355) icm test: ESP_OK
I (355) icm test: 0xEA
I (355) icm test: ESP_OK
I (365) icm test: ESP_OK
I (365) icm test: ESP_OK
I (365) icm test: 2
I (365) icm test: ESP_OK
I (375) icm test: ESP_OK
I (375) icm test: ESP_OK
I (375) icm test: 1
I (385) icm test: ESP_OK
I (385) icm test: 8192.000000
I (385) icm test: ESP_OK
I (395) icm test: 32.799999
I (395) icm test: ESP_OK
I (395) icm test: -0.191895 -0.273438 0.952881
I (405) icm test: ESP_OK
I (405) icm test: -38.628048 8.689025 1.158537
I (515) icm test: ESP_OK
I (515) icm test: -0.200684 -0.342773 0.921875
I (515) icm test: ESP_OK
I (515) icm test: -26.951220 2.469512 -0.853659
I (625) icm test: ESP_OK
I (625) icm test: -0.207031 -0.387207 0.901123
I (625) icm test: ESP_OK
I (625) icm test: -21.493902 3.993902 -0.701219
I (735) icm test: ESP_OK
I (735) icm test: -0.201904 -0.378418 0.895020
I (735) icm test: ESP_OK
I (735) icm test: -19.878050 4.359756 2.195122
I (845) icm test: ESP_OK
I (845) icm test: -0.212891 -0.406494 0.885742
I (845) icm test: ESP_OK
I (845) icm test: -16.371952 2.530488 -0.182927
I (955) icm test: ESP_OK
I (955) icm test: -0.221436 -0.463379 0.875000
I (955) icm test: ESP_OK
I (955) icm test: -22.317074 3.109756 -0.457317
I (1065) icm test: ESP_OK
I (1065) icm test: -0.229492 -0.465576 0.845215
I (1065) icm test: ESP_OK
I (1065) icm test: -13.079268 4.390244 1.097561
I (1175) icm test: ESP_OK
I (1175) icm test: -0.231201 -0.507080 0.837891
I (1175) icm test: ESP_OK
I (1175) icm test: -14.176829 2.073171 0.396341
I (1285) icm test: ESP_OK
I (1285) icm test: -0.238525 -0.521240 0.819824
I (1285) icm test: ESP_OK
I (1285) icm test: -16.707317 2.713415 -1.707317
I (1395) icm test: ESP_OK
I (1395) icm test: -0.233887 -0.545410 0.804688
I (1395) icm test: ESP_OK
I (1395) icm test: -20.701220 2.835366 -0.2134
```
