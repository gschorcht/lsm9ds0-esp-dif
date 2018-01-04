# Driver for the LSM9DS0 module: 3D accelerometer, 3D gyroscope, 3D magnetometer

The driver is for the usage with the ESP8266 and [esp-open-rtos](https://github.com/SuperHouse/esp-open-rtos). If you can't find it in folder [extras/lsm9ds0](https://github.com/SuperHouse/esp-open-rtos/tree/master/extras) of original repository, it is not yet merged. Please take a look to branch [lsm9ds0](https://github.com/gschorcht/esp-open-rtos/tree/lsm9ds0) of my fork in that case.

It is also working with ESP32 and [ESP-IDF](https://github.com/espressif/esp-idf.git) using a wrapper component for ESP8266 functions, see folder ```components/esp8266_wrapper```, as well as Linux based systems using a wrapper library.

## Usage 

LSM9DS0 is simply a combination of the L3GD20H 3D gyroscope and the LSM303D 3D accelerometer and 3D magnetometer module in only one package. Both sensors are addressable by separate I2C addresses. Therefore, the driver is simply mapping to L3GD20H and LSM303D drivers, see in example

## Limitations

At the moment it is only working with I2C interface.
