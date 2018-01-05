# Driver for the LSM9DS0 module: 3D accelerometer, 3D gyroscope, 3D magnetometer

The driver is for the usage with the ESP8266 and [esp-open-rtos](https://github.com/SuperHouse/esp-open-rtos). If you can't find it in folder [extras/lsm9ds0](https://github.com/SuperHouse/esp-open-rtos/tree/master/extras) of original repository, it is not yet merged. Please take a look to branch [lsm9ds0](https://github.com/gschorcht/esp-open-rtos/tree/lsm9ds0) of my fork in that case.

It is also working with ESP32 and [ESP-IDF](https://github.com/espressif/esp-idf.git) using a wrapper component for ESP8266 functions, see folder ```components/esp8266_wrapper```, as well as Linux based systems using a wrapper library.

## About the sensor

LSM9DS0 is simply a combination of two sensors

- the LSM303D 3D accelerometer and 3D magnetometer module, and
- the L3GD20 3D gyroscope sensor

in only one package. Both sensors are addressable seperatly either using different I2C addresses or different SPI CS slave select signals.

## Driver 

The LSM9DS0 driver is simply a wrapper around the L3GD20H and LSM303D drivers, see example [lsm9ds0_example.c](https://github.com/gschorcht/lsm9ds0-esp-dif/blob/master/main/lsm9ds0_example.c).

**Please note:** Alternatively, you could use the sensors with both drivers separately.

For each L3GD20H symbol and each LSM303 symbol there is a mapping to LSM9DS symbols. Thereby, the starts with ```lsm9ds0_``` and are extended according to following rules

Extension  | Meaning
:--------- |:-------
```_am_``` | symbols related to the whole LSM303D sensor
```_a_```  | symbols related to LSM303D's accelerometer functionality
```_m_```  | symbols related to LSM303D's magnetometer functionality
```_g_```  | symbols refering the whole L3GD20H sensor and its functionality 

Following table is showing some examples. Please refer file [lsm9ds0.h](https://github.com/gschorcht/lsm9ds0-esp-dif/blob/master/components/lsm9ds0/lsm9ds0.h) for complete mapping.

Symbol  | Related to | Mapped to 
:---------|:------ |:-------
```lsm9ds0_init_am_sensor``` | whole LSM303D sensor | ```lsm303d_init_sensor``` 
```lsm9ds0_init_g_sensor```  | whole L3GD20 sensor | ```l3gd20h_init_sensor```
```lsm9ds0_set_a_fifo_mode```| LMS303D accelerator FIFO | ```lsm303d_set_fifo_mode```
```lsm9ds0_set_g_fifo_mode```| L3GD20 FIFO | ```l3gd20h_set_fifo_mode```
```lsm9ds0_int_am_signal1``` | LSM303D sensor interrupt signal INT1 | ```lsm303d_int1_signal```
```lsm9ds0_set_a_mode```     | LSM303D accelerator mode | ```lsm303d_set_a_mode```
```lsm9ds0_set_m_mode```     | LSM303D magnetometer mode | ```lsm303d_set_m_mode```
```lsm9ds0_set_g_mode```     | L3GD20 gyroscope mode | ```lsm303d_set_g_mode```
```lsm9ds0_get_int_am_data_source``` | any LSM303D data source | ```lsm303d_get_int_data_source```
```lsm9ds0_get_int_g_data_source``` | L3GD20 gyroscope data source | ```l3gd20h_get_int_data_source```
```lsm9ds0_get_float_a_data```     | LSM303D accelerator data | ```lsm303d_get_float_a_data```
```lsm9ds0_get_float_m_data```     | LSM303D magnetometer data | ```lsm303d_get_float_m_data```
```lsm9ds0_get_float_g_data```     | L3GD20 gyroscope data | ```l3gd20h_get_float_data```
...|...|...

Please refer the ```README.md``` of the respective sensor drivers for details how to use. ([LSM303D](https://github.com/gschorcht/lsm303d-esp-idf) and [L3GD20H](https://github.com/gschorcht/l3gd20h-esp-idf)).

## Hardware Configuration

Hardware configuration might be a challenge especially with ESP8266 since the LSM9DS0 sensor needs up to 9 GPIOs if you want to use SPI together with all 4 interrupt signals. Possible configurations for I2C and SPI are shown in followng pseudo graphics.

### I2C Configuration

```
  +-----------------+     +----------+            +-----------------+     +----------+
  | ESP32           |     | LSM9DS0  |            | ESP8266         |     | LSM9DS0  |
  |                 |     |          |            |                 |     |          |
  |   GPIO 14 (SCL) ------> SCL      |            |   GPIO 14 (SCL) ------> SCL      |
  |   GPIO 13 (SDA) <-----> SDA      |            |   GPIO 13 (SDA) <-----> SDA      |
  |   GPIO 5        <------ INT1_XM  |            |   GPIO 5        <------ INT1_XM  |
  |   GPIO 4        <------ INT2_XM  |            |   GPIO 4        <------ INT2_XM  |
  |   GPIO 22       <------ INT_G    |            |   GPIO 2        <------ INT_G    |
  |   GPIO 23       <------ DRDY_G   |            |   GPIO 0        <------ DRDY_G   |
  +-----------------+     +----------+            +-----------------+     +----------+
```

### SPI Configuration

```
  +-----------------+     +----------+            +-----------------+     +----------+
  | ESP32           |     | LSM9DS0  |            | ESP8266         |     | LSM9DS0  |
  |                 |     |          |            |                 |     |          |
  |   GPIO 16 (SCK) ------> SCK      |            |   GPIO 14 (SCK) ------> SCK      |
  |   GPIO 17 (MOSI)------> SDI      |            |   GPIO 13 (MOSI)------> SDI      |
  |   GPIO 18 (MISO)<--+--- SDO_XM/G |            |   GPIO 12 (MISO)<--+--- SDO_XM/G |
  |                 |  +--- SDO_G    |            |                 |  +--- SDO_G    |
  |   GPIO 19 (CS1) ------> CS_XM    |            |   GPIO 2  (CS1) ------> CS_XM    |
  |   GPIO 21 (CS2) ------> CS_G     |            |   GPIO 0  (CS2) ------> CS_G     |
  |   GPIO 5        <------ INT1_XM  |            |   GPIO 5        <------ INT1_XM  |
  |   GPIO 4        <------ INT2_XM  |            |   GPIO 4        <------ INT2_XM  |
  |   GPIO 22       <------ INT_G    |            |   GPIO 16 !!!   <------ INT_G    |
  |   GPIO 23       <------ DRDY_G   |            |   GPIO 15 !!!   <------ DRDY_G   |
  +-----------------+      +---------+            +-----------------+      +---------+
```

**Please note** Using GPIOs 15 and 16 as interrupts signals might lead to problems on ESP8266 when you flash the program. SPI configuration works without any problems if interrupts are not used or only GPIO 5 and 4 are used as interrupt signals.

