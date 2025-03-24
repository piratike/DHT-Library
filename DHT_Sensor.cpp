#include "DHT_Sensor.h"

#define MIN_INTERVAL_BETWEEN_READINGS
#define TIMEOUT UINT32_MAX

DHT_Sensor::DHT_Sensor(uint8_t gpio){}
DHT_Sensor::begin(){}
DHT_Sensor::readHumidity(){}
DHT_Sensor::readTemperatureC(){}
DHT_Sensor::readTemperatureF(){}
DHT_Sensor::_read(bool force){}
DHT_Sensor::_expect_pulse(bool level){}