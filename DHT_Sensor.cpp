#include "DHT_Sensor.h"

#define MIN_INTERVAL_BETWEEN_READINGS
#define TIMEOUT UINT32_MAX

DHT_Sensor::DHT_Sensor()
{

    _max_cycles = microsecondsToClockCycles(1000);
    _last_read_time = millis() - MIN_INTERVAL_BETWEEN_READINGS;
    _pull_time = 55; // Default value from other DHT libraries

}

DHT_Sensor::config(uint8_t gpio)
{

    _port = digitalPinToPort(gpio);
    _bit = digitalPinToBitMask(gpio);

}

DHT_Sensor::readHumidity(){}

DHT_Sensor::readTemperatureC(){}

DHT_Sensor::readTemperatureF(){}

DHT_Sensor::_read(bool force){}

DHT_Sensor::_expectPulse(bool level)
{

    volatile uint8_t *port_register = portInputRegister(_port);
    uint8_t port_state = level ? _bit : 0;
    uint16_t count = 0;

    while(count < _max_cycles)
    {
        if((*port_register & _bit) != port_state) break;
        ++count;
    }

    return (count >= _max_cycles) ? TIMEOUT : count;

}