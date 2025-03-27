#include "Arduino.h"
#include "DHT_Sensor.h"

#define MIN_INTERVAL_BETWEEN_READINGS 2000
#define TIMEOUT UINT32_MAX

enum DHT_Status {
    DHT_OK,
    DHT_TIMEOUT,
    DHT_CHECKSUM_FAIL,
    DHT_NO_RESPONSE,
    DHT_UNKNOWN_ERROR
};

enum DHT_TYPES {
    NO_TYPE,
    DHT_11,
    DHT_22
};

DHT_Sensor::DHT_Sensor()
{

    _max_cycles = microsecondsToClockCycles(1000);
    _last_read_time = millis() - MIN_INTERVAL_BETWEEN_READINGS;

}

void DHT_Sensor::config(uint8_t gpio)
{

    _gpio = gpio;
    _port = digitalPinToPort(gpio);
    _bit = digitalPinToBitMask(gpio);
    _type = _identifySensorType();
    _start_transmission_low_time = (_type == DHT_11) ? 20000 : 2000;

}

float DHT_Sensor::readHumidity()
{

  if(_read(false) != DHT_OK) return NAN;
  if(_type == DHT_11) return _data[0];
  else return ((_data[0] << 8) | _data[1]) * 0.1;

}

float DHT_Sensor::readTemperatureC()
{

  if(_read(false) != DHT_OK) return NAN;
  if(_type == DHT_11) return _data[2];
  else
  {

    int16_t temperature = ((_data[2] & 0x7F) << 8) | _data[3];
    if(_data[2] & 0x80) temperature *= -1;
    return temperature * 0.1;

  }

}

float DHT_Sensor::readTemperatureF()
{

  float temperature_in_c = readTemperatureC();
  return isnan(temperature_in_c) ? NAN : temperature_in_c * 1.8 + 32;

}

float DHT_Sensor::readTemperatureK()
{

  float temperature_in_c = readTemperatureC();
  return isnan(temperature_in_c) ? NAN : temperature_in_c + 273.15;

}

void DHT_Sensor::_delayMicrosecondsNonBlocking(uint32_t time_to_wait)
{

    uint32_t start_time = micros();
    while((micros() - start_time) < time_to_wait) {}

}

void DHT_Sensor::_start_transmission(uint16_t low_time)
{

    pinMode(_gpio, OUTPUT);
    digitalWrite(_gpio, LOW);
    _delayMicrosecondsNonBlocking(low_time ? low_time : _start_transmission_low_time);
    pinMode(_gpio, INPUT_PULLUP);
    _delayMicrosecondsNonBlocking(80);

}

uint8_t DHT_Sensor::_read(bool force)
{

    uint32_t current_time = millis();

    if (!force && (current_time - _last_read_time) < MIN_INTERVAL_BETWEEN_READINGS) return _last_result;
    _last_read_time = current_time;
    memset(_data, 0, sizeof(_data));

    _start_transmission();
    noInterrupts();

    if (_expectPulse(LOW) == TIMEOUT || _expectPulse(HIGH) == TIMEOUT)
    {
        interrupts();
        return DHT_NO_RESPONSE;
    }

    uint32_t cycles[80];
    for (uint8_t i = 0; i < 80; i++)
        cycles[i] = _expectPulse(i % 2 == 0 ? LOW : HIGH);

    interrupts();

    for (uint8_t i = 0; i < 40; i++)
    {

        uint32_t low_cycles = cycles[2 * i];
        uint32_t high_cycles = cycles[2 * i + 1];

        if (low_cycles == TIMEOUT || high_cycles == TIMEOUT) return DHT_TIMEOUT;

        _data[i / 8] <<= 1;
        if (high_cycles > low_cycles) _data[i / 8] |= 1;

    }

    return (_data[4] == ((_data[0] + _data[1] + _data[2] + _data[3]) & 0xFF)) ? DHT_OK : DHT_CHECKSUM_FAIL;

}

uint8_t DHT_Sensor::_identifySensorType()
{

    const uint16_t test_durations[] = { 2000, 18000 };
    const uint8_t sensor_types[] = { DHT_22, DHT_11 };

    for(uint8_t i = 0 ; i < 2 ; i++)
    {

        _start_transmission(test_durations[i]);
        noInterrupts();
        uint32_t respone_low = _expectPulse(LOW);
        uint32_t respone_high = _expectPulse(HIGH);
        interrupts();

        if(respone_low != TIMEOUT && respone_high != TIMEOUT)
            return sensor_types[i];

    }

    return NO_TYPE;

}

uint32_t DHT_Sensor::_expectPulse(bool level)
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