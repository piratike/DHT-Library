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
    DHT_11,
    DHT_22
}

DHT_Sensor::DHT_Sensor()
{

    _max_cycles = microsecondsToClockCycles(1000);
    _last_read_time = millis() - MIN_INTERVAL_BETWEEN_READINGS;
    _pull_time = 55;

}

void DHT_Sensor::config(uint8_t gpio)
{

    _gpio = gpio;
    _port = digitalPinToPort(gpio);
    _bit = digitalPinToBitMask(gpio);
    _identifySensorType();

}

void DHT_Sensor::temp_func()
{
    _read(false);
    Serial.print("Data stored: ");
    Serial.print(data[0]);
    Serial.print(", ");
    Serial.print(data[1]);
    Serial.print(", ");
    Serial.print(data[2]);
    Serial.print(", ");
    Serial.print(data[3]);
    Serial.print(", ");
    Serial.println(data[4]);
}

float DHT_Sensor::readHumidity(){}

float DHT_Sensor::readTemperatureC(){}

float DHT_Sensor::readTemperatureF(){}

void DHT_Sensor::_delayMicrosecondsNonBlocking(uint32_t time_to_wait)
{

    uint32_t start_time = micros();
    while((micros() - start_time) < time_to_wait) {}

}

void _delayMillisecondsNonBlocking(uint32_t time_to_wait)
{

    _delayMicrosecondsNonBlocking(time_to_wait * 1000);

}


uint8_t DHT_Sensor::_read(bool force)
{

    uint32_t current_time = millis();

    if(!force && (current_time - _last_read_time) < MIN_INTERVAL_BETWEEN_READINGS)
        return DHT_OK;

    _last_read_time = current_time;
    memset(data, 0, sizeof(data));

    pinMode(_gpio, OUTPUT);
    digitalWrite(_gpio, LOW);
    _delayMicrosecondsNonBlocking(20000);
    pinMode(_gpio, INPUT_PULLUP);
    _delayMicrosecondsNonBlocking(80);

    noInterrupts();

    if (_expectPulse(LOW) == TIMEOUT) return DHT_NO_RESPONSE;
    if (_expectPulse(HIGH) == TIMEOUT) return DHT_TIMEOUT;

    uint32_t cycles[80];
    for(uint8_t i = 0 ; i < 80 ; i += 2)
    {
        cycles[i] = _expectPulse(LOW);
        cycles[i + 1] = _expectPulse(HIGH);
    }

    for(uint8_t i = 0 ; i < 40 ; ++i)
    {
        uint32_t low_cycles = cycles[2 * i], high_cycles = cycles[2 * i + 1];
        if(low_cycles == TIMEOUT || high_cycles == TIMEOUT) return DHT_TIMEOUT;
        data[i / 8] <<= 1;
        if(high_cycles > low_cycles) data[i / 8] |= 1;
    }

    interrupts();

    if(data[4] != ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) return DHT_CHECKSUM_FAIL;
    else return DHT_OK;

}

uint8_t DHT_Sensor::_identifySensorType()
{

    pinMode(_gpio, OUTPUT);
    digitalWrite(_gpio, LOW);
    delay(2);
    pinMode(_gpio, INPUT_PULLUP);
    delayMicroseconds(40);

    noInterrupts();

    uint32_t responseLow = _expectPulse(LOW);
    uint32_t responseHigh = _expectPulse(HIGH);

    interrupts();

    if (responseLow == TIMEOUT || responseHigh == TIMEOUT) {
        pinMode(_gpio, OUTPUT);
        digitalWrite(_gpio, LOW);
        delay(18);
        pinMode(_gpio, INPUT_PULLUP);
        delayMicroseconds(40);

        noInterrupts();
        responseLow = _expectPulse(LOW);
        responseHigh = _expectPulse(HIGH);
        interrupts();

        if (responseLow == TIMEOUT || responseHigh == TIMEOUT) return false;
        return DHT_11;
    }
    
    return DHT_22;

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