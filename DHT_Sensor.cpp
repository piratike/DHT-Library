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

void DHT_Sensor::temp_func()
{
    _read(false);
    Serial.print("_data stored: ");
    Serial.print(_data[0]);
    Serial.print(", ");
    Serial.print(_data[1]);
    Serial.print(", ");
    Serial.print(_data[2]);
    Serial.print(", ");
    Serial.print(_data[3]);
    Serial.print(", ");
    Serial.println(_data[4]);
}

float DHT_Sensor::readHumidity(){}

float DHT_Sensor::readTemperatureC(){}

float DHT_Sensor::readTemperatureF(){}

void DHT_Sensor::_delayMicrosecondsNonBlocking(uint32_t time_to_wait)
{

    uint32_t start_time = micros();
    while((micros() - start_time) < time_to_wait) {}

}

void DHT_Sensor::_delayMillisecondsNonBlocking(uint32_t time_to_wait)
{

    time_to_wait *= 1000;
    _delayMicrosecondsNonBlocking(time_to_wait);

}

void DHT_Sensor::_start_transmission()
{

    pinMode(_gpio, OUTPUT);
    digitalWrite(_gpio, LOW);
    _delayMicrosecondsNonBlocking(_start_transmission_low_time);
    pinMode(_gpio, INPUT_PULLUP);
    _delayMicrosecondsNonBlocking(80);

}

uint8_t DHT_Sensor::_read(bool force)
{

    uint32_t current_time = millis();

    // Evita lecturas muy seguidas
    if (!force && (current_time - _last_read_time) < MIN_INTERVAL_BETWEEN_READINGS) {
        return _last_result;
    }
    _last_read_time = current_time;

    memset(_data, 0, sizeof(_data));

    // 🔹 Iniciar transmisión con el tiempo correcto
    _start_transmission();

    noInterrupts();

    if (_expectPulse(LOW) == TIMEOUT || _expectPulse(HIGH) == TIMEOUT) {
        interrupts();
        return DHT_NO_RESPONSE;
    }

    uint32_t cycles[80];

    // 🔹 Leer los 80 pulsos de datos (40 bits)
    for (uint8_t i = 0; i < 80; i += 2) {
        cycles[i] = _expectPulse(LOW);
        cycles[i + 1] = _expectPulse(HIGH);
    }

    interrupts();

    // 🔹 Convertir los ciclos en bits
    for (uint8_t i = 0; i < 40; i++) {
        uint32_t low_cycles = cycles[2 * i];
        uint32_t high_cycles = cycles[2 * i + 1];

        if (low_cycles == TIMEOUT || high_cycles == TIMEOUT) {
            return DHT_TIMEOUT;
        }

        // Cada byte tiene 8 bits
        _data[i / 8] <<= 1;
        if (high_cycles > low_cycles) {
            _data[i / 8] |= 1;
        }
    }

    // 🔹 Verificar Checksum
    if (_data[4] == ((_data[0] + _data[1] + _data[2] + _data[3]) & 0xFF)) {
        return DHT_OK;
    } else {
        return DHT_CHECKSUM_FAIL;
    }

}

uint8_t DHT_Sensor::_identifySensorType()
{

    pinMode(_gpio, OUTPUT);
    
    // 1️⃣ Intentar con tiempo bajo de 2ms (DHT22)
    digitalWrite(_gpio, LOW);
    delayMicroseconds(2000);
    pinMode(_gpio, INPUT_PULLUP);
    delayMicroseconds(40);

    noInterrupts();
    uint32_t responseLow = _expectPulse(LOW);
    uint32_t responseHigh = _expectPulse(HIGH);
    interrupts();

    if (responseLow != TIMEOUT && responseHigh != TIMEOUT) {
        Serial.println("Sensor identificado como DHT22");
        return DHT_22;
    }

    // 2️⃣ Intentar con tiempo bajo de 18ms (DHT11)
    pinMode(_gpio, OUTPUT);
    digitalWrite(_gpio, LOW);
    delay(18);  // 18ms = requerido para DHT11
    pinMode(_gpio, INPUT_PULLUP);
    delayMicroseconds(40);

    noInterrupts();
    responseLow = _expectPulse(LOW);
    responseHigh = _expectPulse(HIGH);
    interrupts();

    if (responseLow != TIMEOUT && responseHigh != TIMEOUT) {
        Serial.println("Sensor identificado como DHT11");
        return DHT_11;
    }

    Serial.println("Error: No se detectó ningún sensor.");
    return 255;  // No detectado

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