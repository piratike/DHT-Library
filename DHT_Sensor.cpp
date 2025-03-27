#include "Arduino.h"
#include "DHT_Sensor.h"

#define MIN_INTERVAL_BETWEEN_READINGS 2000  // Minimum interval between readings (ms)
#define TIMEOUT UINT32_MAX                  // Maximum wait time before timing out

/**
 * @brief Enum for sensor reading status codes.
 */
enum DHT_Status {
    DHT_OK,             // Reading successful
    DHT_TIMEOUT,        // Sensor response timeout
    DHT_CHECKSUM_FAIL,  // Checksum validation failed
    DHT_NO_RESPONSE,    // No response from sensor
    DHT_UNKNOWN_ERROR   // Unknown error
};

/**
 * @brief Enum for supported DHT sensor types.
 */
enum DHT_TYPES {
    NO_TYPE,    // No sensor detected
    DHT_11,     // DHT11 sensor
    DHT_22      // DHT22 sensor
};

/**
 * @brief Constructor: Initializes timing parameters.
 */
DHT_Sensor::DHT_Sensor()
{

    _max_cycles = microsecondsToClockCycles(1000);

    // Ensure first read is allowed immediately
    _last_read_time = millis() - MIN_INTERVAL_BETWEEN_READINGS;

}

/**
 * @brief Configures the sensor by detecting its type and setting up GPIO.
 * @param gpio GPIO pin number where the sensor is connected.
 */
void DHT_Sensor::config(uint8_t gpio)
{

    _gpio = gpio;
    _port = digitalPinToPort(gpio);
    _bit = digitalPinToBitMask(gpio);
    _type = _identifySensorType();

    // Set correct start signal duration
    _start_transmission_low_time = (_type == DHT_11) ? 20000 : 2000;

}

/**
 * @brief Reads and returns the humidity percentage.
 * @return Humidity in % or NAN if the reading fails.
 */
float DHT_Sensor::readHumidity()
{

  if(_read(false) != DHT_OK) return NAN;
  if(_type == DHT_11) return _data[0];
  else return ((_data[0] << 8) | _data[1]) * 0.1;

}

/**
 * @brief Reads and returns the temperature in Celsius.
 * @return Temperature in °C or NAN if the reading fails.
 */
float DHT_Sensor::readTemperatureC()
{

  if(_read(false) != DHT_OK) return NAN;
  if(_type == DHT_11) return _data[2];
  else
  {

    // Convert to signed 16-bit value
    int16_t temperature = ((_data[2] & 0x7F) << 8) | _data[3];
    // Apply negative sign if necessary
    if(_data[2] & 0x80) temperature *= -1;
    // Return temperature value readed
    return temperature * 0.1;

  }

}

/**
 * @brief Reads and returns the temperature in Fahrenheit.
 * @return Temperature in °F or NAN if the reading fails.
 */
float DHT_Sensor::readTemperatureF()
{

  float temperature_in_c = readTemperatureC();
  return isnan(temperature_in_c) ? NAN : temperature_in_c * 1.8 + 32;

}

/**
 * @brief Reads and returns the temperature in Kelvin.
 * @return Temperature in K or NAN if the reading fails.
 */
float DHT_Sensor::readTemperatureK()
{

  float temperature_in_c = readTemperatureC();
  return isnan(temperature_in_c) ? NAN : temperature_in_c + 273.15;

}

/**
 * @brief Implements a non-blocking delay in microseconds.
 * @param time_to_wait Time to wait in microseconds.
 */
void DHT_Sensor::_delayMicrosecondsNonBlocking(uint32_t time_to_wait)
{

    uint32_t start_time = micros();
    while((micros() - start_time) < time_to_wait) {}

}

/**
 * @brief Initiates communication with the sensor by sending a LOW signal.
 * @param low_time Duration of the LOW pulse in microseconds.
 */
void DHT_Sensor::_start_transmission(uint16_t low_time)
{

    pinMode(_gpio, OUTPUT);
    digitalWrite(_gpio, LOW);
    _delayMicrosecondsNonBlocking(low_time ? low_time : _start_transmission_low_time);
    pinMode(_gpio, INPUT_PULLUP);
    _delayMicrosecondsNonBlocking(80); // Wait for sensor response

}

/**
 * @brief Reads data from the DHT sensor and stores it in the _data[] buffer.
 * 
 * This function initiates communication with the sensor, waits for its response, 
 * and then reads 40 bits (5 bytes) of data. The function also ensures that 
 * readings are taken at appropriate intervals to prevent sensor errors.
 * 
 * @param force If true, forces a reading even if the minimum interval has not elapsed.
 * @return Status code indicating the result of the reading (DHT_OK, DHT_TIMEOUT, etc.).
 */
uint8_t DHT_Sensor::_read(bool force)
{

    uint32_t current_time = millis();

    // Enforce the minimum interval between readings unless 'force' is true
    if (!force && (current_time - _last_read_time) < MIN_INTERVAL_BETWEEN_READINGS) return _last_result;

    _last_read_time = current_time;     // Update last read time
    memset(_data, 0, sizeof(_data));    // Clear previous data buffer
    _start_transmission();              // Start the transmission sequence to initiate sensor communication
    noInterrupts();                     // Disable interrupts to ensure precise timing

    // Wait for the sensor's initial response (low and high signals)
    if (_expectPulse(LOW) == TIMEOUT || _expectPulse(HIGH) == TIMEOUT)
    {
        interrupts(); // Re-enable interrupts before returning
        return DHT_NO_RESPONSE;
    }

    uint32_t cycles[80];                // Array to store pulse durations (each bit consists of a low and a high pulse)
    for (uint8_t i = 0; i < 80; i++)    // Capture the duration of 80 signal transitions (40 bits = 80 edges)
        cycles[i] = _expectPulse(i % 2 == 0 ? LOW : HIGH);

    interrupts(); // Re-enable interrupts now that timing-sensitive operations are complete

    for (uint8_t i = 0; i < 40; i++) // Process the captured pulses into meaningful data
    {

        uint32_t low_cycles = cycles[2 * i];        // Duration of the LOW part of the bit
        uint32_t high_cycles = cycles[2 * i + 1];   // Duration of the HIGH part of the bit

        // If we detect a timeout, return an error
        if (low_cycles == TIMEOUT || high_cycles == TIMEOUT) return DHT_TIMEOUT;

        _data[i / 8] <<= 1; // Shift data left to make room for the next bit

        // If the HIGH pulse lasted longer than the LOW pulse, it's a '1'
        if (high_cycles > low_cycles) _data[i / 8] |= 1;

    }

    // Verify the checksum (last byte should be the sum of the first 4 bytes)
    uint8_t checksum = (_data[0] + _data[1] + _data[2] + _data[3]) & 0xFF;
    
    return (_data[4] == checksum) ? DHT_OK : DHT_CHECKSUM_FAIL;

}

/**
 * @brief Identifies the DHT sensor type (DHT11 or DHT22).
 * 
 * This function attempts to determine the sensor type by sending two different 
 * start signals (one for DHT22 and one for DHT11) and checking the response.
 * 
 * @return The detected sensor type (DHT_11, DHT_22, or NO_TYPE if detection fails).
 */
uint8_t DHT_Sensor::_identifySensorType()
{

    // Test durations for start signals corresponding to each sensor type
    const uint16_t test_durations[] = { 2000, 18000 };
    const uint8_t sensor_types[] = { DHT_22, DHT_11 };

    // Try both test signals (DHT22 first, then DHT11)
    for(uint8_t i = 0 ; i < 2 ; i++)
    {

        // Start transmission with the test duration for the current sensor type
        _start_transmission(test_durations[i]);

        // Disable interrupts to ensure accurate timing
        noInterrupts();

        // Check for response pulses (sensor should pull the line LOW, then HIGH)
        uint32_t respone_low = _expectPulse(LOW);
        uint32_t respone_high = _expectPulse(HIGH);

        // Re-enable interrupts after timing-sensitive operations
        interrupts();

        // If both pulses were detected within a reasonable time, we found a sensor
        if(respone_low != TIMEOUT && respone_high != TIMEOUT)
            return sensor_types[i]; // Return the detected sensor type

    }

    // If no response was detected for either test, return NO_TYPE
    return NO_TYPE;

}

/**
 * @brief Waits for a signal transition on the data pin and measures its duration.
 * 
 * This function loops until the pin changes to the expected signal level (HIGH or LOW),
 * counting how long the pin stays in its current state. If the wait time exceeds 
 * a predefined threshold, it returns a timeout.
 * 
 * @param level The expected signal level to wait for (LOW or HIGH).
 * @return The duration (in clock cycles) the signal remained in the expected state,
 *         or TIMEOUT if the signal did not transition within the allowed time.
 */
uint32_t DHT_Sensor::_expectPulse(bool level)
{

    // Get the pointer to the port register corresponding to the GPIO pin
    volatile uint8_t *port_register = portInputRegister(_port);

    // Define the expected port state (bit mask for the pin)
    uint8_t port_state = level ? _bit : 0;

    // Counter to measure the duration of the pulse
    uint16_t count = 0;

    // Loop until the pin changes state or the timeout threshold is reached
    while(count < _max_cycles)
    {
        if((*port_register & _bit) != port_state) break; // If pin state changes, exit loop
        ++count; // Increment counter for each cycle the pin remains in the expected state
    }

    // Return the measured pulse duration or TIMEOUT if the loop reached the max count
    return (count >= _max_cycles) ? TIMEOUT : count;

}