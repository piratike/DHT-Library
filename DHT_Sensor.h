#ifndef DHT_SENDOR_H
#define DHT_SENSOR_H

    #include <Arduino.h>

    /**
     * @brief Class for handling DHT temperature and humidity sensors (DHT11 & DHT22).
     */
    class DHT_Sensor {

        public:

            /**
             * @brief Constructor for the DHT_Sensor class.
             * Initializes default values.
             */
            DHT_Sensor();

            /**
             * @brief Configures the sensor and automatically detects its type.
             * @param gpio GPIO pin number where the sensor is connected.
             */
            void config(uint8_t gpio);

            /**
             * @brief Reads the temperature in degrees Celsius.
             * @return Temperature in °C.
             */
            float readTemperatureC();

            /**
             * @brief Reads the temperature in degrees Fahrenheit.
             * @return Temperature in °F.
             */
            float readTemperatureF();

            /**
             * @brief Reads the relative humidity percentage.
             * @return Humidity in %.
             */
            float readHumidity();

        private:

            uint8_t _data[5];                       // Stores raw data received from the sensor.
            uint8_t _type;                          // Detected sensor type (DHT11 or DHT22).
            uint8_t _gpio;                          // GPIO pin number.
            uint8_t _port;                          // Corresponding port for the GPIO.
            uint8_t _bit;                           // Bitmask for reading the port state.
            uint16_t _start_transmission_low_time;  // Pulse duration to start communication.
            uint32_t _last_read_time;               // Timestamp of the last successful reading.
            uint32_t _max_cycles;                   // Maximum number of cycles to wait for a signal.
            bool _last_result;                      // Last reading result (success or failure).

            /**
             * @brief Performs a non-blocking delay in microseconds.
             * @param time_to_wait Time to wait in microseconds.
             */
            void _delayMicrosecondsNonBlocking(uint32_t time_to_wait);

            /**
             * @brief Initiates data transmission by sending a LOW pulse to the sensor.
             * @param low_time Duration of the LOW pulse in microseconds.
             *                 If 0, the default _start_transmission_low_time is used.
             */
            void _start_transmission(uint16_t low_time = 0);

            /**
             * @brief Reads data from the sensor and stores it in _data[].
             * @param force If true, forces a reading, bypassing the minimum wait time.
             * @return Status code indicating if the reading was successful or failed.
             */
            uint8_t _read(bool force = false);

            /**
             * @brief Automatically identifies the connected sensor type (DHT11 or DHT22).
             * @return Sensor type code (DHT_11 or DHT_22).
             */
            uint8_t _identifySensorType();

            /**
             * @brief Waits for the pin to change to a desired logic level and measures the duration.
             * @param level Expected logic level (HIGH or LOW).
             * @return Pulse duration in clock cycles or TIMEOUT if exceeded.
             */
            uint32_t _expectPulse(bool level);

    };

#endif