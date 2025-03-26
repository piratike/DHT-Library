#ifndef DHT_SENDOR_H
#define DHT_SENSOR_H

    #include <Arduino.h>

    class DHT_Sensor {

        public:

            DHT_Sensor();
            void config(uint8_t gpio);
            float readTemperatureC();
            float readTemperatureF();
            float readHumidity();
            void temp_func();

        private:

            uint8_t _data[5];
            uint8_t _type;
            uint8_t _gpio, _port, _bit;
            uint16_t _start_transmission_low_time;
            uint32_t _last_read_time, _max_cycles;
            bool _last_result;
            void _delayMicrosecondsNonBlocking(uint32_t time_to_wait);
            void _delayMillisecondsNonBlocking(uint32_t time_to_wait);
            void _start_transmission();
            uint8_t _read(bool force = false);
            uint8_t _identifySensorType();
            uint32_t _expectPulse(bool level);

    };

#endif