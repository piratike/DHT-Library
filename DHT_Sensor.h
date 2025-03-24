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

            uint8_t data[5];
            uint8_t _pull_time;
            uint8_t _gpio, _port, _bit;
            uint32_t _last_read_time, _max_cycles;
            bool _last_result;
            bool _read(bool force = false);
            uint32_t _expectPulse(bool level);

    };

#endif