# DHT Sensor Library

A simple and easy-to-use library to read temperature and humidity from DHT11 and DHT22 sensors.

## Features
- Supports DHT11 and DHT22 sensors.
- Reads temperature in Celsius, Fahrenheit, and Kelvin.
- Reads humidity as a percentage.
- Handles sensor communication and errors.

## Installation

1. Download the repository or use the Arduino Library Manager to install it.
2. Include the library in your sketch:
   ```cpp
   #include <DHT_Sensor.h>

## Usage

### Initialize the Sensor
    ```cpp
    DHT_Sensor dht;
    dht.config(2);  // GPIO pin connected to the DHT sensor

### Read Humidity
    ```cpp
    float humidity = dht.readHumidity();

### Read Temperature
    ```cpp
    // In Celsius
    float temperature = dht.readTemperatureC();

    // In Fahrenheit
    float temperature = dht.readTemperatureF();

    // In Kelvin
    float temperature = dht.readTemperatureK();

## Example
    ```cpp
    #include <DHT_Sensor.h>

    DHT_Sensor dht;

    void setup() {
    Serial.begin(9600);
    dht.config(2);  // GPIO pin connected to the DHT sensor
    }

    void loop() {
    float humidity = dht.readHumidity();
    float temperatureC = dht.readTemperatureC();

    if (isnan(humidity) || isnan(temperatureC)) {
        Serial.println("Error reading sensor data.");
    } else {
        Serial.print("Humidity: ");
        Serial.print(humidity);
        Serial.print(" %, Temperature: ");
        Serial.print(temperatureC);
        Serial.println(" °C");
    }

    delay(2000);  // Wait for 2 seconds
    }
