#include <DHT_Sensor.h>

DHT_Sensor DHT_1;
DHT_Sensor DHT_2;

void setup() {

  Serial.begin(9600);
  DHT_1.config(6);
  DHT_2.config(7);

}

void loop() {

  Serial.println("DHT 1: ");
  Serial.print("  - Hum: ");
  Serial.print(DHT_1.readHumidity());
  Serial.println(" %");
  Serial.print("  - Temp(ºC): ");
  Serial.print(DHT_1.readTemperatureC());
  Serial.println(" ºC");
  Serial.print("  - Temp(ºF): ");
  Serial.print(DHT_1.readTemperatureF());
  Serial.println(" ºF");
  Serial.print("  - Temp(ºK): ");
  Serial.print(DHT_1.readTemperatureK());
  Serial.println(" ºK");
  Serial.println();

  Serial.println("DHT 2: ");
  Serial.print("  - Hum: ");
  Serial.print(DHT_2.readHumidity());
  Serial.println(" %");
  Serial.print("  - Temp(ºC): ");
  Serial.print(DHT_2.readTemperatureC());
  Serial.println(" ºC");
  Serial.print("  - Temp(ºF): ");
  Serial.print(DHT_2.readTemperatureF());
  Serial.println(" ºF");
  Serial.print("  - Temp(ºK): ");
  Serial.print(DHT_2.readTemperatureK());
  Serial.println(" ºK");
  Serial.println();
  
  delay(2000);

}