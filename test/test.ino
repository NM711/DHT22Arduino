#include <Arduino.h>
#include "../dht22.hpp"


DHT22Sensor sensor(15);

void setup() {
  Serial.begin(115200);
	Serial.println("(INFO) Running DHT22 Test");
	delay(4000);
}

void loop() {
	sensor.readDataBits();

	ErrorState error = sensor.getErrorState();


	if (error != ErrorState::NONE) {
		Serial.printf("(ERROR) Code: %d, data fallsback to previous successful transmission\n", error);		
		sensor.resetErrorState();
	};


	Serial.printf("(INFO) Temperature C: %f, Temperature F: %f, Humidity: %f\n", sensor.getTemperature(), sensor.getTemperature(TemperatureConversionKind::FARENHEIGHT), sensor.getHumidity());
	
	delay(2000);
}
