#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 2

OneWire oneWire(ONE_WIRE_BUS);

DallasTemperature sensors(&oneWire);

 float Celcius=0;
// float Fahrenheit=0;
 
void setup(void)
{
  
  Serial.begin(9600);
  sensors.begin();
}

void loop(void)
{ 
  sensors.requestTemperatures(); 
  Celcius=sensors.getTempCByIndex(0);
//  Fahrenheit=sensors.toFahrenheit(Celcius);
  Serial.print(Celcius);
  Serial.println(" C");

//  Serial.print(" F  ");
//  Serial.println(Fahrenheit);
//  delay(1000);
}
