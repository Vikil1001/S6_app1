// Philippe Mabilleau ing.
// mai 2021
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_DPS310.h>
#include "SparkFun_Weather_Meter_Kit_Arduino_Library.h"

#define ANALOG_PIN_LUMINERE 34
#define ANALOG_PIN_TEMPERATURE 16
#define ANALOG_PIN_PLUIE 23
#define ANALOG_PIN_VENT_VITESSE 27
#define ANALOG_PIN_VENT_DIRECTION 35

Adafruit_DPS310 dps;
SFEWeatherMeterKit weatherMeterKit(ANALOG_PIN_VENT_DIRECTION, ANALOG_PIN_VENT_VITESSE, ANALOG_PIN_PLUIE);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Démarrer le capteur
  if (!dps.begin_I2C()) {
    Serial.println("Échec de communication avec le DPS310 !");
    while (1);
  }
  
  #ifdef SFE_WMK_PLAFTORM_UNKNOWN
    // The platform you're using hasn't been added to the library, so the
    // expected ADC values have been calculated assuming a 10k pullup resistor
    // and a perfectly linear 16-bit ADC. Your ADC likely has a different
    // resolution, so you'll need to specify it here:
    weatherMeterKit.setADCResolutionBits(10);
    
    Serial.println(F("Unknown platform! Please edit the code with your ADC resolution!"));
    Serial.println();
  #endif

  // Begin weather meter kit
  weatherMeterKit.begin();

  Serial.println("DPS310 détecté !");
}

void loop() {
  readBarometre();
  readHumidite();
  readVent();
  readPluie();
  readLumiere();
}

void readBarometre(){
  Serial.println("--------------------------");
  // Create sensor event structs for temperature and pressure
  sensors_event_t temp_event;
  sensors_event_t pressure_event;

  // Get the latest sensor readings
  dps.getEvents(&temp_event, &pressure_event);

  // Print temperature in Celsius
  Serial.print("Temp: ");
  Serial.print(temp_event.temperature);
  Serial.println(" °C");

  // Print pressure in Pascals (converted from hPa)
  Serial.print("Pressure: ");
  Serial.print(pressure_event.pressure * 100); // Convert hPa to Pa
  Serial.println(" Pa");

  Serial.println("--------------------------");
}

void readHumidite(){
  Serial.println("--------------------------");
  int i, j;
  int duree[42];
  unsigned long pulse;
  byte data[5];
  float humidite;
  float temperature;
  int broche = 16;

  
  pinMode(broche, OUTPUT_OPEN_DRAIN);
  digitalWrite(broche, HIGH);
  delay(250);
  digitalWrite(broche, LOW);
  delay(20);
  digitalWrite(broche, HIGH);
  delayMicroseconds(40);
  pinMode(broche, INPUT_PULLUP);
  
  while (digitalRead(broche) == HIGH);
  i = 0;

  do {
    pulse = pulseIn(broche, HIGH);
    duree[i] = pulse;
    i++;
  } while (pulse != 0);
 
  if (i != 42) 
    Serial.printf(" Erreur timing \n"); 

  for (i=0; i<5; i++) {
    data[i] = 0;
    for (j = ((8*i)+1); j < ((8*i)+9); j++) {
      data[i] = data[i] * 2;
      if (duree[j] > 50) {
        data[i] = data[i] + 1;
      }
    }
  }

  if ( (data[0] + data[1] + data[2] + data[3]) != data[4] ) 
    Serial.println(" Erreur checksum");

  humidite = data[0] + (data[1] / 256.0);
  temperature = data [2] + (data[3] / 256.0);
  Serial.printf("Humidite = %4.0f \%%  Temperature = %4.2f degreC \n", humidite, temperature);
  Serial.println("--------------------------");
}

void readVent(){
  Serial.println("--------------------------");
  pinMode(ANALOG_PIN_VENT_VITESSE, INPUT);
  pinMode(ANALOG_PIN_VENT_DIRECTION, INPUT);

  Serial.print("Vitesse value : ");
  //int analogValueVitesse = analogRead(ANALOG_PIN_VENT_VITESSE);  // Range: 0–4095
  //Serial.println(analogValueVitesse);
  Serial.println(weatherMeterKit.getWindSpeed());

  Serial.print("Direction value : ");
  //int analogValueDirection = analogRead(ANALOG_PIN_VENT_DIRECTION);  // Range: 0–4095
  //Serial.println(analogValueDirection);
  Serial.println(weatherMeterKit.getWindDirection());
  Serial.println("--------------------------");
}

void readPluie(){
  Serial.println("--------------------------");
  pinMode(ANALOG_PIN_PLUIE, INPUT);

  Serial.print("Pluie value : ");
  //int analogValuePluie = analogRead(ANALOG_PIN_PLUIE);  // Range: 0–4095
  //Serial.println(analogValuePluie);
  Serial.println(weatherMeterKit.getTotalRainfall());
  Serial.println("--------------------------");
}

void readLumiere(){
  Serial.println("--------------------------");
  pinMode(ANALOG_PIN_LUMINERE, INPUT);

  Serial.print("Lumiere value : ");
  int analogValueLumiere = analogRead(ANALOG_PIN_LUMINERE);  // Range: 0–4095
  Serial.println(analogValueLumiere);
  Serial.println("--------------------------");
}


