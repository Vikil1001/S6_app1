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

int lumiere;
float temperature1;
float temperature2;
float humidite;
float pression;
float pluie;
float vitesse;
int direction;

bool new_value = false;


#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

HardwareSerial SerialUART(1);

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
BLECharacteristic *pRxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"  // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

//set pin for UART com
#define RX_PIN 13
#define TX_PIN 4

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
  }
};

Adafruit_DPS310 dps;
SFEWeatherMeterKit weatherMeterKit(ANALOG_PIN_VENT_DIRECTION, ANALOG_PIN_VENT_VITESSE, ANALOG_PIN_PLUIE);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Démarrer le capteur pression et temperature
  if (!dps.begin_I2C()) {
    Serial.println("Échec de communication avec le DPS310 !");
    while (1);
  }
  
  // set for spark fun
  #ifdef SFE_WMK_PLAFTORM_UNKNOWN
    weatherMeterKit.setADCResolutionBits(10);
  #endif

  // Begin weather meter kit
  weatherMeterKit.begin();

  // Create the BLE Device
  BLEDevice::init("UART Master");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());
  pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_NOTIFY);

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->addServiceUUID(pService->getUUID());
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");

  // Start com for UART
  SerialUART.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
}

void loop() {

  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);                   // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising();  // restart advertising
    Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
  }

  readBarometre();
  readHumidite();
  readVent();
  readPluie();
  readLumiere();

  if (deviceConnected) {
    uint8_t value = 'N';
    pTxCharacteristic->setValue(&value, 1);
    pTxCharacteristic->notify();
  } 
  if (new_value == true){
    String message = "Lumière: " + String(lumiere) +
                    ", Temperature: " + String((temperature1+temperature2)/2, 1) + "°C" +
                    ", Humidité: " + String(humidite, 1) + "%" +
                    ", Pression: " + String(pression, 2) + "hPa" +
                    ", Pluie: " + String(pluie, 1) + "mm" +
                    ", Vent: " + String(vitesse, 1) + "km/h" +
                    ", Direction: " + String(direction) + "°\t";

    SerialUART.println(message);
    new_value = false;
  }
}

void readBarometre(){
  // Create sensor event structs for temperature and pressure
  sensors_event_t temp_event;
  sensors_event_t pressure_event;

  // Get the latest sensor readings
  dps.getEvents(&temp_event, &pressure_event);

  // Print temperature in Celsius
  if (temperature1 != temp_event.temperature) {
    temperature1 = temp_event.temperature;
    new_value = true;
  }

  // Print pressure in Pascals (converted from hPa)
  if (pression != pressure_event.pressure * 100 ){
    pression = pressure_event.pressure * 100;
    new_value = true;
  }

}

void readHumidite(){
  int i, j;
  int duree[42];
  unsigned long pulse;
  byte data[5];
  float humi;
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

  humi = data[0] + (data[1] / 256.0);
  temperature = data [2] + (data[3] / 256.0);
  
  if (humidite != humi){
    humidite = humi;
    new_value = true;
  }

  if (temperature2 != temperature){
    temperature2 = temperature;
    new_value = true;
  }
}

void readVent(){
  pinMode(ANALOG_PIN_VENT_VITESSE, INPUT);
  pinMode(ANALOG_PIN_VENT_DIRECTION, INPUT);

  if (vitesse != weatherMeterKit.getWindSpeed()){
    vitesse = weatherMeterKit.getWindSpeed();
    new_value = true;
  }

  if (direction != weatherMeterKit.getWindDirection()){
    direction = weatherMeterKit.getWindDirection();
    new_value = true;
  }
}

void readPluie(){
  pinMode(ANALOG_PIN_PLUIE, INPUT);

  if (pluie != weatherMeterKit.getTotalRainfall()){
    pluie = weatherMeterKit.getTotalRainfall();
    new_value = true;
  }
}

void readLumiere(){
  pinMode(ANALOG_PIN_LUMINERE, INPUT);
  int lum = analogRead(ANALOG_PIN_LUMINERE);
  if (lumiere  != lum){
    lumiere  = lum;
    new_value = true;
  }
}


