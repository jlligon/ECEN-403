#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h> //Library to use BLE as server ?? see if needed
//#include <BLEAdvertisedDevice.h>
#include "sdkconfig.h"
/*
    Adapted code based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleWrite.cpp
    Ported to Arduino ESP32 by Evandro Copercini
    Adapted and Modified by Kennedy Jackson - ECEN 404
*/



// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

//Add BLE Services
#define Heart_Rate_Service_UUID "0x180D"
//#define Accelerometer_Service_UUID "21688e72-1add-4a67-a83d-075927ddb2cd" //self-generated UUID
//#define Battery_Level_Service_UUID "0x180F"

//Add BLE Characteristics
#define Heart_Rate_Char_UUID "0x2A37"
BLECharacteristic Heart_Rate_Characteristic 
                      (Heart_Rate_Char_UUID, 
                      BLECharacteristic::PROPERTY_READ | 
                      BLECharacteristic::PROPERTY_NOTIFY);
                      
//#define Accelerometer_Char_UUID "0x27BA" //(this is step per minute)
//#define Battery_Level_Char_UUID "0x2A19"

//Add code for getting date/time to send it via BLE from mobile device


//setup GPIO mapping (ADD resolution(sigfig))
  const int heart_rate_pin = 8;  //AI 
  //const int accelerometer_pin = 9; //AI
  //const int GPOUT_feulg_pin = ; //GPOUT Pin 12 Fuel Gauge DI 
  //const int LED_lowbatt_pin = ; // DO for low batt indic  
  //const int Emergnecy_button_pin = ;  // DO for emergency contact
  
//setup Analog variables
  int heart_rate = 0; 
  //int accelerometer = 0; 
  //int battery_charge = 0;

//setting updates
  int Updated_heart_rate = 0 ;
  //int Updated_accelerometer = 0;
  //int Updated_battery_charge = 0;

//boolCharacteristics
//initalize false to ensure connection
  bool Phone_connected = false;

//add code for callbacks for each BLEaction  
class MyCallbacks: public BLEServerCallbacks {
    //void onRead(BLECharacteristic *pCharacteristic) {
    void onConnect(BLEServer* pServer) {
      Phone_connected = true;
    };
//add onDisconnect startadV again to make sure it works if disconnected
    void onDisconnect(BLEServer* pServer) {
      Phone_connected = false;
      BLEDevice::startAdvertising();
    };
      /*if (Updated_heart_rate != heart_rate){
       Serial.print("Heart Rate (bpm): ");
       Serial.println (heart_rate); //show values just for testing
       Updated_heart_rate = heart_rate;
        }

      if (Updated_accelerometer != accelerometer){
       Serial.println("Steps: ", accelerometer); //show values just for testing
       pAccelerometer_Characteristic.writeValue(accelerometer);
       Updated_accelerometer = accelerometer; 
        }
        
      if (Updated_battery_charge != battery_charge){
       Serial.println("Battery Percent: ",battery_charge); //show values just for testing
       pBattery_Level_Characteristic.writeValue(battery_charge); 
       Updated_battery_charge = battery_charge; 
      }
      */
};

//add onRead, onNotify Callback


void InitBLE() {
  BLEDevice::init("FitTrak");

  //BLE Server
  BLEServer *pServer = BLEDevice::createServer();  
  pServer->setCallbacks(new MyCallbacks());
  BLEService *pHeart = pServer->createService(Heart_Rate_Char_UUID);
  pHeart->addCharacteristic(&Heart_Rate_Characteristic);
  Heart_Rate_Characteristic.setValue(heart_rate);
     
 /* BLEService *pAccel = pServer->createService(Accelerometer_Service_UUID);
  BLECharacteristic *pAccelerometer_Characteristic = createCharacteristic(Accelerometer_Char_UUID, 
                      BLECharacteristic::PROPERTY_READ | 
                      BLECharacteristic::PROPERTY_NOTIFY);
                       
  pService.addCharacteristic(pAccelerometer_Characteristic);
  pAccelerometer_Characteristic.setValue(accelerometer);

  BLEService *pBattery = pServer->createService(Battery_Service_UUID);
  BLECharacteristic *pBattery_Level_Characteristic = createCharacteristic(Battery_Level_Char_UUID, 
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_NOTIFY);
                      
  pService.addCharacteristic(pBattery_Level_Characteristic);
  pBattery_Level_Characteristic.setValue(battery_charge);
*/
  pServer->getAdvertising()->addServiceUUID(Heart_Rate_Service_UUID);
  pHeart->start();
  pServer->getAdvertising()->start();

  //pAdvertise if this doesn't work go back to original method (below requires addit. lib)
  //BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  //pAdvertising->addServiceUUID(SERVICE_UUID);
  //pAdvertising->setScanResponse(true);
  //pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  //pAdvertising->setMinPreferred(0x12);
  //BLEDevice::startAdvertising();
}

void setup() {
  Serial.begin(115200);
  //Analog I/O for Heart Rate Monitor
  pinMode(heart_rate_pin, INPUT);
  
  //Analog I/O for Accelerometer
  //pinMode(accelerometer_pin, INPUT);
  
  //Analog I/O for the battery reading
  //pinMode(GPOUT_feulg_pin, INPUT);
  
  //Digital output for lowbattery indication
  //pinMode(LED_lowbatt_pin, OUTPUT);
  
  //Digital input for Emergency Contact 
  //pinMode(Emergnecy_button_pin, INPUT);

  InitBLE();
}

void loop() {
  // main code here  
  // this is the extrnal view from laptop script (non-BLE)
  
  // reading inputs
   heart_rate = analogRead(heart_rate_pin);   
  // accelerometer = analogRead(accelerometer_pin);
  // battery_charge  = digitalRead(GPOUT_feulg_pin);

  
  // if connected update
  while (Phone_connected) {
  //Serial.print("Successful connection");   // print connected for testing purposes 
      FitnessUpdate();
    }
} 
void FitnessUpdate () {

  if (Updated_heart_rate != heart_rate){
   Serial.print("Heart Rate (bpm): ");
   Serial.println(heart_rate); //show values just for testing
   Heart_Rate_Characteristic.setValue(heart_rate); // not sure if it should be passed through as a pointer
   Heart_Rate_Characteristic.notify();
   Updated_heart_rate = heart_rate;
    }
    
  /*if (Updated_accelerometer != accelerometer){
   Serial.println("Steps: ", accelerometer); //show values just for testing
   pAccelerometer_Characteristic.writeValue(accelerometer);
   pAccelerometer_Characteristic.notifty();
   Updated_accelerometer = accelerometer; 
    }
    
  if (Updated_battery_charge != battery_charge){
   Serial.println("Battery Percent: ",battery_charge); //show values just for testing
   pBattery_Level_Characteristic.writeValue(battery_charge); 
   pBattery_Level_Characteristic.notifty();
   Updated_battery_charge = battery_charge; 
    }
  */
  // else nothing
}

  
  //delay(2000); // how often do we need to sample/flash data to mobile device 


/*
 * Blurred out code to test with the IC breadboarded:
 * Battery Charger (needs functions for LED notification)
 * Emergency contact Button (needs functions for if button is pressed)
*/
