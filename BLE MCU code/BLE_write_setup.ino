/*
    Adapted code based on Neil Kolban example for IDF
    Ported to Arduino ESP32 by Evandro Copercini
    Adapted and Modified by Kennedy Jackson - ECEN 404
*/

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>


//Add Device Name
#define Device_Name "FitTrak"

//Add BLE Services
#define Heart_Rate_Service_UUID "0x180D"

//Add BLE Characteristics
#define Heart_Rate_Char_UUID "0x2A37"
BLECharacteristic *pHeart_Rate_Characteristic;


bool Phone_connected = false;

//add code for callbacks for each BLEaction  
class MyServerCallbacks: public BLEServerCallbacks {
    
    void onConnect(BLEServer* pServer) {
      Phone_connected = true;
      //Prints in the serial monitor to inform on connection
      Serial.println("Connected");
    }
    
//add onDisconnect startadV again to make sure it works if disconnected
    void onDisconnect(BLEServer* pServer) {
      Phone_connected = false;
      
       //Prints in the serial monitor to inform on disconnection
      Serial.println("Disconnected");
    }
};

//add onRead, onNotify Callback
class MessageCallbacks : public BLECharacteristicCallbacks
{
       
    void onRead(BLECharacteristic *pCharacteristic)
    {
        int data = pCharacteristic->getValue();
        char txnum_char[8];
        itoa(data, txnum_char, 10); // 10 is deciamal base
        pCharacteristic->setValue(txnum_char);
    }
};

  

void setup() {
  Serial.begin(115200);

  // Setup BLE Server
    BLEDevice::init(Device_Name);
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Register message service that can receive messages and reply with a static message.
    BLEService *pService = pServer->createService(Heart_Rate_Service_UUID);
    pHeart_Rate_Characteristic = pService->createCharacteristic(Heart_Rate_Char_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    pHeart_Rate_Characteristic->setCallbacks(new MessageCallbacks());
    pHeart_Rate_Characteristic->addDescriptor(new BLE2902());
    pService->start();

    

    // Advertise services
    BLEAdvertising *pAdvertisement = pServer->getAdvertising();
    BLEAdvertisementData adv;
    adv.setName(Device_Name);
    adv.setCompleteServices(BLEUUID(Heart_Rate_Service_UUID));
    pAdvertisement->setAdvertisementData(adv);
    pAdvertisement->start();

    Serial.println("Ready Connection 1");
}

void loop() {
  
 delay(1000);
 
} 
