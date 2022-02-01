//These files needed to send and recieve BLE information
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>


//Add BLE Services
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"

//Add BLE Characteristics
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
BLECharacteristic *pCharacteristic;

//Add Device Name
#define Device_Name "ESP32"

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
    void onWrite(BLECharacteristic *pCharacteristic)
    {
        std::string data = pCharacteristic->getValue();
        Serial.println(data.c_str());
        ;
    }

    void onRead(BLECharacteristic *pCharacteristic)
    {
        pCharacteristic->setValue("Test");
    }
};

  

void setup() {
  Serial.begin(115200);

  // Setup BLE Server
    BLEDevice::init(Device_Name);
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Register message service that can receive messages and reply with a static message.
    BLEService *pService = pServer->createService(SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE);
    pCharacteristic->setCallbacks(new MessageCallbacks());
    pCharacteristic->addDescriptor(new BLE2902());
    pService->start();

    

    // Advertise services
    BLEAdvertising *pAdvertisement = pServer->getAdvertising();
    BLEAdvertisementData adv;
    adv.setName(Device_Name);
    adv.setCompleteServices(BLEUUID(SERVICE_UUID));
    pAdvertisement->setAdvertisementData(adv);
    pAdvertisement->start();

    Serial.println("Ready Connection 1");
}

void loop() {
  
 delay(1000);
 
} 
