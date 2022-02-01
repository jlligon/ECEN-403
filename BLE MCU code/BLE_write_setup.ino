/*
    Adapted code based on Neil Kolban example for IDF
    Ported to Arduino ESP32 by Evandro Copercini
    Adapted and Modified by Kennedy Jackson - ECEN 404
*/

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h> //Library to use BLE as server ?? see if needed
#include <BLE2902.h>

// See the following for generating UUIDs: https://www.uuidgenerator.net/
// FitTrak BLE Code
#define Device_name "FitTrak"

//Add BLE Services
#define Heart_Rate_Service_UUID "0x180D"
//#define Accelerometer_Service_UUID "21688e72-1add-4a67-a83d-075927ddb2cd" //self-generated UUID
//#define Battery_Level_Service_UUID "0x180F"

//Add BLE Characteristics
#define Heart_Rate_Char_UUID "0x2A37"
BLECharacteristic *pHeart_Rate_Characteristic;
                      
/*#define Accelerometer_Char_UUID "0x27BA" //(this is step per minute)
BLECharacteristic *pAccelerometer_Characteristic;
                       
//#define Battery_Level_Char_UUID "0x2A19"
BLECharacteristic *pBattery_Level_Characteristic 
                      (Battery_Level_Char_UUID, 
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_NOTIFY);
                      
*/
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

//boolCharacteristics: initalize false to ensure connection
  bool Phone_connected = false;

//add code for callbacks for each BLEaction  
class MyCallbacks: public BLEServerCallbacks {
    
    void onConnect(BLEServer* pServer) {
      Phone_connected = true;
      Serial.println("Connected");//Prints in the serial monitor to inform on connection
    }
    
      //add onDisconnect startadV again to make sure it works if disconnected
    void onDisconnect(BLEServer* pServer) {
      Phone_connected = false;
      Serial.println("Disconnected");//Prints in the serial monitor to inform on disconnection
    }
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
class MyMessageCallbacks : public BLECharacteristicCallbacks
{

   void onRead(BLECharacteristic *pCharacteristic)
    {
        pCharacteristic->setValue(heart_rate); // see if that works
    }
};


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


  BLEDevice::init(Device_name);

  //BLE Server
  BLEServer *pServer = BLEDevice::createServer();  
  pServer->setCallbacks(new MyCallbacks());

  //BLE Services
  BLEService *pHeart = pServer->createService(Heart_Rate_Service_UUID);
  pHeart_Rate_Characteristic = pHeart->createCharacteristic(Heart_Rate_Char_UUID, 
                                                            BLECharacteristic::PROPERTY_READ | 
                                                            BLECharacteristic::PROPERTY_NOTIFY);
  pHeart_Rate_Characteristic->setCallbacks(new MyMessageCallbacks());
  pHeart_Rate_Characteristic->addDescriptor(new BLE2902());
  pHeart->start();

  /*BLEService *pAccel = pServer->createService(Accelerometer_Service_UUID);
  
  pAccelerometer_Service_Characteristic = pAccel->createCharacteristic(Accelerometer_Char_UUID, 
                                                                       BLECharacteristic::PROPERTY_READ | 
                                                                       BLECharacteristic::PROPERTY_NOTIFY);
  pAccelerometer_Service_Characteristic->setCallbacks(new MyMessageCallbacks());
  pAccelerometer_Service_Characteristic->addDescriptor(new BLE2902());
  pAccel->start();
  
  BLEService *pBattery = pServer->createService(Battery_Service_UUID);
  pBattery_Level_Characteristic = pBattery->createCharacteristic(Battery_Service_UUID
                                                BLECharacteristic::PROPERTY_READ | 
                                                BLECharacteristic::PROPERTY_NOTIFY);
  pBattery_Level_Characteristic->setCallbacks(new MyMessageCallbacks());
  pBattery_Level_Characteristic->addDescriptor(new BLE2902());
  pBattery->start();
*/
  BLEAdvertising *pAdvertisement = pServer->getAdvertising();
  BLEAdvertisementData advdata;
  advdata.setName(Device_name);
  advdata.setCompleteServices(BLEUUID(Heart_Rate_Char_UUID));
//  advdata.setCompleteServices(BLEUUID(Accelerometer_Service_UUID));
//  advdata.setCompleteServices(BLEUUID(Battery_Service_UUID));
  pAdvertisement->setAdvertisementData(advdata);
  pAdvertisement->start();

  Serial.println("Ready for Connection...(working)");



}

void loop() {
  // main code here  
  // this is the extrnal view from laptop script (non-BLE)
  
  // reading inputs
   heart_rate = analogRead(heart_rate_pin);   
  // accelerometer = analogRead(accelerometer_pin);
  // battery_charge  = digitalRead(GPOUT_feulg_pin);
if (Phone_connected){
  if (Updated_heart_rate != heart_rate){
   Serial.print("Heart Rate (bpm): ");
   Serial.println(heart_rate); //show values just for testing
   // needs to be done through the callbacks, try without this first
   // pHeart_Rate_Characteristic->setCallbacks(new MyMessageCallbacks());

   Updated_heart_rate = heart_rate;
    }
    
  /*if (Updated_accelerometer != accelerometer){
   Serial.print("Steps: ");
   Serial.println(accelerometer)); //show values just for testing
   Accelerometer_Characteristic.setValue(accelerometer);
   Accelerometer_Characteristic.notifty();
   Updated_accelerometer = accelerometer; 
    }
    
  if (Updated_battery_charge != battery_charge){
   Serial.println("Battery Percent: ",battery_charge); //show values just for testing
   pBattery_Level_Characteristic.setValue(battery_charge); 
   pBattery_Level_Characteristic.notifty();
   Updated_battery_charge = battery_charge; 
    }
  */
  // else nothing
}
}

  
  //delay(2000); // how often do we need to sample/flash data to mobile device 


/*
 * Blurred out code to test with the IC breadboarded:
 * Battery Charger (needs functions for LED notification)
 * Emergency contact Button (needs functions for if button is pressed)
*/
