//demo files for James Ligon ECEN 403 project
//This code proved that the BNO080 is working

#include <Wire.h>
//#include <avr/wdt.h>

#include "SparkFun_BNO080_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_BNO080
BNO080 myIMU;

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("BNO080 Read Example");
  //plug into gpio 8, 9
  Wire.begin();

  myIMU.begin();
  Serial.println(F("after begin before setClock"));
  Wire.setClock(40000); //Increase I2C data rate to 40kHz

  myIMU.enableGyro(100); //Send data update every 100ms

  Serial.println(F("Gyro enabled"));
  
  myIMU.enableStepCounter(500); //Send data update every 500ms

  Serial.println(F("Step Counter enabled"));
  Serial.println(F("if wake signal, show Threshold exceeded"));
  //Serial.flush();
  Serial.println(F("Step count since sketch started:"));
  //Serial.flush();
  delay(100);
  //wdt_enable(WDTO_8S);

}

void loop() {
  //Serial.flush();
  //dataAvailable->getReadings->receivePacket is breaking in waitForI2C
  //I was looking on the wrong pin
  if (myIMU.dataAvailable() == true)
  {
    float x = myIMU.getGyroX();
    float y = myIMU.getGyroY();
    float z = myIMU.getGyroZ();

//    Serial.print(F("Angular Velocity, "));
//    Serial.print(x, 2);
//    Serial.print(F(","));
//    Serial.print(y, 2);
//    Serial.print(F(","));
//    Serial.print(z, 2);
//    Serial.print(F(", "));

    if (abs(x) > 5) {
      Serial.print(F("Threshold exceeded, "));
    } else {
      Serial.print(F(", "));
    }

    //Serial.println();

    unsigned int steps = myIMU.getStepCount();
    
    Serial.print(F("Steps, "));
    Serial.print(steps);
    Serial.print(F(","));

    Serial.println();
    delay(100);
    //wdt_reset();
  }

}
