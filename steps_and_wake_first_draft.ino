#include <Wire.h>
#include <avr/wdt.h>

#include "SparkFun_BNO080_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_BNO080
BNO080 myIMU;

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("BNO080 Read Example");

  Wire.begin();

  myIMU.begin();

  Wire.setClock(400000); //Increase I2C data rate to 400kHz

  myIMU.enableGyro(100); //Send data update every 50ms

  Serial.println(F("Gyro enabled"));
  Serial.println(F("Output in form x, y, z, in radians per second"));

  myIMU.enableStepCounter(500); //Send data update every 500ms
  myIMU.enableTapDetector(500); //get tap detection every 500ms

  Serial.println(F("Step Counter enabled"));
  Serial.println(F("Step count since sketch started:"));

  delay(2000);
  wdt_enable(WDTO_8S);

}

void loop() {
  if (myIMU.dataAvailable() == true)
  {
    float x = myIMU.getGyroX();
    float y = myIMU.getGyroY();
    float z = myIMU.getGyroZ();

    Serial.print(F("Angular Velocity: "));
    Serial.print(x, 2);
    Serial.print(F(","));
    Serial.print(y, 2);
    Serial.print(F(","));
    Serial.print(z, 2);
    Serial.print(F(","));

    if (abs(x) > 5) {
      Serial.print(F("threshold exceeded"));
    }

    Serial.println();

    unsigned int steps = myIMU.getStepCount();
    
    Serial.print(F("Steps: "));
    Serial.print(steps);
    Serial.print(F(","));

    Serial.println();
    delay(50);
    wdt_reset();
  }

}
