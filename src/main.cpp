#include <Arduino.h>
#include <SPI.h>
#include <SimpleFOC.h>
#include "MagneticSensorSPI_G431b.h"



// Todo: Add unidirectional mode support?
MagneticSensorSPI_G431b sensor = MagneticSensorSPI_G431b();

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);


  Serial2.begin(115200);
  Serial2.println("Init Done");
  // init magnetic sensor   
  sensor.init();

}


void loop() {
  digitalWrite(LED_BUILTIN, millis()%2000>1000);

  Serial2.println((float)sensor.getSensorAngle());
  delay(10);
}