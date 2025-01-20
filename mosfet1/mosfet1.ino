#include <ArduinoHardware.h>
#include <ArduinoTcpHardware.h>
#include <ros.h>

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
     pinMode(3,OUTPUT);
     digitalWrite(3,HIGH);

}

void loop() {
  // put your main code here, to run repeatedly:

  while (1)
  {

      Serial.print('x');
      
      digitalWrite(3,HIGH);

      delay (2000);

     digitalWrite(3,LOW);
     delay (2000);
      
  }}
