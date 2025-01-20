#include "Adafruit_VL53L0X.h"
#include "wire.h"


#define SHUTDOWN1   7
#define SHUTDOWN2   8
#define SHUTDOWN3   9

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup() {
  Serial.begin(115200);

  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }
  Serial.println("Adafruit VL53L0X start");

  pinMode(SHUTDOWN1,OUTPUT);
  digitalWrite(SHUTDOWN1,LOW);
  pinMode(SHUTDOWN2,OUTPUT);
  digitalWrite(SHUTDOWN2,HIGH);
 pinMode(SHUTDOWN3,OUTPUT);
  digitalWrite(SHUTDOWN3,HIGH);
  
   
  delay(100);
  
  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  digitalWrite(SHUTDOWN1,LOW);
  digitalWrite(SHUTDOWN2,LOW);
  digitalWrite(SHUTDOWN3,LOW);
  // power 
  Serial.println(F("VL53L0X API Simple Ranging example\n\n")); 
}

char p = 0;

void loop() {
  VL53L0X_RangingMeasurementData_t measure;
    

  switch(p) {
    
  case 0:
    digitalWrite(SHUTDOWN1,HIGH);
    digitalWrite(SHUTDOWN2,LOW);
    digitalWrite(SHUTDOWN3,LOW);
    p=1;
    break;

  case 1:
    digitalWrite(SHUTDOWN1,LOW);
    digitalWrite(SHUTDOWN2,HIGH);
    digitalWrite(SHUTDOWN3,LOW);
    p=2;
    break;

  case 2:
    digitalWrite(SHUTDOWN1,LOW);
    digitalWrite(SHUTDOWN2,LOW);
    digitalWrite(SHUTDOWN3,HIGH);
    p=0;
    break;
  }

   delay(20);
   if (!lox.begin()) {
    Serial.println(F("Failed to boot1 VL53L0X"));
    while(1);
   }

 
  Serial.print("Reading a measurement... ");
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
  } else {
    Serial.println(" out of range ");
  }
    
  delay(20);
}
