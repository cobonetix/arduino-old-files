/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>

Servo Hservo;  // create servo object to control a servo
Servo Vservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int hidx = 0;    // variable to store the servo position
int vidx = 0;    // variable to store the servo position

unsigned char horLocs[] = { 90,120,180};
unsigned char horLocsx[] = { 0,30,60,90,120,150,180};
unsigned char verLocs[] = { 15, 35, 55};


void setup() {
  Serial.begin(9600);          //  setup serial
  Hservo.attach(3);  
  Vservo.attach(5); 
  Hservo.write(0);
  Vservo.write(30);
  
  Serial.println ("startup");
  pinMode(11, OUTPUT);
  digitalWrite(11,HIGH);
}

void loop() {
  int i;

  for (vidx = 0; vidx < sizeof (verLocs); vidx++)
  {  
     Hservo.write(horLocs[0]);              // tell servo to go to position in variable 'pos'
     Vservo.write(verLocs[vidx]);              // tell servo to go to position in variable 'pos'
     delay(200);                       // waits 15ms for the servo to reach the position
    
     for (hidx = 0; hidx < sizeof(horLocs) ; hidx++)
     {  
        Serial.println(horLocs[hidx]);
        
        Hservo.write(horLocs[hidx]);              // tell servo to go to position in variable 'pos'
        delay(1000);                       // waits 15ms for the servo to reach the position
     }
   }

/*
   while (1)
  {
    delay (100);
    digitalWrite(11,LOW);
    delay (100);
    
    while (analogRead(0) < 100)
      Serial.println(analogRead(0));
     
     Serial.println(analogRead(0));
   
      
    digitalWrite(11,HIGH);    
  }
*/


}
