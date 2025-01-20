/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>
#include "SoftwareSerial.h"

Servo UpDown;   
Servo Rotate;   
Servo Elbow;

// twelve servo objects can be created on most boards


int rtidx = 0;    // variable to store the servo position

unsigned int seq[][5] = {   // bend, rotate, elbow, vacuum, delay
  
   { 115,  5, 120,1,1000},    // starting position
   { 40,   5, 120,1,400},     // down and extend
   { 45,   5, 180,1,500},     // pull back
   { 60,   5, 180,1,500},     // lift up
   { 60,   5, 120,1,500},     // reach out for label
   { 26,   5, 120,0,800},     // grab the label
   { 115,  5, 120,0,1000},     // go vertical
   
   { 115, 87, 120,0,1000},    // rotate to start of AA process
   { 65,  87, 180,0,2000},    // go down to AA device
   { 65,  40, 180,0,200},    // slide across device
   { 115, 40, 120,1,500},    // lift up
   { 115,  5, 120,1,1000},    // return to starting position
    
//   { 115,175,120,0,4000},   // rotate to EE spot
//   { 115,175,120,1,2000},   // release label
   {0xff,0,0,0,0} };

   

void setup() {
  Serial.begin(19200);          //  setup serial
  UpDown.attach(5);  
  Rotate.attach(6); 
  Elbow.attach(9);
  
  UpDown.write(seq[0][0]);
  Rotate.write(seq[1][0]);
  Elbow.write(seq[2][0]);

  Serial.println ("startup");
  pinMode(8, OUTPUT);          // pump relay
  digitalWrite(8,HIGH);
}

void loop() {
  int i;
   while (!Serial.available());
   Serial.read();

  for (rtidx = 0; seq[rtidx] [0] != 0xff; rtidx++)
  {  
/*     Serial.print("\n seq ");
     Serial.print(rtidx);
     Serial.print(seq[rtidx][0]);
     Serial.print(seq[rtidx][1]);
     Serial.print(seq[rtidx][2]);
     Serial.println(seq[rtidx][3]);
  */   
     
     UpDown.write(   seq[rtidx][0] );              // tell servo to go to position in variable 'pos'
     Rotate.write(   seq[rtidx][1] );              // tell servo to go to position in variable 'pos'
     Elbow.write(   seq[rtidx][2] );              // tell servo to go to position in variable 'pos'
     digitalWrite(8, seq[rtidx][3] );

     delay ( seq[rtidx][4]);  
   }
   Serial.println ("loop");
   delay(1000);

}
