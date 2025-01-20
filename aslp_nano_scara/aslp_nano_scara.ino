
 
#include "Wire.h"


#define USE_TIMER_1     true
#define USE_TIMER_2     false
#define USE_TIMER_3     false
#define USE_TIMER_4     false
#define USE_TIMER_5     false

#include "TimerInterrupt.h"
//#include <TimerInterrupt.hpp>         //https://github.com/khoih-prog/TimerInterrupt
//#include <ISR_Timer.hpp>              //https://github.com/khoih-prog/TimerInterrupt


// A pair of varibles to help parse serial commands (thanks Fergs)

int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[8];
char argv2[8];
char argv3[8];
char argv4[8];
 
// The arguments h to integers
long arg1;
long arg2;
long arg3;
long arg4;
 
//only pins 2 and 3 are interrupt

#define MR_SG  3
#define MR_EN  4
#define MR_DIR 5
#define MR_BR  6
#define MR_VR  7


#define ML_SG  21
#define ML_EN  38
#define ML_DIR 40
#define ML_BR  42
#define ML_VR  44


#define ST_DIR 6
#define ST_PL  7
#define ST_EN  5

#define LS_AN  A7
#define LS_DA  17

#define TOWER_STOPPED 0
#define TOWER_UP 1
#define TOWER_DOWN 2

int towerDirection = TOWER_STOPPED;
int towerLocation = 0;
int towerLimit;
int towerUpperLimit = 0xffff;
int towerLowerLimit = 0 ;
int inchesToTicks = 100;




unsigned long lastMilli = 0;
#define LOOPTIME                      1000     //Looptime in millisecond

int leftPos = 0;
int leftSteps = 0;

int rightPos = 0;
int rightSteps = 0;
  
#define DIR_STOPPED 0
#define DIR_FWD     1
#define DIR_BWD     2

char dir_left;
char dir_right;
int pos_left;
int pos_right;



void leftWheelStop(){
  digitalWrite(ML_EN,LOW);
  digitalWrite(ML_BR,HIGH);
  dir_left = DIR_STOPPED;
  Serial.println( "Left Wheel Stopped");
}

void rightWheelStop(){
  digitalWrite(MR_EN,LOW);
  digitalWrite(MR_BR,HIGH);
  dir_right = DIR_STOPPED;
  Serial.println( "Right Wheel Stopped");
}

void leftWheelMove( int dir, int speed,int steps){
      pos_left = 0;
      leftSteps = steps;
      digitalWrite(ML_EN,LOW);
      analogWrite(ML_VR, speed);
      delay(100);
      
      if (dir=='f'){
        digitalWrite(ML_DIR,LOW);
        dir_left = DIR_FWD;      
      }
      else {
         digitalWrite(ML_DIR,HIGH);
         dir_left = DIR_BWD;      
      }    
      delay(100);
      digitalWrite(ML_BR,LOW);
      digitalWrite(ML_EN,HIGH);
      Serial.print( "Left Wheel Started: ");
      Serial.println(speed);
}
      

void rightWheelMove( int dir, int speed, int steps){
      pos_right = 0;
      rightSteps = steps;
      digitalWrite(MR_EN,LOW);
      analogWrite(MR_VR, speed);
      
      delay(100);
      
      if (dir=='f'){
        digitalWrite(MR_DIR,HIGH);
        dir_right = DIR_FWD;      
      }
      else {
         digitalWrite(MR_DIR,LOW);
         dir_right = DIR_BWD;      
      }    
      delay(100);
      digitalWrite(MR_BR,LOW);
      digitalWrite(MR_EN,HIGH);
      Serial.print( "Right Wheel Started: ");
      Serial.println(speed);

}
void rightMotorInt() {
  switch(dir_right) {
    
  case DIR_STOPPED:
    break;
    
  case DIR_FWD:
    pos_right ++;
    break;
    
  case DIR_BWD:
    pos_right --;
    break;
  }    

}

void leftMotorInt() {
  switch(dir_left) {
    
  case DIR_STOPPED:
    break;
    
  case DIR_FWD:
    pos_left ++;
    break;
    
  case DIR_BWD:
    pos_left --;
    break;
  }   
  //erial.println(pos_left) ;
}

int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  char buff[80];

  arg1 = atol(argv1);
  arg2 = atol(argv2);
  arg3 = atol(argv3);
  arg4 = atol(argv4);

  Serial.println ("input: "  + String(cmd) + " " + String(argv1[0]) + " " +  String(arg2) + " " + String(arg3) );
       
  switch(cmd) {

  case 's':
     rightWheelStop();
     leftWheelStop();
     return;

       
  case 'l':   //left motor:  f/b speed steps
        
       Serial.println ("Left Motor "  + String(argv1[0]) + " " +  String(arg2) + " " + String(arg3) );
       
      if (   (argv1[0] == 'f') || (argv1[0] == 'b') )
        leftWheelMove(argv1[0], arg2, arg3);
      else
          Serial.println ("Invalid Value");         
        
      return;
      
  case 'r':  // right motor: f/b speed steps
        
       Serial.println ("Right Motor "  + String(argv1[0]) + " " +  String(arg2) + " " + String(arg3) );

       if ((argv1[0] == 'f') || (argv1[0] == 'b') )
         rightWheelMove(argv1[0], arg2, arg3);
       else
          Serial.println ("Invalid Value");         
        
      return;
       
  case 'b':  // both motors: f/b speed steps
        
       Serial.println ("Both Motors "  + String(argv1[0]) + " " +  String(arg2) + " " + String(arg3) );

       if ((argv1[0] == 'f') || (argv1[0] == 'b') ) {
        rightWheelMove(argv1[0], arg2, arg3);
        leftWheelMove(argv1[0], arg2, arg3);
      }
       else
          Serial.println ("Invalid Value");         
        
      return;
       
  case 'x':
     digitalWrite(MR_BR,HIGH);
     digitalWrite(ML_BR,HIGH);
     return;
     
   case 'u':
     digitalWrite(MR_BR,LOW);
     digitalWrite(ML_BR,LOW);
     return;
 
   case 'e':
     digitalWrite(ML_EN,HIGH);
     digitalWrite(MR_EN,HIGH);
      return;
     
   case 'd':
     digitalWrite(ML_EN,LOW);
     digitalWrite(MR_EN,LOW);
      return;
 
  case 'q':
      digitalWrite(ST_EN,LOW);
      
      if (argv1[0] == 'u')
        digitalWrite(ST_DIR,HIGH);
      else
         digitalWrite(ST_DIR,LOW);
      
      for (i = 0; i < arg2; i ++){
        digitalWrite(ST_PL,HIGH);
        delay(1);
        digitalWrite(ST_PL,LOW);
        delay(1);
      }
      digitalWrite(ST_EN,HIGH);
       return;

   case 't':

    switch (argv1[0]){

      case 'l':  // return current location
        Serial.print("OK: ");
        Serial.print(towerLocation);
        Serial.print("  HL: ");
        Serial.print(towerUpperLimit);
        Serial.print("  LL: ");
        Serial.println(towerLowerLimit);
        return;

      case 'h':  // stop moving
        towerDirection = TOWER_STOPPED;
        digitalWrite(ST_EN,HIGH);
      

        // stop interrupts somehow

        break;

      case 'u':  // start moving up

       towerDirection = TOWER_UP;
       towerLimit = towerUpperLimit;
       digitalWrite(ST_EN,LOW);
       digitalWrite(ST_DIR,HIGH);

       // start interrrupts 

       break;

      case 'd':  // start moving down

        towerDirection = TOWER_DOWN;
        towerLimit == towerLowerLimit;
        digitalWrite(ST_EN,LOW);
        digitalWrite(ST_DIR,LOW);

        // start interrupts

        break;

      case 'c':  // calibrate top/bottom

         if (argv2[0] == 't')
           towerUpperLimit = towerLocation;
         else
         if (argv2[0] == 'b')
           towerLowerLimit = towerLocation;
         
         break;

      case 'm':  // move to inch location

        if (argv2[0] == 'u'){
           towerLimit = min(towerUpperLimit, arg3*inchesToTicks);
           towerDirection = TOWER_UP;
           digitalWrite(ST_EN,LOW);
           digitalWrite(ST_DIR,HIGH);

           // start interrupts

        }
        else
        { if (argv2[0] == 'd')
           towerLimit = max(towerLowerLimit, arg3*inchesToTicks);
           towerDirection = TOWER_UP;
           digitalWrite(ST_EN,LOW);
           digitalWrite(ST_DIR,LOW);

           // start interrupts
           
        }
        break;

      case 'n':  // move to tick location
        if (argv2[0] == 'u'){
           towerLimit = min(towerUpperLimit, arg3);
           towerDirection = TOWER_UP;
           digitalWrite(ST_EN,LOW);
           digitalWrite(ST_DIR,HIGH);

           // start interrupts

        }
        else
        { if (argv2[0] == 'd')
           towerLimit = max(towerLowerLimit, arg3);
           towerDirection = TOWER_DOWN;
           digitalWrite(ST_EN,LOW);
           digitalWrite(ST_DIR,LOW);

           // start interrupts
           
        }
        break;

      default:
        Serial.println("ERR-bad command");
        break;
 
      }
      Serial.println("OK");
      break;

    case '>':
       digitalWrite(ST_EN,LOW);
       return;

    case '<':
      digitalWrite(ST_EN,HIGH);
      return;     
      
    case 'z':
      while (1)
      {
        Serial.print(analogRead(LS_AN));
        Serial.print(":");
        Serial.println(digitalRead(LS_DA));
        delay(500);
        }
          return;


  default:
      Serial.println ("Invalid Command");
      break;
  }
}

/* Clear the current command parameters */
void resetCommand() {
   
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  memset(argv3, 0, sizeof(argv3));
  memset(argv4, 0, sizeof(argv4));
  index = 0;
  arg = 0;

}

void TimerHandler()
{
  // Doing something here inside ISR

  Serial.println("tick");
  return;


  switch (towerDirection)  {

    case TOWER_STOPPED:
      return;

    case TOWER_UP:
      if (towerLocation > towerLimit)
      {
         towerDirection = TOWER_STOPPED;
         digitalWrite(ST_EN,LOW);
         return;
      }
      towerLocation ++;
      digitalWrite(ST_PL,HIGH);
      delay(1);
      digitalWrite(ST_PL,LOW);
      return;

    case TOWER_DOWN:
      if (towerLocation < towerLimit)
      {
         towerDirection = TOWER_STOPPED;
         digitalWrite(ST_EN,LOW);
         return;
      }
      towerLocation --;
      digitalWrite(ST_PL,HIGH);
      delay(1);
      digitalWrite(ST_PL,LOW);
      return;
  }
}

void setup() {

  // put your setup code here, to run once:
  Serial.begin(57600);
  
  //wheel left - Setup pins
  
#ifdef wheel
  pinMode(ML_EN, OUTPUT);    //stop/start - EL 
  digitalWrite(ML_EN,LOW);  xx
  pinMode(ML_SG, INPUT);     //plus       - Signal  
  pinMode(ML_DIR, OUTPUT);   //direction  - ZF 
  pinMode(ML_VR, OUTPUT);    //pwm output 
  pinMode(ML_BR, OUTPUT);    //brake output 
  digitalWrite(ML_BR,HIGH);
   
  //Hall sensor detection - Count steps
  
  attachInterrupt(digitalPinToInterrupt(ML_SG), leftMotorInt, CHANGE);

  //wheel right - Setup pins
  pinMode(MR_EN, OUTPUT);    //stop/start - EL 
  digitalWrite(MR_EN,LOW);
  
  pinMode(MR_SG, INPUT);     //plus       - Signal  
  pinMode(MR_DIR, OUTPUT);   //direction  - ZF 
  pinMode(MR_VR, OUTPUT);    //pwm output 
  pinMode(MR_BR, OUTPUT);    //brake output 
  digitalWrite(MR_BR,HIGH);


  pinMode(ST_EN,OUTPUT);
  pinMode(ST_DIR,OUTPUT);
  pinMode(ST_PL,OUTPUT);

  digitalWrite(ST_DIR,LOW);
  digitalWrite(ST_PL,LOW);
  digitalWrite(ST_EN,HIGH);  // backward to docs
   
  pinMode(LS_DA,INPUT);
#endif
  //Hall sensor detection - Count steps
  
  //attachInterrupt(digitalPinToInterrupt(MR_SG), rightMotorInt, CHANGE);
 
  leftPos = 0;
  rightPos = 0;
  

#ifdef xx
  // Init timer ITimer1
   ITimer1.init();

#define TIMER_INTERVAL_MS 1000

   // Interval in unsigned long millisecs
  if (ITimer1.attachInterruptInterval(TIMER_INTERVAL_MS, TimerHandler))
    Serial.println("Starting  ITimer OK, millis() = " + String(millis()));
  else
    Serial.println("Can't set ITimer. Select another freq. or timer");  
#endif

  resetCommand();
    
  delay(100);
  Serial.println("setup done");

}

void loop() {

    char *p;
     
  // handle any pending characters
  
  while (Serial.available() > 0) {
    
    // Read the next character
    chr = Serial.read();
 
    // Terminate a command with a CR
 
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      else if (arg == 3) argv3[index] = NULL;
      else if (arg == 4) argv4[index] = NULL;
      
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      } else if (arg == 2)  {
        argv2[index] = NULL;
        arg = 3;
        index = 0;
      } else if (arg == 3)  {
        argv3[index] = NULL;
        arg = 4;
        index = 0;
      } else if (arg == 4)  {
        argv4[index] = NULL;
        index = 0;
      }
     continue;
    }
    else 
    {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        // Subsequent arguments can be more than one character
        argv2[index] = chr;
        index++;
      }
      else if (arg == 3) {
        // Subsequent arguments can be more than one character
        argv3[index] = chr;
        index++;
      }
      else if (arg == 4) {
        // Subsequent arguments can be more than one character
        argv4[index] = chr;
        index++;
      }
     }
  }

  /*
  if((millis()-lastMilli) >= LOOPTIME)   
  {                                                                           // enter timed loop
    lastMilli = millis();
   
    if (   dir_right != DIR_STOPPED) {
      Serial.print("R-");
      Serial.println(pos_right);
    }

     if ( dir_left != DIR_STOPPED) {
      Serial.print("L-");
      Serial.println(pos_left);
    }
 
    
    if ((abs(pos_right) -rightSteps) < 5){
      rightWheelStop();
    }

     if ((abs(pos_left) - leftSteps) < 5){
      leftWheelStop();
    }
  */

}
