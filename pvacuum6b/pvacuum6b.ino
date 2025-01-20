 
// VACUUM/TOWER CONTROL
//
////////////////////////////////////////////////////

#define USE_TIMER_1 true
#define USE_TIMER_2 false
#define USE_TIMER_3 false
#define USE_TIMER_4 false
#define USE_TIMER_5 false

#include "TimerInterrupt.h"

// GPIO ASSIGNMENTS

#define TANK_SENSOR    A0

#define BOT_SWITCH_PIN     2        
#define TOP_SWITCH_PIN  3

#define ST_DIR     5
#define ST_EN      6
#define ST_PL      7

#define LIFT_P2    10
#define LIFT_P1    11
#define LIFT_MAIN1 12
#define PUMP_ON    13

// TOWER STATES

#define TOWER_STOPPED        0
#define TOWER_STOPPED_SWITCH 1
#define TOWER_UP             2
#define TOWER_DOWN           3

#define TOWER_ACC    0
#define TOWER_STEADY 1
#define TOWER_DEC    2


int towerMode = TOWER_ACC;
long  towerSteadyTrigger = 0;
long towerDecTrigger = 0;
long towerTickCount = 0;
char towerIntervalCount = 0;
char towerIntervalTable[] = { 4, 4, 3, 3, 2, 2, 1, 1 };
char towerIntervalTableIndex = 0;
char towerBuckets = sizeof(towerIntervalTable) / sizeof(towerIntervalTable[0]);
long towerIntervalChangeTrigger = 0;
int towerIntervalChangeIncrement = 0;


char towerDirection = TOWER_STOPPED;
long towerLocation = 0;
long towerLimit;
long towerUpperLimit = 99999;
long towerLowerLimit = 0;
int inchesToTicks = 43;
int towerSpeed = 0;
int towerCurrentSpeed = 0;


#define CALIBRATE_IDLE            0
#define CALIBRATE_GOING_UP        1  
#define CALIBRATE_GOING_DOWN_BIG  2 
#define CALIBRATE_GOING_DOWN_FINE 3

char switchMask;
#define TOP_SWITCH_TRIGGER 1
#define BOT_SWITCH_TRIGGER 2

char calibrateState;
long calibrateGoUp = 90000;
long calibrateGoDown = 90000;

char switchTestLast,switchTest;

// tanks  100psi = 511

// PUMP Control variables

char pumpAuto = 0;
char pumpOn = 0;
int pumpOnThreshold = 250;   // above this turn pump on  ~50 psi
int pumpOffThreshold = 512;  // below this turn pump off ~100 psi

int pumpSampleInterval = 1000;
long pumpSampleTime = 0;

#define PUMP_LIMIT 470 /* about 90 pounds */

////////////////////////////////////////////////////

// command line values

#define SENSOR 's'
#define PUMP 'p'
#define RESET 'z'
#define PUMP_MON 'm'

#define LIFT 'l'
#define QUICK 'q'
#define TOWER 't'
#define TOWER2 'u'



//////////////

// command parsing variables

char debugPrint = 0;

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
char *px;
unsigned long arg1;
long arg2;
long arg3;
long arg4;

///////////////////////////////////////////////////////////


void startMotor(char cmd, long ticks){
  char gpiodir;
  
   if (cmd == 'u')
  { 
            towerLimit = towerLocation + ticks;
            towerDirection = TOWER_UP;
            switchMask = TOP_SWITCH_TRIGGER;
            gpiodir = HIGH;
  }
  else if (cmd == 'd')
  {
            towerLimit = towerLocation - ticks;
            towerDirection = TOWER_DOWN;
            switchMask = BOT_SWITCH_TRIGGER;
             gpiodir = LOW; 
  }
  else
    return;

  towerMode = TOWER_ACC;
  towerTickCount = 0;
  towerIntervalTableIndex = 0;
  towerIntervalCount = towerIntervalTable[towerIntervalTableIndex];
           
  towerSteadyTrigger = min(arg3 / 8, 200);
  towerDecTrigger = arg3 - towerSteadyTrigger;
  towerIntervalChangeIncrement = towerSteadyTrigger / towerBuckets;
  towerIntervalChangeTrigger = towerIntervalChangeIncrement;
  
  digitalWrite(ST_EN, LOW);
  digitalWrite(ST_DIR, gpiodir);
}
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int tank;
  int t;

  arg1 = atol(argv1);
  arg2 = atol(argv2);
  arg3 = atol(argv3);
  arg4 = atol(argv4);

  switch (cmd) {

    case RESET:
      setup();
      Serial.println("OK");
      return;

    case SENSOR:
        tank = analogRead(TANK_SENSOR);
        Serial.print(tank);
        Serial.print(":");
        Serial.print((tank * 10) / 51);
        Serial.println("OK");
        return;

    case PUMP:
      if (argv1[0] == 'o') {
        digitalWrite(PUMP_ON, LOW);
      } else {
        digitalWrite(PUMP_ON, HIGH);
      }

      Serial.println("OK");
      return;

    case PUMP_MON:

      Serial.println("OK");

      if (argv1[0] == 'o') {
        pumpAuto = 1;
        pumpSampleTime = 0;
        pumpOn = 0;

        tank = analogRead(TANK_SENSOR);
        if (tank < pumpOffThreshold) {
          digitalWrite(PUMP_ON, LOW);
          pumpOn = 1;
        }
      } else {
        pumpAuto = 0;
        pumpOn = 0;
        digitalWrite(PUMP_ON, HIGH);
      }
      Serial.println("OK");
      return;



    case TOWER2:
      digitalWrite(ST_EN, LOW);

      if (argv1[0] == 'u')
        digitalWrite(ST_DIR, HIGH);
      else
        digitalWrite(ST_DIR, LOW);

      for (i = 0; i < arg2; i++) {
        digitalWrite(ST_PL, HIGH);
        delayMicroseconds(500);
        digitalWrite(ST_PL, LOW);
        delayMicroseconds(500);
      }
      digitalWrite(ST_EN, HIGH);
      Serial.println("OK");
      return;

  case LIFT:
      if (argv1[0] == 'u')   // up
      {
         digitalWrite(LIFT_MAIN1,LOW );
        digitalWrite(LIFT_P1,LOW);
         digitalWrite(LIFT_P2,LOW);   
      }
      else if (argv1[0] == 'd')   // down
      {
         digitalWrite(LIFT_MAIN1,LOW );
         digitalWrite(LIFT_P1,HIGH);
         digitalWrite(LIFT_P2,HIGH);             
      }
      else   //stop
      {
         digitalWrite(LIFT_MAIN1,HIGH );
      }
      Serial.println("OK");
      break;
      
      case TOWER:
      calibrateState = CALIBRATE_IDLE;

      switch (argv1[0]) {

        case 'l':  // return current location
          Serial.print("OK: TL ");
          Serial.print(towerLocation);
          Serial.print("  HL: ");
          Serial.print(towerUpperLimit);
          Serial.print("  LL: ");
          Serial.print(towerLowerLimit);
          Serial.print("  SP: ");
          Serial.print(towerCurrentSpeed);
          Serial.print("  ITT: ");
          Serial.println(inchesToTicks);
          return;

        case 'h':  // stop moving
          towerDirection = TOWER_STOPPED;
          digitalWrite(ST_EN, HIGH);
          break;

        case 'u':  // start moving up
          towerDirection = TOWER_UP;
          towerLimit = towerUpperLimit;
          digitalWrite(ST_EN, LOW);
          digitalWrite(ST_DIR, HIGH);
          break;

        case 'd':  // start moving down
          towerDirection = TOWER_DOWN;
          towerLimit = towerLowerLimit;
          digitalWrite(ST_EN, LOW);
          digitalWrite(ST_DIR, LOW);
          break;

        case 's':
          towerCurrentSpeed = arg2;
          towerSpeed = arg2;
          break;

       case 'i':
          inchesToTicks = arg2;
          break;
        
        case 'c':  // calibrate top/bottom
          towerLocation = 150000;
          towerUpperLimit = 300000;
          calibrateState = CALIBRATE_GOING_DOWN_BIG;
          switchMask = BOT_SWITCH_TRIGGER;
          startMotor('d',calibrateGoDown);
          Serial.println("cal-start");
          break;

        case 'm':  // move to inch location
          t = arg2 * inchesToTicks;

          Serial.println(t);

          if (t < towerUpperLimit) {
            towerLimit = t;

            if (t < towerLocation) {
              towerDirection = TOWER_DOWN;
              digitalWrite(ST_EN, LOW);
              digitalWrite(ST_DIR, LOW);
            } else {
              towerDirection = TOWER_UP;
              digitalWrite(ST_EN, LOW);
              digitalWrite(ST_DIR, HIGH);
            }
          } else {
            Serial.println("ERR: Invalid location");
            return;
          }
          break;

        case 'n':  // move ticks up or down
 
          startMotor(argv2[0],arg3);
          break;

        case '>':
          digitalWrite(ST_EN, LOW);
          return;

        case '<':
          digitalWrite(ST_EN, HIGH);
          return;

        case ']':
          digitalWrite(ST_DIR, LOW);
          return;

        case '[':
          digitalWrite(ST_DIR, HIGH);
          return;

        case 'x':
          switchTestLast  =0;
          Serial.println(arg2);

          if (arg2 == 1)
            switchTest = 1;
          else
            switchTest = 0;
          break;

       case 'y':
         if (argv2 != 0)
           towerUpperLimit = arg2 + arg3;
         else
           towerUpperLimit = 84270;  
          break;

         
       case 'z':
         if (argv2 != 0)
           towerUpperLimit = arg2 + arg3;
           
         switchMask = BOT_SWITCH_TRIGGER;
         towerLocation = 100000;
         startMotor('d',90000);
         break;

         
       default:
          Serial.println("ERR-bad command");
          break;
      }
      Serial.println("OK");
      break;

    default:
      break;
  }
}

/* Clear the currenLOWt command parameters */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  memset(argv3, 0, sizeof(argv3));
  memset(argv4, 0, sizeof(argv4));
  arg1 = 0;
  arg2 = 0;
  arg3 = 0;
  arg4 = 0;
  arg = 0;
  index = 0;
}

int towerCalcInterval() {
  // see if we should do anything

  if (towerIntervalCount != 1) {  // we do nothing, exit with signal that says skip further processing
    towerIntervalCount--;
    return 0;
  }
  // we will generate a tick and so calculate the next interval

  switch (towerMode) {

    case TOWER_ACC:
      //Serial.println("a");
      if (towerTickCount >= towerSteadyTrigger) {
        // time to transition to steady state motion using the shortest interval

        towerTickCount++;
        towerIntervalCount = towerIntervalTable[towerBuckets - 1];
        //Serial.println("acc stddy");
        towerMode = TOWER_STEADY;
        return 1;
      } else {
        // not at steady time, calc interval time

        if (towerTickCount >= towerIntervalChangeTrigger) {
          // time to move to next interval, which will be less than the current one

          if (towerIntervalTableIndex != towerBuckets - 1)
            towerIntervalTableIndex++;
          towerIntervalCount = towerIntervalTable[towerIntervalTableIndex];
          towerIntervalChangeTrigger = towerTickCount + towerIntervalChangeIncrement;
          towerTickCount++;
          // Serial.print(towerIntervalCount);
          // Serial.print(" acc inc ");
          // Serial.println(towerIntervalCount);
          return 1;
        } else {
          // use current interval
          towerIntervalCount = towerIntervalTable[towerIntervalTableIndex];
          towerTickCount++;
          return 1;
        }
      }
      break;

    case TOWER_DEC:
      //Serial.println("d");

      // see if time to slow down even more

      if (towerTickCount >= towerIntervalChangeTrigger) {
        // time to move to next interval, which will be more than the current one

        if (towerIntervalTableIndex != 0)  // make sure we don't go below max interval
          towerIntervalTableIndex--;

        towerTickCount++;
        towerIntervalCount = towerIntervalTable[towerIntervalTableIndex];
        towerIntervalChangeTrigger = towerTickCount + towerIntervalChangeIncrement;
        //Serial.print(towerIntervalCount);
        //Serial.print(" dec dec ");
        //Serial.println(towerIntervalCount);
        return 1;
      } else {
        // use current interval
        towerIntervalCount = towerIntervalTable[towerIntervalTableIndex];
        towerTickCount++;
        return 1;
      }
      break;

    case TOWER_STEADY:
      //Serial.println("s");
      if (towerTickCount < towerDecTrigger) {
        //
        // not yet time to dec, return steady inteval

        towerIntervalCount = towerIntervalTable[towerBuckets - 1];
        towerTickCount++;
        //Serial.print(towerIntervalCount);
        //Serial.print(" stddy ");
        //Serial.println(towerTickCount);
        return 1;
      }

      // time to start dec
      towerMode = TOWER_DEC;
      towerIntervalTableIndex = towerBuckets - 1;
      towerIntervalCount = towerIntervalTable[towerIntervalTableIndex];
      towerIntervalChangeTrigger = towerTickCount + towerIntervalChangeIncrement;
      towerTickCount++;
      //Serial.print(towerIntervalCount);
      //Serial.print(" stddy dec ");
      // Serial.println(towerTickCount);
      return 1;
  }
}
void TimerHandler() {
  
  if ( ((switchMask == TOP_SWITCH_TRIGGER) && (!digitalRead(TOP_SWITCH_PIN))) or
       ((switchMask == BOT_SWITCH_TRIGGER) && (!digitalRead(BOT_SWITCH_PIN))) ) 
  {
     //
        if (switchMask == BOT_SWITCH_TRIGGER)
          towerLocation = 0;
        
        if (switchMask == TOP_SWITCH_TRIGGER)
          towerUpperLimit = towerLocation ;
        
        switchMask = 0;
        towerDirection = TOWER_STOPPED_SWITCH;
        digitalWrite(ST_EN, LOW);
        Serial.print("Switch detection: " );
        Serial.println(digitalRead(TOP_SWITCH_PIN) + (digitalRead(BOT_SWITCH_PIN) << 1),HEX);
        return;
  }
  

  if (towerSpeed != 0) {
    towerSpeed--;
    // return;
  }

  towerSpeed = towerCurrentSpeed;

  switch (towerDirection) {

    case TOWER_STOPPED:
      return;

    case TOWER_UP:
      if (towerLocation >= towerLimit) {
        towerDirection = TOWER_STOPPED;

        digitalWrite(ST_EN, LOW);
        Serial.println("stoppedu");
        return;
      }

      if (towerCalcInterval()) {
        towerLocation++;
        digitalWrite(ST_PL, HIGH);
        delayMicroseconds(50);
        digitalWrite(ST_PL, LOW);

        //Serial.print(towerLocation);
        //Serial.println("-mu-");
      }
      return;

    case TOWER_DOWN:
      if (towerLocation <= towerLimit) {
        towerDirection = TOWER_STOPPED;
        digitalWrite(ST_EN, LOW);
        Serial.println("stoppedd");
        return;
      }

      if (towerCalcInterval()) {
        towerLocation--;
        digitalWrite(ST_PL, HIGH);
        delayMicroseconds(50);
        digitalWrite(ST_PL, LOW);
        //Serial.print(towerLocation);
        //Serial.println("-md-");
      }
      return;
  }
}


void setup() {


  Serial.begin(9600);
  pumpAuto = 0;
  pumpOn = 0;
  calibrateState = CALIBRATE_IDLE;

  //   relays are off except sensor power (for now)

  pinMode(TOP_SWITCH_PIN, INPUT_PULLUP);     
  pinMode(BOT_SWITCH_PIN, INPUT_PULLUP);  
 
  pinMode(PUMP_ON, OUTPUT);  // pump off
  digitalWrite(PUMP_ON, HIGH);

  pinMode(LIFT_MAIN1, OUTPUT);
  digitalWrite(LIFT_MAIN1, HIGH);

  pinMode(LIFT_P1, OUTPUT);
  digitalWrite(LIFT_P1, HIGH);

  pinMode(LIFT_P2, OUTPUT);
  digitalWrite(LIFT_P2, HIGH);

  pinMode(ST_EN, OUTPUT);
  pinMode(ST_DIR, OUTPUT);
  pinMode(ST_PL, OUTPUT);

  digitalWrite(ST_DIR, LOW);
  digitalWrite(ST_PL, LOW);
  digitalWrite(ST_EN, HIGH);  // backward to doc

  switchTest= 0;
  towerLocation = 99999;  // don't really know where it is

  // Init timer ITimer1
  ITimer1.init();

#define TIMER_INTERVAL_MS 1

  // Interval in unsigned long millisecs
  if (ITimer1.attachInterruptInterval(TIMER_INTERVAL_MS, TimerHandler))
    Serial.println("Starting  ITimer OK, millis() = " + String(millis()));
  else
    Serial.println("Can't set ITimer. Select another freq. or timer");

  // change the timer interval to 512us rather than 1ms

  OCR1A = OCR1A / 4;
  Serial.println("VSetup Done1");
}


/////////////////////
///
// Main execution loop

void loop() {

  int tank, temp;

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
      else if (arg == 1) {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      } else if (arg == 2) {
        argv2[index] = NULL;
        arg = 3;
        index = 0;
      } else if (arg == 3) {
        argv3[index] = NULL;
        arg = 4;
        index = 0;
      } else if (arg == 4) {
        argv4[index] = NULL;
        arg = 5;
        index = 0;
      }
      continue;
    } else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      } else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      } else if (arg == 2) {
        argv2[index] = chr;
        index++;
      } else if (arg == 3) {
        argv3[index] = chr;
        index++;
      } else if (arg == 4) {
        argv4[index] = chr;
        index++;
      }
    }
  }

  // handle state machines
  
  // see if doing calibrate. 

  
  switch (calibrateState)
  {
    case CALIBRATE_IDLE:
      break;

    case CALIBRATE_GOING_DOWN_BIG:
      // we went down a whole bunch. Make sure we did not hit the bottom switch

      switch (towerDirection)
      {
      case TOWER_STOPPED_SWITCH:
      
        // we hit the bottom switch, 

        towerLocation = 0;
        calibrateState = CALIBRATE_GOING_UP;
        startMotor('u',calibrateGoUp);
        switchMask = TOP_SWITCH_TRIGGER;
        Serial.println("cal-bot-sw-hit");
        break;
        
      case TOWER_STOPPED:
        // ran out of ticks, go down a bit more
        // we moved a whole bunch down but did not hit the switch. go down some more

        calibrateState = CALIBRATE_GOING_DOWN_BIG;
        startMotor('d',calibrateGoDown);
        switchMask = BOT_SWITCH_TRIGGER;
        Serial.println("cal-down-big");
        break; 

      default: 
        break;   // not done
      } 
      break;
    
    case CALIBRATE_GOING_UP:
      switch (towerDirection)
      {
      case TOWER_STOPPED_SWITCH:
      
        // we hit the top switch. see how many ticks are left

        towerUpperLimit = towerLocation;
        towerLowerLimit = 0;        

        calibrateState = CALIBRATE_IDLE;
        Serial.print("cal-done: " );
        Serial.println(towerUpperLimit);
        break;
        
      case TOWER_STOPPED:
        // ran out of ticks, go down a bit more
        // we moved a whole bunch down but did not hit the switch.  

        switchMask = TOP_SWITCH_TRIGGER;
        calibrateState = CALIBRATE_GOING_UP;;
        startMotor('u',calibrateGoUp);
        Serial.println("cal-up-fine");
        break; 

      default: 
        break;   // not done
      }     
      break;   
  }
   
  if (switchTest)
  {
    temp = digitalRead(BOT_SWITCH_PIN) + (digitalRead(TOP_SWITCH_PIN) << 1);
    if (temp != switchTestLast)
    {
      Serial.println(temp,HEX);
      switchTestLast = temp;
    }
  } 
  if (pumpAuto) {
    // we are monitoring it

    if (pumpSampleTime < millis()) {
      // time to check if we should start or stop the pump

      tank = analogRead(TANK_SENSOR);
      Serial.print(tank);
      Serial.print(":");
      Serial.println((tank * 10) / 51);

      if (pumpOn) {
        // pump is on, has it reached the point where we can turn it off

        if (tank > pumpOffThreshold) {
          // yes, turn pump off

          pumpOn = 0;
          digitalWrite(PUMP_ON, HIGH);
          Serial.println("pump off");
        }
      } else {
        // pump is off, do we need to turn it on

        if (tank < pumpOnThreshold) {
          // pump needs to be turned on

          pumpOn = 1;
          digitalWrite(PUMP_ON, LOW);
          Serial.println("pump on");
        }
      }
      // set timer when we should check again

      pumpSampleTime = millis() + pumpSampleInterval;
    }
  }
}
