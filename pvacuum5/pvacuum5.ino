
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


#define TANK_SENSOR A3
#define ATTACH_SENSOR A1

#define PUMP_ON 12
#define VACCUM_ON 13
#define LIFT_MAIN1 11
#define LIFT_P1 10U
#define LIFT_P2 9

#define ST_EN 6
#define ST_DIR 5
#define ST_PL 7

#define TOWER_STOPPED 0
#define TOWER_UP 1
#define TOWER_DOWN 2

#define TOWER_ACC 0
#define TOWER_STEADY 1
#define TOWER_DEC 2


int towerMode = TOWER_ACC;
int towerSteadyTrigger = 0;
int towerDecTrigger = 0;
int towerTickCount = 0;
int towerIntervalCount = 0;
//int towerIntervalTable[] = {16,16,16,18,8,8,4,1};
int towerIntervalTable[] = { 4, 4, 4, 3, 3, 3, 2, 1 };
//int towerIntervalTable[] = {4,4,4,3,3,2,2,1};
int towerIntervalTableIndex = 0;
int towerBuckets = sizeof(towerIntervalTable) / sizeof(towerIntervalTable[0]);
int towerIntervalChangeTrigger = 0;
int towerIntervalChangeIncrement = 0;


int towerDirection = TOWER_STOPPED;
int towerLocation = 0;
int towerLimit;
int towerUpperLimit = 1180;
int towerLowerLimit = 0;
int inchesToTicks = 43;
int towerSpeed = 0;
int towerCurrentSpeed = 0;

// tanks  100psi = 511

// PUMP Control variables

char pumpAuto = 0;
char pumpOn = 0;
int pumpOnThreshold = 250;   // above this turn pump on  ~50 psi
int pumpOffThreshold = 512;  // below this turn pump off ~100 psi

int pumpSampleInterval = 1000;
long pumpSampleTime = 0;

#define PUMP_LIMIT 470 /* about 90 pounds */

// attach sequence control variables

int attachTriggerThreshold;
int defaultAttachTriggerThreshold = (3.5 * 1023) / 5;

long attachTimeoutTime;
int defaultAttachTimeoutInterval = 5000;  // 5 seconds
int attachTimeoutInterval;

char attachAuto = 0;
char attachTrying = 0;


////////////////////////////////////////////////////

// command line values

#define VAC 'v'
#define SENSOR 's'
#define PUMP 'p'
#define RESET 'z'
#define PUMP_MON 'm'
#define ATTACH 'a'
#define RELEASE 'r'

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

    case VAC:

      if (argv1[0] == 'o')
        digitalWrite(VACCUM_ON, LOW);
      else
        digitalWrite(VACCUM_ON, HIGH);

      Serial.println("OK");
      return;

    case SENSOR:
      if (argv1[0] == 't') {
        tank = analogRead(TANK_SENSOR);
        Serial.print(tank);
        Serial.print(":");
        Serial.print((tank * 10) / 51);
        Serial.println("OK");
      } else if (argv1[0] == 'a') {
        tank = analogRead(ATTACH_SENSOR);
        Serial.print(tank);
        Serial.print(":");
        Serial.print((tank * 10) / 51);
        Serial.println("OK");
      } else
        Serial.println("ERR");
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
        attachAuto = 0;
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

    case ATTACH:
      if (arg1 != 0)
        attachTimeoutInterval = arg1;  // time in ms
      else
        attachTimeoutInterval = defaultAttachTimeoutInterval;

      if (arg2 != 0)
        attachTriggerThreshold = arg2;
      else
        attachTriggerThreshold = defaultAttachTriggerThreshold;

      attachAuto = 1;
      //pumpOn = 0;  // !!!!assume we are already in auto mode
      pumpSampleTime = 0;
      attachTrying = 0;

      digitalWrite(PUMP_ON, LOW);
      Serial.println("OK");
      break;

    case RELEASE:
      attachAuto = 0;

      // just close vaccum
      digitalWrite(VACCUM_ON, HIGH);

      Serial.println("OK");
      break;

    case LIFT:
      if (argv1[0] == 'u')  // up
      {
        digitalWrite(LIFT_MAIN1, LOW);
        digitalWrite(LIFT_P1, LOW);
        digitalWrite(LIFT_P2, LOW);
      } else if (argv1[0] == 'd')  // down
      {
        digitalWrite(LIFT_MAIN1, LOW);
        digitalWrite(LIFT_P1, HIGH);
        digitalWrite(LIFT_P2, HIGH);
      } else  //stop
      {
        digitalWrite(LIFT_MAIN1, HIGH);
      }
      Serial.println("OK");
      break;

    case QUICK:

      // turn vac on
      digitalWrite(VACCUM_ON, LOW);

      // wait 1 seconds and turn off vac

      delay(1000);

      // close both vac and hold on

      digitalWrite(VACCUM_ON, HIGH);
      Serial.println("OK");
      break;

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

    case TOWER:

      switch (argv1[0]) {

        case 'l':  // return current location
          Serial.print("OK: ");
          Serial.print(towerLocation);
          Serial.print("  HL: ");
          Serial.print(towerUpperLimit);
          Serial.print("  LL: ");
          Serial.print(towerLowerLimit);
          Serial.print("  SP: ");
          Serial.println(towerCurrentSpeed);
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

        case 'c':  // calibrate top/bottom
          if (argv2[0] == 't') {
            towerUpperLimit = arg3;
          } else if (argv2[0] == 'b') {
            towerLowerLimit = arg3;
          } else if (argv2[0] == 'c') {
            towerLocation = arg3;
          }
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
          if (arg3 > 32000) {
            Serial.println("ERR: movement limited to 32000 ticks");
            return;
          }
          if (argv2[0] == 'u') {
            towerLimit = towerLocation + arg3;
            towerDirection = TOWER_UP;
            towerMode = TOWER_ACC;
            towerTickCount = 0;
            towerIntervalTableIndex = 0;
            towerIntervalCount = towerIntervalTable[towerIntervalTableIndex];
            ;
            towerSteadyTrigger = min(arg3 / 8, 200);
            towerDecTrigger = arg3 - towerSteadyTrigger;
            //Serial.println(towerSteadyTrigger);
            //Serial.println(towerDecTrigger);
            towerIntervalChangeIncrement = towerSteadyTrigger / towerBuckets;
            towerIntervalChangeTrigger = towerIntervalChangeIncrement;
            //Serial.println(towerIntervalChangeIncrement);
            digitalWrite(ST_EN, LOW);
            digitalWrite(ST_DIR, HIGH);
          } else {
            if (argv2[0] == 'd')
              towerLimit = towerLocation - arg3;
            towerDirection = TOWER_DOWN;
            towerMode = TOWER_ACC;
            towerIntervalTableIndex = 0;
            towerTickCount = 0;
            towerIntervalCount = towerIntervalTable[towerIntervalTableIndex];
            ;
            towerSteadyTrigger = min(arg3 / 8, 200);
            towerDecTrigger = arg3 - towerSteadyTrigger;
            //Serial.println(towerSteadyTrigger);
            //Serial.println(towerDecTrigger);
            towerIntervalChangeIncrement = towerSteadyTrigger / towerBuckets;
            towerIntervalChangeTrigger = towerIntervalChangeIncrement;
            //Serial.println(towerIntervalChangeIncrement);
            digitalWrite(ST_EN, LOW);
            digitalWrite(ST_DIR, LOW);
          }
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
  // Doing something here inside ISR

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

  //   relays are off except sensor power (for now)

  pinMode(VACCUM_ON, OUTPUT);  // Vac valve is closed
  digitalWrite(VACCUM_ON, HIGH);

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


  // Init timer ITimer1
  ITimer1.init();

#define TIMER_INTERVAL_MS 1

  // Interval in unsigned long millisecs
  if (ITimer1.attachInterruptInterval(TIMER_INTERVAL_MS, TimerHandler))
    Serial.println("Starting  ITimer OK, millis() = " + String(millis()));
  else
    Serial.println("Can't set ITimer. Select another freq. or timer");

  // change the timer interval to 512us rather than 1ms

  OCR1A = OCR1A / 2;
  Serial.println("VSetup Done1");
}


/////////////////////
///
// Main execution loop

void loop() {

  int tank, attach;

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

  // Handle monitoring of vacuum tank state

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

  // now see if we are doing an attach sequence

  if (attachAuto) {
    // trying to do it. we assume the attach valve has been opened and so we are
    // just waiting for things to stick

    if (attachTrying) {
      // we are trying, have we made it yet?

      delay(100);
      attach = analogRead(ATTACH_SENSOR);
      tank = analogRead(TANK_SENSOR);

      Serial.println(attach);
      Serial.println(tank);

      if (attach < attachTriggerThreshold) {
        Serial.println("Attach");
        attachTrying = 0;
        attachAuto = 0;
        return;
      } else {
        // see if timed out

        if (millis() > attachTimeoutTime) {
          // timed out, stop and give error
          attachTrying = 0;
          attachAuto = 0;
          digitalWrite(VACCUM_ON, HIGH);
          Serial.println("Timeout");
          return;
        }
      }
      return;
    } else {
      // we are going to start attaching but make sure vac is at right level

      if (!pumpOn) {
        attachTimeoutTime = millis() + attachTimeoutInterval;
        attachTrying = 1;
        digitalWrite(VACCUM_ON, LOW);
      }
    }
  }
}
