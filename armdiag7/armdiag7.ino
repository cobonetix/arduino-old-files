

#include <Wire.h>
#include <MCP23017.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// A pair of varibles to help parse serial commands (thanks Fergs)

int arg = 0;
int idx = 0;

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

unsigned char breg = 0;
unsigned char testreg;
unsigned char zerotest;
unsigned char zin;
unsigned char zlast = 0xff;

#define MCP23017_ADDR 0x20

#define GPA0 0
#define GPA1 1
#define GPA2 2
#define GPA3 3
#define GPA4 4
#define GPA5 5
#define GPA6 6
#define GPA7 7
#define GPB0 8
#define GPB1 9
#define GPB2 10
#define GPB3 11
#define GPB4 12
#define GPB5 13
#define GPB6 14
#define GPB7 15

MCP23017 mcp = MCP23017(MCP23017_ADDR);

#define ROT_CCW          'w'
#define ROT_CW         'c'
#define ROT_CCW_DEGREES  'y'
#define ROT_CW_DEGREES 'e'

#define ROT_HALT          'h'
#define ZERO              'z'

#define LOCATION          'l'
#define ROT_TO_POSITION   'g'
#define ARM_LOCK          'a'
#define DEMO              'd'
#define POINT             'p'
#define CALIBRATE         'b'
#define SET_ANGLES        's'

// Motor states
#define MOTOR_IDLE  0
#define MOTOR_CCW  1
#define MOTOR_CW 2


typedef struct MOTOR {
    char enGpio;
    char dirGpio;
    char stepGpio;
    char positionAnalogPin;

    unsigned char zeroMask;
    float ticksPerDegree;
    float avPerDegree;
    float ticksPerAv;
   
    char direction;
    int beginZeroMonitoring;
    
    char stopFlag;
    char pulseHigh;
    char speedMode;
    int position;

    int lastPositionDegrees;

    int targetAv;
    int currentAv;
    int lastAv;
    
    int ticksLeft;
    int ticksPerformed;           // how many ticks so far
    int ticksFullSpeedTrigger;    // when to go to full speed
    int ticksSlowDownTrigger;     // when to slow down
       
    int ticksSkip;                // ticks left to skip
    int tickSkipIntervalIndex;    // index into skip interval table
    int skipIntervalChangeIncrement;    // length of tick interval
    int ticksIntervalChangeTrigger;    // trigger to move to next interval
    int skipBuckets;
  } MOTOR;
 
typedef struct MOTOR_LIMITS{
  int lowerLimit;
  int upperLimit;
  int defaultSpeed;
} MOTOR_LIMITS;

#define NUMBER_POS_SETS  7

typedef struct POSITION_SET {
  int degree;
  int av;
  float aVPerDegree;
} POSITION_SET;


#define MOTORS_DEFINED 3

MOTOR motors[MOTORS_DEFINED] = { {GPA6,GPA7,7,A0,0x40,19.0,4.0,0.0,MOTOR_IDLE,0,0}, {GPA4,GPA5,8,A1,0x80,15.9,4.0,0.0,MOTOR_IDLE,0,0}, {GPA2,GPA3,10,A2,0x20,15.9,3.9,0.0,MOTOR_IDLE,0,0}  }; 

const MOTOR_LIMITS motorLimits[MOTORS_DEFINED] = {  {75,276,0},{80,274,0},{71,291,0} }  ;

const POSITION_SET positionSets[MOTORS_DEFINED][NUMBER_POS_SETS] = 
{
    {   
      {70,   924, float(924-850)/(90-75)},
      {90,   850, float(850.0-720)/45},
      {135,  720, float(720.0-507)/45},
      {180,  507, float(507.0-392)/45},
      {225,  392, float(392.0-129)/45},
      {270,  129, float(129.0-81)/45},
      {275,  81,  float(129.0-81)/45}
    },

     {   
      {70,   938, float(938-868)/float(90-75)},
      {90,   868, float(868-694)/float(45)},
      {135,  694, float(694-540)/float(45)},
      {180,  540, float(540-380)/float(45)},
      {225,  380, float(380-164)/float(45)},
      {270,  164, float(164-98)/float(45)},
      {275,  98,  float(129-81)/float(45)}
    },

    {
     {75, 119,  float(135.0-112)/float(90-75)},
     {90, 182,  float(384.0-182)/float(45)},
     {135,384,  float(507.0-384)/float(45)},
     {180,507,  float(681.0-507)/float(45)},
     {225,704,  float(930.0-681)/float(45)},
     {270,930,  float(1007.0-930)/float(45)},
     {300,1023, float(1007.0-930)/float(45)}
    }
};

int skipIntervalTable[] = {2,2,2,2,1,1,1,0};
volatile unsigned int tickA = 0;
volatile byte flag = 0;
unsigned long lastTime;

#define NUMB_POINTS 16
#define POINT_IDLE 0
#define POINT_ACTIVE 1

char pointState = POINT_IDLE;
char currentPoint;
char highestPoint;
int pointArray[NUMB_POINTS][3];


enum calibrationStates {CAL_IDLE,CAL_WAITING, ZERO_HOME, ZERO_90,ONE_HOME, ONE_90,TWO_HOME, TWO_90};

enum calibrationStates calState = CAL_IDLE;


#define NUM_SERVOS 4
Servo servo[NUM_SERVOS];  // create servo object to control a servo
int servoGpio [NUM_SERVOS] = {9,6,5,3};  // all these pins can do PWM


#define sol1  0x02
#define sol2  0x01
#define vvalve 0x10

#define ZEROMASK 0xE0
#define LIGHTMASK 0x01

unsigned char lighttest;
unsigned char lin;
unsigned char llast = 0xff;
unsigned long lighttestTime = 0;

unsigned long avReadTime = 0;
char avReadMode = MOTORS_DEFINED+1;

volatile unsigned char mcpPortin;

int demoCycles;
int demoCyclesCount;
int demoTicks;

#define MOTOR_ACC    0
#define MOTOR_STEADY 1
#define MOTOR_DEC    2



POSITION_SET * getPositionSetbyDegree(int motor,int degrees, char increasing)
{
  int i;

  if (increasing)  // look for highest
  {
    for (i = 0; i < NUMBER_POS_SETS; i++)
    {
//      Serial.println("Dposition decreasing");
//      Serial.println("Dposition itst :" + String(degrees) + " " + String( positionSets[motor][i].degree));
      if (degrees < positionSets[motor][i].degree){
//       Serial.println("Dposition set :" + String(i));
        return &positionSets[motor][i-1];
      }
    }
  }
  else
  {
//    Serial.println("Dposition decreasing");
    for (i = NUMBER_POS_SETS-1; i >= 0; i--) 
    {
//      Serial.println("Dposition dtst :" + String(degrees) + " " + String( positionSets[motor][i].degree));
       if (degrees > positionSets[motor][i].degree)
      {
//        Serial.println("Dposition set :" + String(i));
        return &positionSets[motor][i+1];
      }
    }
  }
  Serial.println("Dposition failure :" + String(i));
}

POSITION_SET * getPositionSetbyAv(int motor,int av, char increasing)
{
  int i;

  if (increasing)  // look for highest
  {
    for (i = 0; i < NUMBER_POS_SETS; i++)
    {
//      Serial.println("Aposition itst :" + String(av) + " " + String( positionSets[motor][i].av));

      if (av < positionSets[motor][i].av){
//        Serial.println("Aposition set :" + String(i));
        return &positionSets[motor][i-1];
      }
    }
  }
  else
  {
    for (i = NUMBER_POS_SETS-1; i >= 0; i--) 
    {
//      Serial.println("Aposition dtst :" + String(av) + " " + String( positionSets[motor][i].av));
      if (av > positionSets[motor][i].av)
      {
//        Serial.println("Aposition set :" + String(i));
        return &positionSets[motor][i+1];
      }
    }
  }
  Serial.println("Aposition failure :" + String(i));
}

ISR(TCA0_OVF_vect) {
  char m;

  TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm; //clear flag by writing 1 resets interrupt
  tickA++;

  if (tickA >=800) //4 x 250 ms overflow toggle
  {
    flag = 1;
    tickA = 0;
 //   digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
  
  for (m=0; m < MOTORS_DEFINED;m++)
  {
    if (motors[m].direction != MOTOR_IDLE)
    {
      //Serial.println(motors[m].ticksPerformed);
      //Serial.println(motors[m].zeroMask,HEX);
     // Serial.println(mcpPortin,HEX);

      if ( ((mcpPortin & motors[m].zeroMask) == 0) && (motors[m].ticksPerformed > motors[m].beginZeroMonitoring)  && (motors[m].stopFlag == 0 ) )
      {
        // we hit the zero indicator. stop the transfer
        motors[m].stopFlag = 1;
        Serial.print("zero detected m ");
        Serial.println(m);
        continue;
      }
    }  

    // see if skipping this tick interrupt
    
    if (motors[m].ticksSkip != 0)
    {
      // yes
      motors[m].ticksSkip--;
      continue;
    }

    // don't skip this interrupt but first see if we are done
    
    if ((motors[m].direction != MOTOR_IDLE) && (motors[m].stopFlag == 0)  )
    {

       motors[m].currentAv = analogRead(motors[m].positionAnalogPin);

       //Serial.println("Y " + String(motors[m].currentAv) + ":" + String( motors[m].targetAv));
      
      if(motors[m].direction == MOTOR_CW) 
      {

        // motors 0 and 1 rotate opposite motor 2
        
        if (m == 2)
        {
          if (motors[m].currentAv >= motors[m].targetAv)
          {
             // we are done
             motors[m].stopFlag = 1;
             Serial.println("X1 " + String(motors[m].ticksLeft) + " " + String(motors[m].currentAv) + ":" + String( motors[m].targetAv));
             continue;
          }
        } 
        else
        {
          if (motors[m].currentAv < motors[m].targetAv)
          {
             // we are done
             motors[m].stopFlag = 1;
             Serial.println("X2 " + String(motors[m].ticksLeft) + " " + String(motors[m].currentAv) + ":" + String( motors[m].targetAv));
             continue;
          }
        }       
      }
      else
      {
         // going CCW

        if (m == 2)
        {
          if (motors[m].currentAv < motors[m].targetAv) 
          {
             // we are done
             motors[m].stopFlag = 1;
             Serial.println("X3 " + String(motors[m].ticksLeft) + " " + String(motors[m].currentAv) + ":" + String( motors[m].targetAv));
             continue;
          }
        }
        else
        {
          if (motors[m].currentAv >= motors[m].targetAv) 
          {
             // we are done
             motors[m].stopFlag = 1;
             Serial.println("X4 " + String(motors[m].ticksLeft) + " " + String(motors[m].currentAv) + ":" + String( motors[m].targetAv));
             continue;
          }       
        } 
      }

      // do the tick
      
      if (motors[m].pulseHigh)
      {
        digitalWrite(motors[m].stepGpio, LOW);        
        motors[m].pulseHigh = 0;
        motors[m].ticksLeft --;
        motors[m].position += (motors[m].direction == MOTOR_CW ? 1 : -1);
        motors[m].ticksPerformed++; 

        // was that the last tick?
        
        if (motors[m].ticksLeft == 0) {
          
            Serial.println("X5 " + String(motors[m].currentAv) + ":" + String( motors[m].targetAv));
           
           // see if we are doing repeats for demo

           if (demoCycles != 0)
           {
             demoCycles--;
             motors[m].ticksLeft = demoTicks;
             motors[m].direction = (motors[m].direction == MOTOR_CCW ? MOTOR_CW : MOTOR_CCW);
           }
           motors[m].stopFlag = 1;
           continue;
        }
        
        // check if we are actually making progress
        
        //if ( abs(motors[m].currentAv -  motors[m].lastAv) <  1)
        //   Serial.println('j');

        motors[m].lastAv =  motors[m].currentAv;

        // there are ticks left. Is it time to change the skip interval?
        
        switch (motors[m].speedMode)
        {
        case MOTOR_ACC:
          // still getting up to speed. See if we are there
 //         Serial.print("a");
          
           if (motors[m].ticksPerformed >= motors[m].ticksFullSpeedTrigger )
           {
              // yes, go to steady
              motors[m].speedMode = MOTOR_STEADY;
              motors[m].ticksSkip = 0;
              //Serial.print("AS");
              continue;
           }

           // not time to go steady, is it time to change the skip value

          //Serial.print(motors[m].ticksPerformed);
          //Serial.print(":");          
          //Serial.println(motors[m].ticksIntervalChangeTrigger);
          
          if (motors[m].ticksPerformed >= motors[m].ticksIntervalChangeTrigger )
          {
            // bump interval count
              
            if (motors[m].tickSkipIntervalIndex != motors[m].skipBuckets-1 )
              motors[m].tickSkipIntervalIndex++;             
            // calc next update trigger
            motors[m].ticksIntervalChangeTrigger = motors[m].ticksPerformed + motors[m].skipIntervalChangeIncrement ;                 
           // Serial.print("Ai");
          }

          // load skip value
          motors[m].ticksSkip = skipIntervalTable[motors[m].tickSkipIntervalIndex];
//          Serial.println(motors[m].ticksSkip);
          break;
        
        case MOTOR_DEC:
 //         Serial.print("d");
  
          // see if time to go next interval
          if (motors[m].ticksPerformed >= motors[m].ticksIntervalChangeTrigger )
          {
            // dec interval count unless already at lowest value
              
            if (motors[m].tickSkipIntervalIndex != 0 )
              motors[m].tickSkipIntervalIndex--;
            
            motors[m].ticksIntervalChangeTrigger = motors[m].ticksPerformed + motors[m].skipIntervalChangeIncrement ;                 

            //Serial.print("Dd");              
          }
          // load skip value
          motors[m].ticksSkip = skipIntervalTable[motors[m].tickSkipIntervalIndex];
          //Serial.println(motors[m].ticksSkip);
         break;;
        
        case MOTOR_STEADY:
//          Serial.print("s");
          // see if time to start slowing down
          
          if (motors[m].ticksPerformed > motors[m].ticksSlowDownTrigger )
           {
              // yes
              motors[m].speedMode = MOTOR_DEC;
              motors[m].tickSkipIntervalIndex = motors[m].skipBuckets-1;
              motors[m].ticksIntervalChangeTrigger = motors[m].ticksPerformed + motors[m].skipIntervalChangeIncrement ;                 
              motors[m].ticksSkip = skipIntervalTable[motors[m].tickSkipIntervalIndex];
              //Serial.print("Sd");              
              //Serial.println(motors[m].ticksSkip);
         }
          else
            motors[m].ticksSkip = 0;
          break;  
        }          
       }
      else
      {
        // assert pulse low
     //   Serial.print("x");
        digitalWrite(motors[m].stepGpio, HIGH);        
        motors[m].pulseHigh = 1;
      }
    }
  }
}

void setup_timer_A() {
  //ATMega4809 runs at 16MHz without Main Clock Prescaller and default is TCA_SINGLE_CLKSEL_DIV64_gc
  //4us per tick

  TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm; //enable overflow interrupt
  TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_NORMAL_gc;//timer mode may break pwm

  //1ms = 250 x .000004  set H and L period registers

  TCA0.SINGLE.PERL = lowByte(250);
  TCA0.SINGLE.PERH = highByte(250);

  TCA0.SINGLE.CTRLA  |= TCA_SINGLE_ENABLE_bm;
}

void setupInt() {
  // put your setup code here, to run once:
 // pinMode(LED_BUILTIN, OUTPUT);
  setup_timer_A();
}

float aVPerDegree = 3.4;

int moveToPosition(int motor,int newPosition)
{
    int moveAv,moveTicks,increasingPosition,currentPosition;
     POSITION_SET * pset;

//       Serial.println("mtp: " + String(motor) + " " + String(newPosition) );

        if (newPosition > motorLimits[motor].upperLimit )
        {
          Serial.println ("ERR - too far");
          return 0;
        }
        
        if (newPosition < motorLimits[motor].lowerLimit )
        {
          Serial.println ("ERR - too close");
          return 0;
        }

        // get current position

        motors[motor].currentAv = analogRead(motors[motor].positionAnalogPin);

        if (motor == 2){
           // get current degree position from currentAv
           
           pset = getPositionSetbyAv(motor,  motors[motor].currentAv, true);
           currentPosition =pset->degree +  (motors[motor].currentAv - pset->av) / pset->aVPerDegree;
           
           // now get info for target
           
           increasingPosition =  (newPosition > currentPosition ? true : false);
           pset = getPositionSetbyDegree(motor,  newPosition,increasingPosition);
         }
        else 
        {
          pset = getPositionSetbyAv(motor,  motors[motor].currentAv, false);
          increasingPosition =  (newPosition > currentPosition ? true : false);
          
          currentPosition = pset->degree - ( pset->av - motors[motor].currentAv) / pset->aVPerDegree;
          pset = getPositionSetbyDegree(motor,  newPosition,!increasingPosition);
         }     

//        Serial.println("-cp " +  String(motors[motor].currentAv) + " " + String(currentPosition) + " " + String(increasingPosition));
         
         // we have info to calc targetAv
        
//        Serial.println("--- " + String(pset->av) + " " + String(pset->degree) + " " + String(pset->aVPerDegree));
        
        motors[motor].targetAv = pset->av + (newPosition - pset->degree) * pset->aVPerDegree;
                
        moveAv = motors[motor].targetAv - motors[motor].currentAv;               
        moveTicks = moveAv * motors[motor].avPerDegree;
        
//        Serial.println("--- " + String(moveAv = motors[motor].targetAv) + " " + String(moveAv) + " " + String(moveTicks));
 
         if (motor == 2)
        {
          startMotor(motor,(moveTicks >= 0 ? ROT_CW : ROT_CCW),abs(moveTicks)+1000,0);
        }
        else
        {
           startMotor(motor,(moveTicks < 0 ? ROT_CW : ROT_CCW),abs(moveTicks) + 1000,0);
        }
  return 1;
}

void startMotor(char motor, char cmd,int ticks, int repeat)
{
 
       if (cmd == ROT_CCW)
        {
          motors[motor].direction = MOTOR_CCW;
          mcp.digitalWrite(motors[motor].dirGpio,HIGH);
//          Serial.println("Going CCW");
        }
        else
        {
          motors[motor].direction = MOTOR_CW;
          mcp.digitalWrite(motors[motor].dirGpio,LOW);
//          Serial.println("Going CW");
         }

        motors[motor].stopFlag = 0;
        motors[motor].ticksLeft = (ticks == 0? 2 : ticks);
        motors[motor].pulseHigh = 0; 
        motors[motor].beginZeroMonitoring = (motors[motor].ticksLeft < 200 ? motors[motor].ticksLeft/2 : 200);
        motors[motor].ticksPerformed = 0; 
        motors[motor].skipBuckets = sizeof (skipIntervalTable)  /sizeof(skipIntervalTable[0]);
        motors[motor].ticksFullSpeedTrigger = min(200, motors[motor].ticksLeft/8);
        motors[motor].ticksSlowDownTrigger = motors[arg1].ticksLeft - motors[motor].ticksFullSpeedTrigger;
        motors[motor].skipIntervalChangeIncrement = motors[motor].ticksFullSpeedTrigger/motors[motor].skipBuckets;          
        
        motors[motor].speedMode = MOTOR_ACC;
        motors[motor].tickSkipIntervalIndex = 0;
        motors[motor].ticksSkip = skipIntervalTable[motors[motor].tickSkipIntervalIndex];          
        motors[motor].ticksIntervalChangeTrigger = motors[motor].ticksPerformed + motors[motor].skipIntervalChangeIncrement;
        demoTicks = motors[motor].ticksLeft;
        demoCycles = repeat;
        demoCyclesCount = 0;
        
        mcp.digitalWrite(motors[motor].enGpio ,LOW);
}


int runCommand() {
  int i = 0;
  int m;
  char *p = argv1;
  char *str;
    int rept, endpt,ticks,degreeChange;
  
  arg1 = atol(argv1);
  arg2 = atol(argv2);
  arg3 = atol(argv3);
  arg4 = atol(argv4);
  
   
  switch(cmd) {

  case '0':

    if (arg1 == 1)
      zerotest = 1;
    else
      zerotest = 0;
    Serial.println ("OK");
    break;

  case '1':   // solenoid 1 and 2
    if (arg1 == 1)
    {
      mcp.digitalWrite(GPA0,1);
      mcp.digitalWrite(GPA1,1);
     }
    else 
    {
        mcp.digitalWrite(GPA0,0);
        mcp.digitalWrite(GPA1,0);
    }
    Serial.println ("OK");
    break;


  case '2':   // solenoid 1
    if (arg1 == 1)
      mcp.digitalWrite(GPA0,1);
    else 
      mcp.digitalWrite(GPA0,0);
    Serial.println ("OK");
    break;

  case '3':  // solenoid 2
    if (arg1 == 1)
      mcp.digitalWrite(GPA1,1);
    else 
      mcp.digitalWrite(GPA1,0);
    Serial.println ("OK");
    break;

  case '4':  // vac 
    if (arg1 == 1)
      mcp.digitalWrite(GPB4,1);
    else 
      mcp.digitalWrite(GPB4,0);
    Serial.println ("OK");
    break;

  case '5':

    if (arg1 == 1)
      lighttest = 1;
    else
      lighttest = 0;
  
    lighttestTime = millis() + 1000;
    Serial.println ("OK");
    break;

  case '6':    // servo test   6 servo# endpt repeat

     if (arg1 < NUM_SERVOS)
     {
        arg1--;
        endpt = arg2 & 0xff;
        rept = (arg3 == 0 ? 1 : arg3);

        for (i = 0 ; i < rept; i++)
        {
          servo[arg1].write(0);
          delay(500);
          servo[arg1].write(endpt);
          delay(500);
        }
        Serial.println ("OK");
     }
     else
        Serial.println ("Err");
      break;

  case '7':
    
    if (arg1 <  MOTORS_DEFINED)
    {
     
      if (arg2 == 1)
      {
        avReadMode = arg1;
        avReadTime = millis() + 1000;
      } 
      else
        avReadMode = MOTORS_DEFINED+1;     
    }
    Serial.println ("OK");
    break;

  case ROT_CCW_DEGREES:
  case ROT_CW_DEGREES:
  
    pointState = POINT_IDLE;
    if (arg1 < MOTORS_DEFINED)
    {
        ticks = int(motors[arg1].ticksPerDegree * arg2);
        
        cmd = (cmd == ROT_CCW_DEGREES ? 'w' : 'c');

        Serial.println(ticks);
        startMotor(arg1, cmd,ticks, 0);
        Serial.println("OK ");
    }
    else
    {
      Serial.println("ERR"); 
    }
    break;

  case ROT_CCW:
  case ROT_CW:
      
    pointState = POINT_IDLE;
 
      if (arg1 < MOTORS_DEFINED)
      {
        startMotor(arg1, cmd,arg2, 0);
        Serial.println("OK ");
      }
      else
        Serial.println("ERR"); 
      break;

  case ROT_HALT:
      if (arg1 < MOTORS_DEFINED)
      {
        motors[arg1].stopFlag = 1;
        demoCycles = 0;
        motors[arg1].ticksLeft = 0;
        motors[arg1].pulseHigh = 0;           
        mcp.digitalWrite(motors[arg1].enGpio ,HIGH);
        Serial.println("OK");
      }
      else
        Serial.println("ERR");
      break;

  case ARM_LOCK:
      for (i=0; i < MOTORS_DEFINED;i++)
      {
        mcp.digitalWrite(motors[i].enGpio,(arg1 == 1? LOW : HIGH));
      }
      Serial.println("OK");
      break;

   
    
case ROT_TO_POSITION:
   pointState = POINT_IDLE;

    if (arg1 < MOTORS_DEFINED)
      {
         moveToPosition(arg1,arg2);
         Serial.println("OK");
      }
      else 
         Serial.println("ERR");
      break;
    
case DEMO:
    pointState = POINT_IDLE;
    delay (2000);
     for (i=0; i < MOTORS_DEFINED;i++)
     {
          motors[i].direction = (arg1 == 1 ? ROT_CCW: ROT_CW);
          mcp.digitalWrite(motors[i].dirGpio, (arg1 == 1 ? HIGH : LOW)); 
          
          motors[i].ticksLeft = (arg2 == 0? 2 : arg2);
          demoTicks = motors[i].ticksLeft;
          demoCycles = arg3;

          motors[i].pulseHigh = 0;           
          mcp.digitalWrite(motors[i].enGpio ,LOW);
      }
      Serial.println("OK");
      break;

  case ZERO:
     break;

  case SET_ANGLES:
    // set all three angles
    
    if (moveToPosition(0,arg1) == 0)
    {
      Serial.println("ERR 0");
      break;
    }
    
    if (moveToPosition(1,arg2) == 0)
    {
      Serial.println("ERR 1");
      break;
    }
 
    if (moveToPosition(2,arg3) == 0)
    {
      Serial.println("ERR 2");
      break;
    }
    Serial.println("OK");
    break;

  case LOCATION:
    Serial.print("OK: " + String(motors[0].position) );
    Serial.print(":"+ String(motors[0].position/motors[0].ticksPerDegree) );
    Serial.print("  "    + String(motors[1].position)) ;
    Serial.print(":" + String(motors[1].position/motors[1].ticksPerDegree) );
    Serial.print("  "  + String(motors[2].position));
    Serial.println(":"+ String(motors[2].position/motors[2].ticksPerDegree) );
    break;

  case POINT:
    pointState = POINT_IDLE;
    if (argv1[0] == 'i')
      highestPoint = 0;
    else if (argv1[0] == 'g')
    {
      currentPoint = 0;
      pointState = POINT_ACTIVE;
      for (m = 0; m < MOTORS_DEFINED; m++)
         moveToPosition(m, pointArray[0][m]);            
    }  
    else if (argv1[0] == 'l')
      {
        currentPoint = 0;
         for (m = 0; m < highestPoint; m++)
        {
          Serial.print("(" + String(pointArray[m][0]) ); 
          Serial.print("," + String(pointArray[m][1]) ); 
          Serial.println("," + String(pointArray[m][2]) + ")" ); 
        }                
      } 
      else
       {
        // add to list

        pointArray[highestPoint][0] = arg1;
        pointArray[highestPoint][1] = arg2;
        pointArray[highestPoint][2] = arg3;
        highestPoint++;
    }
    Serial.println("OK");
    break;

  case CALIBRATE:
   pointState = POINT_IDLE;
    Serial.println(argv1[0]);
    
    if (argv1[0] == 's')
    {
      for (i=0; i < MOTORS_DEFINED;i++)
      {
        mcp.digitalWrite(motors[i].enGpio,(arg1 == 1? LOW : HIGH));
      }
      
      calState = CAL_WAITING;
      Serial.println("Arrange arm and enter b p when done");      
    }
    else
       if (argv1[0] == 'p')
       {
          if (calState == CAL_WAITING)
          {
            calState = ZERO_HOME;
            startMotor(0, ROT_CCW,10000, 0);
            Serial.println("OK Proceeding");
          }
          else
            Serial.println("ERR");
       }
       else
         Serial.println("ERR");
    break;


   default:
      break;
  }
  return 0;
}

/* Clear the current command parameters */
void resetCommand() {
   
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  memset(argv3, 0, sizeof(argv3));
  memset(argv4, 0, sizeof(argv4));
 
  idx = 0;
  arg = 0;

}


void setup() {

    int i;
  
     Serial.begin(9600);
  
     MCP23017 mcp = MCP23017(MCP23017_ADDR);
      
     Wire.begin();
     mcp.init();  

    mcp.portMode(MCP23017Port::A, 0);          //Port A as output
    mcp.portMode(MCP23017Port::B, 0b11101111); //Port B as input


    mcp.pinMode(GPB7, INPUT_PULLUP, 0);
    mcp.pinMode(GPB6, INPUT_PULLUP, 0);
    mcp.pinMode(GPB5, INPUT_PULLUP, 0);

    mcp.writeRegister(MCP23017Register::GPIO_A, 0x00);  //Reset port A 
    mcp.writeRegister(MCP23017Register::GPIO_B, 0x00);  //Reset port B
      
    mcp.writePort(MCP23017Port::A, 0);
    
    testreg = mcp.readRegister(MCP23017Register::IOCON);
    //Serial.println(testreg);

    for (i = 0; i < MOTORS_DEFINED; i++)
    {
      pinMode(motors[i].stepGpio ,OUTPUT);
      digitalWrite(motors[i].stepGpio,HIGH);   
      mcp.digitalWrite(motors[i].enGpio,LOW) ;
      motors[i].direction = MOTOR_IDLE; 

      motors[i].ticksPerAv = motors[i].ticksPerDegree/motors[i].avPerDegree;

    }
     
 
    for (i = 0; i < NUM_SERVOS;i++)
    {
     servo[i].attach(servoGpio[i]);
     servo[i].write(0);
    }
    setupInt();
    Serial.println("OK-Servo Setup Done");

    avReadMode = MOTORS_DEFINED+1;
    calState = CAL_IDLE;
}

void loop() {

    char *p;
  
  // read mcp port here since we cannot do it in ISR
  
  mcpPortin = mcp.readPort(MCP23017Port::B) & ZEROMASK;
       
  // handle any pending characters
  
  while (Serial.available() > 0) {
    
    // Read the next character
    chr = Serial.read();
 
    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[idx] = NULL;
      else if (arg == 2) argv2[idx] = NULL;
      else if (arg == 3) argv3[idx] = NULL;
      else argv4[idx] = NULL;

      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[idx] = NULL;
        arg = 2;
        idx = 0;
      } else if (arg == 2)  {
        argv2[idx] = NULL;
        arg = 3;
        idx = 0;
      } else if (arg == 3)  {
        argv3[idx] = NULL;
        arg = 4;
        idx = 0;
      } else if (arg == 4)  {
        argv4[idx] = NULL;
        arg = 5;
        idx = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[idx] = chr;
        idx++;
      }
      else if (arg == 2) {
        argv2[idx] = chr;
        idx++;
      }
      else if (arg == 3) {
        argv3[idx] = chr;
        idx++;
      }
      else if (arg == 4) {
        argv4[idx] = chr;
        idx++;
      }
    }
  }

  // do idle setting here since cannot to mcp calls from inside an ISR
  
  if (demoCycles == 0)
  {
    for (char m=0; m < MOTORS_DEFINED;m++)
    {
      if (motors[m].direction == MOTOR_IDLE)
        continue;
        
      if (motors[m].stopFlag == 1)
      {
         // mcp.digitalWrite(motors[m].enGpio,HIGH);
         motors[m].direction = MOTOR_IDLE;
       
         if (motors[m].ticksLeft != 0)
         {
           // we aborted early. display remaining ticks. At some point this will be part of calibration process. 

           Serial.print("Ticks Remaining: ");
           Serial.println(motors[m].ticksLeft );
         }
      } 
    }
  }
  else
  {
    delay(5000);
    Serial.print("Cycles: ");
    Serial.println(++demoCyclesCount);
    demoCycles--;
   
    for (char m=0; m < MOTORS_DEFINED;m++)
    {
      if (motors[m].stopFlag == 1)
      {
        if (motors[m].ticksLeft != 0)
        {
           // we aborted early. display remaining ticks. At some point this will be part of calibration process. 

           Serial.print("Demo aborted, Ticks Remaining: ");
           Serial.println(motors[m].ticksLeft ); 
           motors[m].direction = MOTOR_IDLE;
           demoCycles = 0;  
           continue;
        }
    
        if (motors[m].direction == MOTOR_IDLE)
          continue;
        
        motors[m].ticksLeft = demoTicks;
        if (motors[m].direction == MOTOR_CCW)
        {
           motors[m].direction = MOTOR_CW;
           mcp.digitalWrite(motors[m].dirGpio,LOW);
        }
        else
        {
          motors[m].direction = MOTOR_CCW;
          mcp.digitalWrite(motors[m].dirGpio,HIGH);          
        }  
        motors[m].speedMode = MOTOR_ACC;
        motors[m].ticksPerformed = 0;
        motors[m].tickSkipIntervalIndex = 0;
        motors[m].ticksSkip = skipIntervalTable[motors[m].tickSkipIntervalIndex];          
        motors[m].ticksIntervalChangeTrigger = motors[m].ticksPerformed + motors[arg1].skipIntervalChangeIncrement;
        motors[m].stopFlag = 0;
      }
    }
  }
  
  switch (calState){

  case CAL_IDLE:
    break;
    
  case CAL_WAITING:
    break;

  case ZERO_HOME:
    // ignore til movement is done
    if  (motors[0].stopFlag == 0)
      break;

    motors[0].position = motorLimits[0].lowerLimit * motors[0].ticksPerDegree;
    // move it 90 degrees
    calState = ZERO_90;
    startMotor(0, ROT_CW,90*motors[0].ticksPerDegree, 0);     
    break;

  case ZERO_90:
    // ignore til movement is done
    if  (motors[0].stopFlag == 0)
      break;

    calState = ONE_HOME;
    startMotor(1, ROT_CCW,10000, 0);
    break;

  case ONE_HOME:
    // ignore til movement is done
    if  (motors[1].stopFlag == 0)
      break;

    // move it 90 degrees
    motors[1].position = motorLimits[1].lowerLimit*motors[1].ticksPerDegree;
    calState = ONE_90;
    startMotor(1,ROT_CW,90*motors[1].ticksPerDegree, 0);    
    break;

  case ONE_90:
    // ignore til movement is done
    if  (motors[1].stopFlag == 0)
      break;
    calState = TWO_HOME;
    startMotor(2, ROT_CCW,10000, 0);
    break;

  case TWO_HOME:
    // ignore til movement is done
    if  (motors[2].stopFlag == 0)
      break;
    motors[2].position = motorLimits[2].lowerLimit*motors[2].ticksPerDegree;
    calState  = TWO_90;
    startMotor(2,ROT_CW,90*motors[2].ticksPerDegree, 0);     
    break;

  case TWO_90:
    // ignore til movement is done
    if  (motors[2].stopFlag == 0)
      break;
 
    Serial.println("Calibration Complete");
    calState = CAL_IDLE;
    break;
  }

  if (pointState == POINT_ACTIVE)
  {
    // we are doing point move, we go on to the next point when all three motors are stopped

    if (motors[0].stopFlag && motors[1].stopFlag && motors[2].stopFlag )
    { 
       // move on to the next point if there is one
  
      currentPoint++;
      
      if (currentPoint < highestPoint)
       {
          // got another point
          //Serial.print("next point ");
         // Serial.println(currentPoint);

          for (char m = 0; m < MOTORS_DEFINED; m++)
            moveToPosition(m, pointArray[currentPoint][m]);            
       }
       else
       {
         pointState = POINT_IDLE;
         Serial.println("Movement Complete");
       }
    }
  }

  if (zerotest)
  {
     zin = mcp.readPort(MCP23017Port::B) & ZEROMASK;
     if (zin != zlast){
       Serial.println(zin,HEX);
       zlast = zin;
     }
  }

  if (lighttest) {
    lin = mcp.readPort(MCP23017Port::B) & LIGHTMASK;
    if (lin != llast) {
       Serial.println(lin);
       llast = lin;
    }
  
    if (lighttestTime < millis()) {
      Serial.print("V: ");
      Serial.println(analogRead(A7));
      lighttestTime = millis() + 1000;     // every second
    }
  }

  if (avReadTime  < millis())
  {
    avReadTime = millis() + 1000;
    switch (avReadMode)
    {
    case 0:
      Serial.println (analogRead(A0));
      break;
      
    case 1:
      Serial.println (analogRead(A1));
      break;
      
    case 2:
      Serial.println (analogRead(A2));
      break;

    default:
      break;
    }
  }
}  
