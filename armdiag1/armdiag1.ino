

#include <Wire.h>
#include <MCP23017.h>

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

#define ROT_LEFT   'w'
#define ROT_RIGHT  'c'
#define ROT_HALT   'h'
#define ZERO       'z'
#define POSITION   'p'
#define ROT_TO_POSITION 'g'
#define ARM_LOCK   'a'
#define DEMO 'd'


// Motor states
#define MOTOR_IDLE  0
#define MOTOR_LEFT  1
#define MOTOR_RIGHT 2
#define MOTOR_SET_IDLE 3



typedef struct MOTOR {
    int enGpio;
    int dirGpio;
    int stepGpio;
    int direction;
    int ticksLeft;
    int pulseHigh;
    int position;
    int speed;
} MOTOR;

typedef struct MOTOR_LIMITS{
  int lowerLimit;
  int upperLimit;
  int defaultSpeed;
} MOTOR_LIMITS;


#define MOTORS_DEFINED 3

MOTOR motors[MOTORS_DEFINED] = { {GPA6,GPA7,7,MOTOR_IDLE,0,0}, {GPA4,GPA5,8,MOTOR_IDLE,0,0}, {GPA2,GPA3,10,MOTOR_IDLE,0,0}  }; 

const MOTOR_LIMITS motorLimits[MOTORS_DEFINED] = {  {0,0,0},{0,0,0},{0,0,0} }  ;

volatile unsigned int tickA = 0;
volatile byte flag = 0;
unsigned long lastTime;

#define NUM_SERVOS 4
Servo servo[NUM_SERVOS];  // create servo object to control a servo
int servoGpio [NUM_SERVOS] = {9,6,5,3};

#define zero1 0x80
#define zero2 0x40
#define zero3 0x20
#define sol1  0x02
#define sol2  0x01
#define vvalve 0x10
 
unsigned char breg = 0;
unsigned char testreg;
unsigned char zerotest;
unsigned char zin;
unsigned char zlast = 0xff;
unsigned char lighttest;
unsigned char lin;
unsigned char llast = 0xff;
unsigned long lighttestTime = 0;


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
      if (motors[m].pulseHigh)
      {
        //Serial.print(".");
        //Serial.print(motors[m].ticksLeft);

        digitalWrite(motors[m].stepGpio, LOW);        
        motors[m].pulseHigh = 0;
        motors[m].ticksLeft --;
        motors[m].position += (motors[m].direction == MOTOR_LEFT ? 1 : -1);

        if (motors[m].ticksLeft == 0) {
         // Serial.print("Z");
          motors[m].direction = MOTOR_SET_IDLE;
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

int runCommand() {
  int i = 0;
  int m;
  char *p = argv1;
  char *str;
  
  arg1 = atol(argv1);
  arg2 = atol(argv2);
  arg3 = atol(argv3);
  arg4 = atol(argv4);
  
   
   switch(cmd) {

    case ROT_LEFT:
    case ROT_RIGHT:

      if (arg1 < MOTORS_DEFINED)
      {
        if (cmd == ROT_LEFT)
        {
          motors[arg1].direction = ROT_LEFT;
          mcp.digitalWrite(motors[arg1].dirGpio,HIGH);
        }
        else
        {
          motors[arg1].direction = ROT_RIGHT;
          mcp.digitalWrite(motors[arg1].dirGpio,LOW);
         }

        motors[arg1].ticksLeft = (arg2 == 0? 2 : arg2);
        motors[arg1].pulseHigh = 0;           
        mcp.digitalWrite(motors[arg1].enGpio ,LOW);
        Serial.println("OK ");
      }
      else
         Serial.println("ERR");
      
      break;

     case ROT_HALT:
      if (arg1 < MOTORS_DEFINED)
      {
        motors[arg1].direction = MOTOR_IDLE;
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
      Serial.println("ERR");
      break;

    case ROT_TO_POSITION:
      if (arg1 < MOTORS_DEFINED)
      {
        if (arg2 > motorLimits[arg].upperLimit )
        {
          Serial.println ("ERR - too far");
        }
        
        if (arg2 < motorLimits[arg].lowerLimit )
        {
          Serial.println ("ERR - too far");
        }

        if (motors[arg1].position > arg2)
        {
          motors[arg1].direction = ROT_LEFT;
          mcp.digitalWrite(motors[arg1].dirGpio,HIGH);
          Serial.println("going left");
        }
        else
        {
          motors[arg1].direction = ROT_RIGHT;
          mcp.digitalWrite(motors[arg1].dirGpio,LOW);
           Serial.println("going right");
        }

        motors[arg1].ticksLeft = abs(motors[arg1].position - arg2);
        motors[arg1].pulseHigh = 0;           
        mcp.digitalWrite(motors[arg1].enGpio,LOW);
        Serial.println("OK ");
      }
      else
         Serial.println("ERR");
      
      break;
    
    case DEMO:
     delay (2000);
     for (i=0; i < MOTORS_DEFINED;i++)
     {
          motors[i].direction = (arg1 == 1 ? ROT_LEFT: ROT_RIGHT);
          mcp.digitalWrite(motors[i].dirGpio, (arg1 == 1 ? HIGH : LOW)); 
          motors[i].ticksLeft = (arg2 == 0? 2 : arg2);
          motors[i].pulseHigh = 0;           
          mcp.digitalWrite(motors[i].enGpio ,LOW);
      }
      Serial.println("OK");
      break;


 
    case ZERO:
      if (arg1 < MOTORS_DEFINED)
      {
        motors[arg1].position = 0;
        Serial.println("OK");
      }
      else
        Serial.println("ERR");
      break;

    case POSITION:
      if (arg1 < MOTORS_DEFINED)
      {
        Serial.println("OK: " + String(motors[arg1].position));
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
    Serial.println(testreg);

  for (i = 0; i < MOTORS_DEFINED; i++)
  {
    pinMode(motors[i].stepGpio ,OUTPUT);
    digitalWrite(motors[i].stepGpio,HIGH);   
    mcp.digitalWrite(motors[i].enGpio,LOW) ; 

  }
  
  for (i = 0; i < NUM_SERVOS;i++)
    {
     servo[i].attach(servoGpio[i]);
     servo[i].write(0);
    }
  setupInt();
  Serial.println("OK-Servo Setup Done");
}

void loop() {

    char *p;
  
     
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

  Serial.println(cmd);
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

 for (char m=0; m < MOTORS_DEFINED;m++)
 {
   if (motors[m].direction == MOTOR_SET_IDLE) {
      // mcp.digitalWrite(motors[m].enGpio,HIGH);
       motors[m].direction = MOTOR_IDLE ;
   }
}

  if (zerotest)
  {
     zin = mcp.readPort(MCP23017Port::B);
     if (zin != zlast){
       Serial.println(zin);
       zlast = zin;
     }
  }


}  
    
