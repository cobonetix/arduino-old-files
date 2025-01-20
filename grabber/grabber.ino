
 char debugPrint = 0;

// A pair of varibles to help parse Serial commands (thanks Fergs)
int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments

char argv1[32];
char argv2[32];
char argv3[32];

// The arguments converted to integers

long arg2;
long arg3;

/* Serial port baud rate */
#define BAUDRATE     57600

// commands

#define GET_BAUDRATE   'b'
#define STATUS         's'
#define RESET          'r'
#define HALT           'h'
#define EXECUTE        'e'
#define PARAMETER      'p'
#define USE_DEFAULT    'u'
#define CLEAR          'c'
#define VALVE          'v'
#define HEATERC        'g'
#define VTANK          't'


 
// GPIO PIN USAGE

#define VAC_VALVE_1    2
#define VAC_VALVE_2    3
#define PRESSURE_VALVE 4
#define VACUUM_PUMP    5   /* digital output */
#define HEATER         6


#define TANK_VACUUM    A0     /* analog input */
#define GRIPPER_VACUUM A1     /* analog input */

#define DEFAULT_VTANK_ON_LEVEL     100
#define DEFAULT_VTANK_OFF_LEVEL    200
#define DEFAULT_PTANK_ON_LEVEL     100
#define DEFAULT_PTANK_OFF_LEVEL    200

#define DEFAULT_ATTACH_MADE_LEVEL  100


#define DEFAULT_HEATER_PREHEAT_TIME   2000
#define DEFAULT_PRESSURE_ON_TIME      3000

#define DEFAULT_ATTACH_TIMEOUT        2000
#define DEFAULT_ATTACH_RETRY_LIMIT       5
#define DEFAULT_ATTACH_RETRY_TIME     1000
#define DEFAULT_ATTACH_HOLD_TIME      5000

#define TASK_IDLE             0
#define TASK_REMOVAL          1
#define TASK_PLACE            2

#define REMOVE_IDLE           0
#define REMOVE_START          1
#define REMOVE_HEATER_ON      2
#define REMOVE_PRESSURE_ON    3
#define REMOVE_HOT_AIR_ON     4
#define REMOVE_VACUUM_ON      5
#define REMOVE_ATTACH_FAILED  6
#define REMOVE_ATTACH_MADE    7
#define REMOVE_RETRY_WAIT     8
#define REMOVE_VACUUM_WAIT    9
#define REMOVE_ATTACH_RELEASE 10

#define VPUMP_IDLE            0
#define VPUMP_OFF             1
#define VPUMP_ON              2


int hpt = DEFAULT_HEATER_PREHEAT_TIME;   // heater preheat on time in ms
int pot = DEFAULT_PRESSURE_ON_TIME;      // pressure on time in ms
int ato = DEFAULT_ATTACH_TIMEOUT;        // attach process timeout in ms
int aml = DEFAULT_ATTACH_MADE_LEVEL;     // attachment made indication level 
int arl = DEFAULT_ATTACH_RETRY_LIMIT;    // attach retry limit
int arw = DEFAULT_ATTACH_RETRY_TIME;     // attach retry wait time in ms

int aht = DEFAULT_ATTACH_HOLD_TIME ;     // attach hold timein ms

int vtonl = DEFAULT_VTANK_ON_LEVEL;      // vacuum tank on level
int vtofl = DEFAULT_VTANK_OFF_LEVEL;     // vacuum tank off level

int ptonl = DEFAULT_PTANK_ON_LEVEL;      // pressure tank on level
int ptofl = DEFAULT_PTANK_OFF_LEVEL;     // pressure tank off level


int attachRetryCount = 0;
long heaterOffTime = 0;
long actionStopTime = 0;
int taskState = TASK_IDLE;
int removeState = REMOVE_IDLE;
int vacuumPumpState =  VPUMP_IDLE;

int gpioState[HEATER+2] = {0,0,0,0,0,0,0};

int valveId;


typedef struct {
  char *n;
  int *vp;
  int maxValue;
} Parameter_def;

Parameter_def parameterTable [] = {
 
    {"hpt",&hpt,5000},      
    {"pot",&pot,5000},
    {"ato",&ato,5000},
  {"aml",&aml,1023},
  {"arl",&arl,1023},
  {"arw",&arw,5000},
  {"aht",&aht,5000},
  {"vtonl",&vtonl,1023},
  {"vtofl",&vtofl,1023},
  {"ptonl",&ptonl,1023},
  {"ptofl",&ptofl,1023},
  {"",(int *)0}
  };


 

/* Clear the current command parameters */

void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  memset(argv3, 0, sizeof(argv3));
  arg2 = 0;
  arg3 = 0;
  arg = 0;
  index = 0;
}


void clearHardware() {
  // cfg relay controls. all are high which keeps the relays not latched
  
  pinMode(VACUUM_PUMP , OUTPUT);
  digitalWrite(VACUUM_PUMP ,HIGH);
 
  pinMode( HEATER, OUTPUT);
  digitalWrite(HEATER ,HIGH);

  pinMode( VAC_VALVE_1, OUTPUT);
  digitalWrite(VAC_VALVE_1 ,HIGH);
 
  pinMode(VAC_VALVE_2 , OUTPUT);
  digitalWrite(VAC_VALVE_2 ,HIGH);

  pinMode(PRESSURE_VALVE , OUTPUT);
  digitalWrite(PRESSURE_VALVE ,HIGH);
  
}
void setup() 
{
  Serial.begin(BAUDRATE);          //  setup Serial

  taskState = TASK_IDLE;
  removeState = REMOVE_IDLE;
  
  clearHardware();
 
  Serial.println ("Idle");
  
}

void printStats(int i)
{
  char buff[80];

  Serial.println(buff);
}  
  
/* Run a command.  Commands are defined in commands.h */
int runCommand() {
  int i = 0;
  char *str;
  arg2 = atoi(argv2);
  arg3 = atoi(argv3);
  char buff[80];

#if 0

  Serial.println("a1-");
  Serial.println( argv1);
  Serial.println("a2-");
  if (*argv2) Serial.println( argv2);
  Serial.println("a3-");
  if (*argv3) Serial.println( argv3);
#endif
  
  switch(cmd) {
    
  case GET_BAUDRATE:
    Serial.println(BAUDRATE);
    Serial.println("OK"); 
   break;

  case  RESET:
    break;
     
  case  HALT:
     setup();
     Serial.println("Attach Idle"); 
     break;
   
  case  EXECUTE:
     taskState = TASK_REMOVAL;
     removeState = REMOVE_START;
     Serial.println("Attach Started"); 
     break;
     
 case  STATUS:
     Serial.print("Task State: ");
     Serial.println(taskState,DEC);
     Serial.print("Removal State: ");
     Serial.println(removeState,DEC);
   
     Serial.print("Vacuum Levels: ");
     Serial.print(  analogRead(TANK_VACUUM),DEC);
     Serial.print(":");
     Serial.println(analogRead(GRIPPER_VACUUM),DEC);

     Serial.print("Heater State: ");
     Serial.println(  gpioState[HEATER],DEC);
   
     Serial.print("Valve State (v1,v2,pr): ");
     Serial.print(    gpioState[VAC_VALVE_1],DEC);
     Serial.print(    gpioState[VAC_VALVE_2],DEC);
     Serial.println(  gpioState[PRESSURE_VALVE],DEC);
   
     Serial.print("Vacuum Pump gpio: ");
     Serial.println(  gpioState[VACUUM_PUMP] ,DEC);

     Serial.print("Vacuum Pump State: ");
     Serial.println( vacuumPumpState ,DEC);
     
     break;
     

  case PARAMETER:
  
     // if not parameters, just print all values
   
   Serial.println("Parameter Values");
   
   if (strlen(argv1) == 0)
   {   
     for (i =0; strlen(parameterTable[i].n) != 0; i++)
     {
           Serial.print(parameterTable[i].n);
           Serial.print(": ");           
           Serial.println (*parameterTable[i].vp,DEC);
     }
     return;   
   }
   
   // look through parameter table looking for match
   
   for (i = 0; strlen(parameterTable[i].n) != 0; i++)
   {
      
     if (strcmp(parameterTable[i].n,argv1) == 0)
     {
         // found match, chk max value_comp
      
         if (arg2 <= parameterTable[i].maxValue)
         {
           *parameterTable[i].vp = arg2;
            Serial.println("Parameter updated"); 
            return;
         }
         else
         {
            Serial.println("Value too large"); 
            return;
         }
     }  
   }  
   Serial.println("Unrecognized parameter"); 
    
   break;
   
  case HEATERC:
  
     // heater is either on or off
   
   if ( strcmp("on", argv1) == 0)
   {
     // heater on 
     digitalWrite(HEATER ,LOW);
     Serial.println("Heater On"); 
     gpioState[HEATER] = 1;
   }
   else if ( strcmp("off", argv1) == 0)
   {
     // heater off 
     digitalWrite(HEATER ,HIGH);
     Serial.println("Heater Off"); 
     gpioState[HEATER] = 0;
   }
   else  
        Serial.println("Parameter Error"); 
     break;
   
  case VALVE:
   if ( strcmp("v1", argv1) == 0)
   {
     valveId = VAC_VALVE_1;
   }
   else if ( strcmp("v2", argv1) == 0)
     {
     valveId = VAC_VALVE_2  ;
     }
   else if ( strcmp("pr", argv1) == 0)
     {
     valveId = PRESSURE_VALVE  ;
     }
   else  
     valveId = 0  ;
     
   if (  (strcmp("on", argv2) == 0) && (valveId != 0))
   {
     // valve on 
     digitalWrite(valveId ,LOW);
     Serial.println("Valve on"); 
     gpioState[valveId]  = 1;
   }
   else
     if ( (strcmp("off", argv2) == 0) && (valveId != 0))
     {
       digitalWrite(valveId ,HIGH);
       Serial.println("Valve off"); 
       gpioState[valveId]  = 0;
     }
     else  
        Serial.println("Parameter Error"); 
     break;
   
  case VTANK:
     // 
   
   if (strcmp( argv1 ,"on" ) == 0)
   {
     // heater on 
     digitalWrite(VACUUM_PUMP ,LOW);
     Serial.println("v pump On"); 
     gpioState[VACUUM_PUMP] = 1;
   }
   else if ( strcmp("off", argv1) == 0)
   {
     // heater off 
     digitalWrite(VACUUM_PUMP ,HIGH);
     Serial.println("v pump Off"); 
     gpioState[VACUUM_PUMP] = 0;
   }
   else  
        Serial.println("Parameter Error"); 
     break;
   
  
  default:
    Serial.println("Invalid Command");
    break;
  }
}


void loop()
{
  char sl;
  char sdone ;
  char buff[80];
  long currentTime;
  
  // handle any pending characters
  
  while (Serial.available() > 0) {
    
    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
        else argv3[index] = NULL;
   
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
      } else {
        argv2[index] = NULL;
        arg = 3;
        index = 0;      
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
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
      else if (arg == 3) {
        argv3[index] = chr;
        index++;
      }
    }
  }  
  
  // see uf we are diubg a grab seq
  
  if (taskState != TASK_IDLE)
  {
    currentTime = millis();
  
    switch(removeState)
    {
  case REMOVE_IDLE:
      break;

  case REMOVE_START:
  
    // turn preheat on
    digitalWrite(HEATER ,LOW);
    actionStopTime = currentTime + pot;
    Serial.println("Preheat started"); 
    removeState = REMOVE_HEATER_ON;
    
    vacuumPumpState = VPUMP_OFF;
    break;
  
  case REMOVE_HEATER_ON:
    // see if time to turn on pressure
    
    if (actionStopTime <= currentTime)
    {
       digitalWrite(PRESSURE_VALVE,LOW);
       actionStopTime = currentTime + pot;
       Serial.println("Pressure started"); 
       removeState = REMOVE_PRESSURE_ON;

      }
      break;

  case REMOVE_PRESSURE_ON:
    // see if time to turn off pressure
    
    if (actionStopTime <= currentTime)
    {
       digitalWrite(HEATER,HIGH);
       digitalWrite(PRESSURE_VALVE,HIGH);
       digitalWrite(VAC_VALVE_1,LOW);
       digitalWrite(VAC_VALVE_2,LOW);
       actionStopTime = currentTime + ato;
       Serial.println("Vacuum started"); 
       removeState = REMOVE_VACUUM_ON;
      }
      break;
    
   case REMOVE_VACUUM_ON:
      
    // attachment made?
    
    if (analogRead(GRIPPER_VACUUM) < aml)
    {
      // attachment made, turn off one vacuum valve
      
        digitalWrite(VAC_VALVE_1,LOW);
        digitalWrite(VAC_VALVE_2,HIGH);
        Serial.println("attach made"); 
        removeState = REMOVE_ATTACH_MADE;
    }
    else
      if (actionStopTime <= currentTime)
        {
           // attach timeout
           digitalWrite(VAC_VALVE_1,HIGH);
           digitalWrite(VAC_VALVE_2,HIGH);
           Serial.println("attach timeout"); 
           removeState = REMOVE_ATTACH_FAILED;
      }     break;

    case REMOVE_ATTACH_MADE:
     // hold on for awhile

       actionStopTime = currentTime + ato;
       Serial.println("Vacuum started"); 
       removeState = REMOVE_ATTACH_RELEASE;
       break;

    case REMOVE_ATTACH_FAILED:

      // attachment failed, get valves, etc back to normal status

      clearHardware();

      // make sure vac pump is on  
      
      Serial.println("Vacuum started"); 
      removeState = REMOVE_VACUUM_WAIT;
      break;

   case REMOVE_VACUUM_WAIT:

     // is vac pump still on?

     if (vacuumPumpState != VPUMP_ON)
     {
        Serial.println("Retry Ready"); 
        actionStopTime = currentTime + arw;
        removeState = REMOVE_VACUUM_WAIT;
     }
     break;
     
     case REMOVE_RETRY_WAIT:
       if (actionStopTime <= currentTime)
       {
         Serial.println("retry wait complete started"); 
         removeState = REMOVE_START;
       }
       break;
 
    case REMOVE_ATTACH_RELEASE:   
       clearHardware();
       removeState = REMOVE_IDLE;
       Serial.println("Attach Release"); 
       break;
 
    }
  }
  
  switch  (vacuumPumpState)
  {
      case VPUMP_IDLE:   
      break;
      
      case VPUMP_ON:
    
      if (analogRead(VACUUM_PUMP) > vtofl)
      {
         vacuumPumpState = VPUMP_OFF;
         digitalWrite(VACUUM_PUMP ,LOW);
         Serial.println("Vacuum pump off"); 
      }
        break;
    
      case VPUMP_OFF:
      if (analogRead(VACUUM_PUMP) > vtonl)
      {
         vacuumPumpState = VPUMP_ON;
         digitalWrite(VACUUM_PUMP ,HIGH);
         Serial.println("Vacuum pump on"); 
      }
        break;
  }
  
     
}

