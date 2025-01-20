

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Servo.h"

#define S_ENABLE   12
#define S_SLEEP    11

#define X_DIR       9
#define X_STEP     10
#define Y_DIR       8
#define Y_STEP      7

#define X_HOME     A6
#define Y_HOME     A7

#define VF_STEP    A0
#define VF_DIR     A1
#define VR_STEP    A4
#define VR_DIR     A5

#define ENABLE_LASER 13
#define CAMERA_LED   6
#define DOOR_SERVO  3   /* pwm port */
 
// unused a2 D2

////////////////////////////////////////////////////

#define X_STEPPER  0
#define Y_STEPPER  1
#define Z1_STEPPER 2
#define Z2_STEPPER 3

#define S_DIR      0
#define S_STEP     1
#define S_HOME     2

#define S_FORWARD  HIGH
#define S_REVERSE  LOW

char steppers[Z2_STEPPER+1][S_HOME+1] = { 
  {X_DIR,Y_DIR,X_HOME},
  {Y_DIR,Y_DIR,Y_HOME},
  {VF_DIR,VF_DIR,0},
  {VR_DIR,VR_DIR,0} };

#define MAX_STEPS 300

////////////////////////////////////////////////////

#define ACTIVE  'a'
#define DOOR    'd'
#define HOME    'h'
#define CAM_LED 'l'
#define MOVE    'm'
#define RESET   'r'
#define TEST    't'
#define STOP    'u'


char debugPrint = 0;
char monitoring = 0;

char inOutCount;
#define MIN_IN 4
#define MAX_OUT 4
 
char stable = 0;

Servo doorServo;
char doorOpen;

#define DOOR_OPEN 30
#define DOOR_CLOSED 110

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

 
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69

MPU6050 accelgyro;

//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
//int16_t gx, gy, gz;

int16_t tlx,tux,tly,tuy,tlz,tuz;
  
#define  MIN_ADJ 300
#define  MAX_ADJ 300
 
#define LED_PIN 13
bool blinkState = false;

void moveStepper(char direction, char stepper, int steps)
{
  int i;
  digitalWrite(S_ENABLE,LOW);
  digitalWrite(steppers[stepper][S_DIR],direction);

  for (i =0; i < steps; i++)
  {
     digitalWrite(steppers[stepper][S_STEP],HIGH);
     delay(10);
     digitalWrite(steppers[stepper][S_STEP],LOW);
     delay(10);
  }
  digitalWrite(S_ENABLE,HIGH);
}

int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  char axis,direction;
  
  arg1 = atol(argv1);
  arg2 = atol(argv2);
  arg3 = atol(argv3);
  arg4 = atol(argv4);
  char buff[80];


  switch(cmd) {

  case RESET:  
    return;
    

  case CAM_LED:  // camera LED
    if (arg1 == 1)   // led on
    {
       digitalWrite(CAM_LED, HIGH);
    }
    else
    {
      digitalWrite(CAM_LED, LOW);
    }
    break;        
    
  case DOOR:
    if (arg1 == 1)   // door open
    {
   //   digitalWrite(DOOR_PWR, LOW);
      doorOpen = 1;
      doorServo.write(DOOR_OPEN);
      Serial.println("ok-Door Open");
  //    delay(1000);
  //    digitalWrite(DOOR_PWR, HIGH);
    }
    else
    {
 //     digitalWrite(DOOR_PWR, LOW);
      doorServo.write(DOOR_CLOSED);
      doorOpen = 0;
      Serial.println("ok-Door Closed");
   //   delay(1000);
   //   digitalWrite(DOOR_PWR, HIGH);
    }
    break;    
    
  case HOME:  // home movers
    if (argv1[0] == 'x')
    {
        while (analogRead(X_HOME) > 512)
        {
           moveStepper(S_REVERSE,X_STEPPER,10);
        }
        return;
    }
    else if (argv1[0] == 'x')
    {
        // home Y direction
        
        while (analogRead(Y_HOME) > 512)
        {
           moveStepper(S_REVERSE,Y_STEPPER,10);
        }
        return;
    }
    else
       Serial.println("ER-VALUE");
    break;

  
  case MOVE:  // move x/y position
    switch (argv1[0]) {
      case 'x':
        axis = X_STEPPER;
        break;

      case 'y':
        axis = Y_STEPPER;
        break;

      case 'z':
        axis = Z1_STEPPER;
        break;

      default:
       Serial.println("ER-VALUE");
       return;
    }

    switch (argv2[0]) {
      case 'f':
        direction = S_FORWARD;
        break;

      case 'b':
        direction = S_REVERSE;
        break;

      default:
       Serial.println("ER-VALUE");
       return;
    }

    if (arg3 < MAX_STEPS)
    {
       moveStepper(direction,axis,arg3);
    }
    else
    {
        Serial.println("ER-VALUE");
        return;
    }
    Serial.println("ok");
    return;
    
  case TEST:  // imu test mode
      digitalWrite(ENABLE_LASER, LOW);
      Serial.println("test stable");     // for testing onl
      return; 
  
  case STOP:   // stop lmu monitoring
    
      monitoring = 0;
      stable = 0;
      digitalWrite(ENABLE_LASER, HIGH);        // low => active relay 
      Serial.println("mon off");
      return;
      
  case ACTIVE:   // enable lmu monitoring
  
    accelgyro.getAcceleration(&ax, &ay, &az);
    
    // set boundaries based on current position

    tlx = (ax - MIN_ADJ);
    tux = (ax + MAX_ADJ);
    tly = (ay - MIN_ADJ);
    tuy = (ay + MAX_ADJ);
    tlz = (az - MIN_ADJ);
    tuz = (az + MAX_ADJ);

    Serial.print("thrsh: ");
    Serial.print(ax); 
    Serial.print(':'); 
    Serial.print(tlx); 
    Serial.print(':'); 
    Serial.print(tux); 
    Serial.print(':'); 
    Serial.print(ay); 
    Serial.print(':'); 
    Serial.print(tly); 
    Serial.print(':');
    Serial.print(tuy); 
    Serial.print(':'); 
    Serial.print(az); 
    Serial.print(':'); 
    Serial.print(tlz); 
    Serial.print(':'); 
    Serial.println(tuz); 

    
// start monitoring
    
    monitoring = 1;
    digitalWrite(ENABLE_LASER, HIGH);
    Serial.println("mon on");
    break;
    
  default:
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

}

void setup() {
    char i;
    
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
 //   Serial.begin(9600);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // use the code below to change accel/gyro offset values
    /*
    Serial.println("Updating internal sensor offsets...");
    // -76	-2359	1688	0	0	0
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    accelgyro.setXGyroOffset(220);
    accelgyro.setYGyroOffset(76);
    accelgyro.setZGyroOffset(-85);
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    */

    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);

    // configure output to other CPU
    
    pinMode(ENABLE_LASER, OUTPUT);
    digitalWrite(ENABLE_LASER, HIGH);
    
    pinMode(CAMERA_LED, OUTPUT);
    digitalWrite(CAMERA_LED, LOW);
    
    // init pins for stepper motors
    
    pinMode(S_ENABLE,OUTPUT);
    digitalWrite(S_ENABLE,HIGH);
    
    pinMode(S_SLEEP,OUTPUT);
    digitalWrite(S_SLEEP,HIGH);

    for ( i = 0; i < Z2_STEPPER;i++)
    {
       pinMode(steppers[i][S_DIR],OUTPUT);
       digitalWrite(steppers[i][S_DIR],HIGH);

       pinMode(steppers[i][S_STEP],OUTPUT);
       digitalWrite(steppers[i][S_STEP],HIGH);

       if (steppers[i][S_HOME] != 0)
       {
          pinMode(steppers[i][S_HOME],INPUT);
       }
    }
     
    Serial.begin(9600);
    stable = 0 ;
    Serial.println("ok-Setup Complete");
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
        arg = 5;
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
      else if (arg == 4) {
        argv4[index] = chr;
        index++;
      }
    }
  }
    // read raw accel/gyro measurements from device
    //accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // these methods (and a few others) are also available

    accelgyro.getAcceleration(&ax, &ay, &az);
    
    //accelgyro.getRotation(&gx, &gy, &gz);

    // display tab-separated accel/gyro x/y/z values
    
#if 0
    Serial.print("a:\t");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.println(az); Serial.print("\t");
#endif

    if (monitoring)
    {
       if ( 
            ((ax > tlx) && (ax < tux)) &&
            ((ay > tly) && (ay < tuy)) &&
            ((az > tlz) && (az < tuz))  )
       {
          // we are okay but we require multiple readings to be good

          if (!stable)
          {
             inOutCount ++;

             if  (inOutCount == MIN_IN)
             {
               Serial.println("stable");
               digitalWrite(ENABLE_LASER, LOW);
               inOutCount = 0;
               stable = 1;  
             }
          }
       }
       else
       {
          // we have a problem but need three readings in a row to be bad

          if (stable)
          { 
             inOutCount ++;

             if (inOutCount == MAX_OUT)
             {
               Serial.println("unstable");
               inOutCount = 0;
               digitalWrite(ENABLE_LASER, HIGH);   
               stable = 0;  
               delay(2000);   // give other CPU time to notice and stop things
             }
          }
       }       
    }
 
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);

    delay(100);
    
}
