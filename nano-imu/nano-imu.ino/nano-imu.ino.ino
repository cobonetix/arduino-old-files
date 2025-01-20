

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project


#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

#define ENABLE_LASER 4
#define STABLE_NOTIFY  5

char debugPrint = 0;
char monitoring = 0;

char inOutCount;
#define MIN_IN 4
#define MAX_OUT 4
 
char stable = 0;


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
char argv5[8];
char argv6[8];

// The arguments h to integers
long arg1;
long arg2;
long arg3;
long arg4;
long arg5;
long arg6;

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h


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

int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
/*  
 *   
  arg1 = atol(argv1);
  arg2 = atol(argv2);
  arg3 = atol(argv3);
  arg4 = atol(argv4);
  arg5 = atol(argv5);
  arg6 = atol(argv6);
  char buff[80];
*/

  switch(cmd) {

  case 't':
      digitalWrite(ENABLE_LASER, LOW);
      digitalWrite(STABLE_NOTIFY, HIGH);
      Serial.println("test stable");     // for testing onl
      return 0; 
  
  case 'u':
    
      monitoring = 0;
      stable = 0;
      digitalWrite(ENABLE_LASER, HIGH);        // low => active relay 
      digitalWrite(STABLE_NOTIFY, LOW);
      Serial.println("mon off");
      return 0;
      
  case 'l':
  
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
    digitalWrite(STABLE_NOTIFY, LOW);
    Serial.println("mon on");
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
  memset(argv5, 0, sizeof(argv5));
  memset(argv6, 0, sizeof(argv6));
  idx = 0;

}

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Serial.begin(9600);
    Serial.println("start setup");
    
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    Serial.println("start 2");


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
    
    pinMode(STABLE_NOTIFY, OUTPUT);
    digitalWrite(STABLE_NOTIFY, LOW);
    
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
      if (arg == 1) argv1[idx] = NULL;
      else if (arg == 2) argv2[idx] = NULL;
      else if (arg == 3) argv3[idx] = NULL;
      else if (arg == 4) argv4[idx] = NULL;
      else if (arg == 5) argv5[idx] = NULL;
      else argv6[idx] = NULL;
      
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
      } else if (arg == 5)  {
        argv5[idx] = NULL;
        arg = 6;
        idx = 0;
        } else {
        argv6[idx] = NULL;
        arg = 7;
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
      else if (arg == 5) {
        argv5[idx] = chr;
        idx++;
      }
      else if (arg == 6) {
        argv6[idx] = chr;
        idx++;
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
               digitalWrite(STABLE_NOTIFY, HIGH);
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
               digitalWrite(STABLE_NOTIFY, LOW);
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
