

#include <Wire.h>
#include <MCP23017.h>
#include <math.h>
#include <Servo.h>

#include <LibPrintf.h>
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
#define ROT_TO_POSITION   'r'
#define ARM_LOCK          'a'
#define DEMO              'd'
#define POINT             'p'
#define CALIBRATE         'b'
#define SET_ANGLES        's'
#define GO_XY             'g'


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
    
    int moveTicks;
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

const MOTOR_LIMITS motorLimits[MOTORS_DEFINED] = {  {75,274,0},{80,274,0},{75,291,0} }  ;

const POSITION_SET positionSets[MOTORS_DEFINED][NUMBER_POS_SETS] = 
{
    {   
      {70,   950, float(924-850)/(90-75)},
      {90,   850, float(850.0-720)/45},
      {135,  720, float(720.0-507)/45},
      {180,  507, float(507.0-392)/45},
      {225,  392, float(392.0-129)/45},
      {270,  129, float(129.0-81)/45},
      {275,  81,  float(129.0-81)/45}
    },

     {   
      {70,   950, float(938-868)/float(90-75)},
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

double mycurrentposition[3];
double* Robot_Plan;
double RobotPlan[6] = { 0 };

#define MAX_TRAJ 60
#define NUMB_ANGLES 3

double trajectory[MAX_TRAJ][NUMB_ANGLES];
int trajCount = 0;
int trajIndex;

#define TRAJECTORY_IDLE 0
#define TRAJECTORY_ACTIVE 1

char trajectoryState = TRAJECTORY_IDLE;



 
#define plorneg(mynum) (signbit(mynum) ?  -1 : 1)

double myloc(double b1, double b2, double b3);
double* elbow(double Pef[], double l1, double l2);
double* getarmpoints(double g1, double g2, double a1, double a2, double h, double d2);
double* planangle(double mx, double my, double mz, double tm4);
double* mymatx(double k[], double x, double y);
void* attention(double currentposition[], double tm4);
void* gotopoint(double currentposition[], double tm4, double newposition[], double newtm4);
double* getcurrentwristposition(double angle1, double angle2, double angle3, double H);

using namespace std;
//*************************************test*********************************

int mechToKinDegrees(int motor,int mdegrees){

  if (motor == 0)
    return mdegrees;

  return mdegrees -180;
}

int kinToMechDegrees(int motor,int kdegrees){

  if (motor == 0)
    return kdegrees;

  return kdegrees +180;
}



//double mymatx(double k[3], double x, double y );
//*************************************test*********************************

double p[3] = { 0 };
  
double* mymatx(double k[], double x, double y) {
	//double p[]={ 10, k[1], 30 };
	//
	// double* k;


	p[0] = 10;
	p[1] = k[1];
	p[2] = 30;

	//p = {10, k[1], 30};

	return p;
	//return p[0];
}

//************************************* myloc *********************************

double myloc(double b1, double b2, double b3) {

	double my_loc_result = acos((b1*b1 + b2*b2 - b3*b3) / (2 * b1 * b2));

	return my_loc_result;
}
//************************************* getarmpoints *********************************

double armpoints[9] = {0};
  
double*  getarmpoints(double g1, double g2, double a1, double a2, double h, double d2) {
	//double* p1;
	//double* p2;
	double p1[3] = { 0 };
	double p2[3] = { 0 };
	double pi = 3.1415928;

	//Serial.println("gap " + String(a1) + " " + String(a2));

  
  p1[0] = -g1 * cos(a1 * pi / 180);
	p1[1] = g1 * sin(a1 * pi / 180);
	p1[2] = h;

	p2[0] = p1[0] - g2 * cos((a1 + a2) * pi / 180);
	p2[1] = p1[1] + g2 * sin((a1 + a2) * pi / 180);
	p2[2] = h;

	//printf("-->\n");
	//printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", g1, g2, a1, a2, h, d2);
	//printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", p1[0], p1[1], p1[2], p2[0], p2[1], p2[2]);
	//printf("-->\n");

	armpoints[0] = p1[0];
	armpoints[1] = p1[1];
	armpoints[2] = p1[2];
	armpoints[3] = p1[0] - (g2 * cos((a1 + a2) * pi / 180));  //use these if there is a heght change on wrist servo
	armpoints[4] = p1[1] + (g2 * sin((a1 + a2) * pi / 180));   //use these if there is a heght change on wrist servo
	armpoints[5] = p1[2] + d2;  //use these if there is a heght change on wrist servo
	armpoints[6] = p2[0];
	armpoints[7] = p2[1];
	armpoints[8] = p2[2];
	//=[p1(0),p1(1),p1(2),p1(0),p1(1),p1(2)+d2,p2(0),p2(1),p2(2)];
	//printf("\n");
	//printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,", armpoints[0], armpoints[1], armpoints[2], armpoints[3], armpoints[4], armpoints[5], armpoints[6], armpoints[7], armpoints[8]);
	//printf("\n");

	return armpoints;

	}

		


//************************************* elbow  *********************************






double MYDATA[2] = { 0 };

double* elbow(double Pef[], double l1, double l2) {

	//function MYDATA = elbow(Pef,l1,l2)

	//#Pref is the base reference point and is 0, 0, 0
		//# Pef is the location of the wrist in x, y, z
		//# Pefdelta is the offset from the shelf of the robot to the end of the
		//# second arm segment or middle of rotation for the robot servo
		//#elbo always points to the right for a left handed arm

	double T[3] = { 0 };
	double pi = 3.1415928;

	double x_arm = 0;
	double y_arm = 0;
	double z_arm = 0;

	double x_elbow = Pef[0];
	double y_elbow = Pef[1];
	double z_elbow = Pef[2];

	double arm_length = l1;
	double forearm_length = l2;

	double myzeta = 0;
	double k = pi/180;
	double rz = 0;
	double gumma3, myt1;

	if (x_elbow >= 0 && y_elbow >= 0) 
	{
		if (abs(x_elbow) <= 0.0001) 
		{
			gumma3 = 90 * pi / 180;
		}
		else 
		{
			gumma3 = atan(y_elbow / x_elbow);
		// servo rotates cw from -x axis
		}
		
		rz = sqrt(x_elbow * x_elbow + y_elbow * y_elbow);

		myzeta = myloc(arm_length, rz, forearm_length);//........................
		myt1 = pi - (gumma3 + myzeta);
		//servo rotates cw from -x axis
		//loopme=1;
	}
	
	if (x_elbow < 0 && y_elbow >= 0) 
	{
		if (abs(x_elbow) <= 0.0001) 
		{
				gumma3 = 90 * pi / 180;
		}
		else
		{
			gumma3 = atan(y_elbow / abs(x_elbow));
		}
	
		#//servo rotates cw from -x axis
		rz = sqrt(x_elbow*x_elbow + y_elbow*y_elbow);
		myzeta = myloc(arm_length, rz, forearm_length); //........................
		myt1 = (gumma3 - myzeta);
		// servo rotates cw from -x axis
		//loopme=2;

	}

	if (x_elbow <= 0 && y_elbow < 0) 
	{
		if (abs(x_elbow) <= 0.0001)
		{
			gumma3 = 90 * pi / 180;
		}
		else
		{	gumma3 = atan(abs(y_elbow) / abs(x_elbow));
		}
		// servo rotates cw from -x axis
		rz = sqrt(x_elbow*x_elbow + y_elbow*y_elbow);
		myzeta = myloc(arm_length, rz, forearm_length); //........................
		myt1 = -(gumma3 + myzeta);
		// servo rotates cw from -x axis
		//loopme=3;
	}

	if (x_elbow >= 0 && y_elbow < 0) 
	{
		if (abs(x_elbow) <= 0.0001)
		{
			gumma3 = 90 * pi / 180;
		}
		else
		{
			
			gumma3 = atan(abs(y_elbow) / abs(x_elbow));
		}
		// servo rotates cw from -x axis
		rz = sqrt(x_elbow*x_elbow + y_elbow*y_elbow);
		myzeta = myloc(arm_length, rz, forearm_length);//........................
		myt1 = 180 * k - (-gumma3 + myzeta);
		// servo rotates cw from -x axis
		//loopme=4;

	}

	double my_arm_angle = myt1 / k;

	//my_elbow_angle = (pi()-myzeta)/k;
	double my_elbow_angle = 180 - myloc(arm_length, forearm_length, rz) / k;
	MYDATA[0] = my_arm_angle;
	MYDATA[1] = my_elbow_angle;

	return MYDATA;

	//return (MYDATA[0], MYDATA[1]);

}

//************************************* planangle *********************************

double MYPLAN[6] = {0};

double* planangle(double mx, double my, double mz, double tm4) {
	double PELB[3] = { 0 };
	double* T;
	double* G;
	double GG[3] = { 0 };
	double pi = 3.1415928;
	//double G[9] = {0};
	double E[3] = {0};
	//double* PEF;
	double PEF[3] = { 0 };
	//double* MYPLAN;
	double H;
	//function MyPlan = PlanAngle(mx, my, mz, tm4, frame_index)
	//%all numbers are in inches and degrees
	double l1 = 9; 	//%length of arm in inches
	double l2 = 9; 	//%length of forearm in inches
	double l00 = 4;	//% distance from the rotation point of wrist to robot mount position in inches
	//% robot

double dservo = 0; //%height of elbo servo from arm to forearm
//%initial position and is actually zero in bobs design
double t1 = 0; 	//%sholder angle
double t2 = 0;	//%elbow angle

H = mz; //%height from base
//%prompt1 = "wrist position [x y z] ";
//%PELB = input(prompt1);
PELB[0] = mx;
PELB[1] = my;
PELB[2] = mz;

H = PELB[2];
//%prompt2 = "hand angle (in degrees from x axis)";
//%wrist_ang = input(prompt2);
double wrist_ang = tm4-180;
//%PELB= [9 8 H]; %should match GG below.  this is where the wrist rotation 
//% point is located
double reach_test = sqrt(PELB[0]*PELB[0] + PELB[1]*PELB[1]); //% test for max reach and prevent 
//% code lockup
	if (reach_test > (l1 + l2)) 
		{
			PELB[0] = l1;
			PELB[1] = l2;
			T = elbow(PELB, l1, l2);
		}
	else 
		{
			T = elbow(PELB, l1, l2); //%computes the angles of the arm
		}

//% these are the rotation angles used to move the arm into position

t1 = T[0]; //% these come from the elbow function
t2 = T[1];

if (t1 < -20)
  t1 = -20;

if (t1 > 180)
  t1 = 180;

double rot_ang_of_wrist = wrist_ang * pi / 180;  //%wrist angle in degrees from 
//% base as manual input- add vehicle angle to this to square off the robot

//% rot_ang_of_wrist = (t1+t2+0)*pi()/180; %wrist angle in degrees 
//% from forearm

G = getarmpoints(l1, l2, t1, t2, H, dservo); //%compute xyz of wrist position
//printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,", G[0], G[1], G[2], G[3], G[4], G[5], G[6], G[7], G[8]);
//printf("\n");

GG[0] = G[6];
GG[1] = G[7];
GG[2] = G[8];

double hand_x = G[6] + l00 * sin(rot_ang_of_wrist);
double hand_y = G[7] + l00 * cos(rot_ang_of_wrist);

E[0] = 0;
E[1] = 0;
E[2] = 0;
//****************************************   Need to fix this with Function to reference the default position
PEF[0] = 5;
PEF[1] = 0;
PEF[2] = 1;


double t1c = t1 + 90;
double m1out = t1c;
if (abs(m1out) < 0.0001) {
	m1out = 0;
}

//******************* m2out ***********************
//m2=num2str(t2);
double m2out = t2;
if (abs(m2out) < 0.0001) {
	m2out = 0;
}
//******************* m3out ***********************

//ms="   ";
//m_hand = num2str(rot_ang_of_wrist*180/pi());
double m3out = (rot_ang_of_wrist * 180 / pi) + 180;
if (abs(m3out) < 0.0001) {
	m3out = 0;
}

//******************* m4out ***********************
//robot_center_x = num2str(hand_x);
//robot_center_y = num2str(hand_y);
//robot_center_z = num2str(H);
double m4out = hand_x;
if (abs(m4out) < 0.0001) {
	m4out = 0;
}

//******************* m5out ***********************
double m5out = hand_y;
if (abs(m5out) < 0.0001) {
	m5out = 0;
}

//******************* m6out ***********************
double m6out = H;
if (abs(m6out) < 0.0001) {
	m6out = 0;
}


//%strcat(m1,ms,m2,ms,m3,ms,m4,ms,m5)
//strcat(m1,ms,m2,ms,m_hand,ms, robot_center_x,ms,robot_center_y,ms, ...
 //   robot_center_z);
//MyPlan = [m1out m2out m3out m4out m5out m6out];
MYPLAN[0] = m1out;
MYPLAN[1] = m2out;
MYPLAN[2] = m3out;
MYPLAN[3] = m4out;
MYPLAN[4] = m5out;
MYPLAN[5] = m6out;


return MYPLAN;


	}

	//************************************* attention *********************************

void* attention(double currentposition[], double tm4) {
	//function [Attn_Position] = Attention(Current_Position, tm4, frame_index)
	//%UNTITLED8 Summary of this function goes here
	//%   Detailed explanation goes here
	//#define plorneg(mynum) (signbit(mynum) ?  -1 : 1) *************need to add to main after includes
	double *Robot_Plan;
	double RobotPlan[6] = { 0 };
	double attn_ms[3] = { 0, 5, 10 };//% attention position
	//double* attn_ms;
    double attn_tm = 90;
	int n;
	double Curr_msx = currentposition[0];
	double Curr_msy = currentposition[1];
	double Curr_msz = currentposition[2];;
	double Curr_tm = tm4;

	double del_x = (attn_ms[0] - Curr_msx);
	double del_y = (attn_ms[1] - Curr_msy);
	double del_z = (attn_ms[2] - Curr_msz);
	double del_tm = (attn_tm - Curr_tm) / 5; //increments of 5 degrees but can be dynamic for out application


	//currently everything is moved in increments of 1 inch  or 1 degree
	for (n = 1; n<=abs(int(del_z)); ++n) {
		Robot_Plan = planangle(Curr_msx, Curr_msy, Curr_msz + n * (signbit(del_z) ? -1 : 1), tm4);
		//printf("delz ->%d ; ", abs(int(del_z)));
//		printf("%.2f, %.2f, %.2f, %.2f, %.2f, %.2f", Robot_Plan[0], Robot_Plan[1], Robot_Plan[2], Robot_Plan[3], Robot_Plan[4], Robot_Plan[5]);
//		printf("\n");
	};
	// Z is now at attention position
	for (n = 1; n<=abs(int(del_x)); ++n) {
		//printf("delx ->%d ; ", abs(int(del_x)));
		Robot_Plan = planangle(Curr_msx + n * (signbit(del_x) ? -1 : 1), Curr_msy, attn_ms[2], tm4);
		//printf("%d,%.2f, %.2f, %.2f, %.2f, %.2f,%.2f", abs(int(del_x)), Robot_Plan[0], Robot_Plan[1], Robot_Plan[2], Robot_Plan[3], Robot_Plan[4], Robot_Plan[5]);
//		printf("%.2f, %.2f, %.2f, %.2f, %.2f, %.2f", Robot_Plan[0], Robot_Plan[1], Robot_Plan[2], Robot_Plan[3], Robot_Plan[4], Robot_Plan[5]);
//		printf("\n");
	};
	//x is now at attention position
	for (n = 1; n<=abs(int(del_y)); ++n) {
		//printf("dely ->%d ; ", abs(int(del_y)));
		Robot_Plan = planangle(attn_ms[0], Curr_msy + n * (signbit(del_y) ? -1 : 1), attn_ms[2], tm4);
		//printf("%d,%.2f, %.2f, %.2f, %.2f, %.2f,%.2f", abs(int(del_y)), Robot_Plan[0], Robot_Plan[1], Robot_Plan[2], Robot_Plan[3], Robot_Plan[4], Robot_Plan[5]);
//		printf("%.2f, %.2f, %.2f, %.2f, %.2f, %.2f", Robot_Plan[0], Robot_Plan[1], Robot_Plan[2], Robot_Plan[3], Robot_Plan[4], Robot_Plan[5]);
//		printf("\n");
	};
	//%y is now at attention position
	for (n = 1; n<=abs(int(del_tm)); ++n) {
		//printf("deltm ->%d ;", abs(int(del_tm)));
		Robot_Plan = planangle(attn_ms[0], attn_ms[1], attn_ms[2], Curr_tm + (5 * n * (signbit(del_tm) ? -1 : 1)));
		//printf("%d,%.2f, %.2f, %.2f, %.2f, %.2f,%.2f", abs(int(del_tm)), Robot_Plan[0], Robot_Plan[1], Robot_Plan[2], Robot_Plan[3], Robot_Plan[4], Robot_Plan[5]);
//		printf("%.2f, %.2f, %.2f, %.2f, %.2f, %.2f", Robot_Plan[0], Robot_Plan[1], Robot_Plan[2], Robot_Plan[3], Robot_Plan[4], Robot_Plan[5]);
//		printf("\n");
	};
	return 0;
}

//*********************************************** gotopoint ******************************************
//function[Planned_Angles] = gotopoint(Current_Position, tm4, New_Position, new_tm4, frame_index)
//% UNTITLED8 Summary of this function goes here
//

double limitMovement(int m, double inp)
{
   double t = max(double(motorLimits[m].lowerLimit),inp);
   
   t = min(double(motorLimits[m].upperLimit),t);
   return t;

}

void addTrajectoryPoint(double as,double ae,double aw)
{
  if (trajCount < MAX_TRAJ)
  {
    trajectory[trajCount][0]= as+90 ; //limitMovement(0,as+90);
    trajectory[trajCount][1]= ae;  //limitMovement(1,ae);
    trajectory[trajCount][2]= aw;  //limitMovement(2,aw);  
    trajCount++;
    return;

    if (trajCount == 0)
       trajCount++;
    else
      if (  (trajectory[trajCount-1][0] != trajectory[trajCount][0]) ||
            (trajectory[trajCount-1][1] != trajectory[trajCount][1]) ||
            (trajectory[trajCount-1][2] != trajectory[trajCount][2]) )
             trajCount++;   
  }
}

void* gotopoint(double currentposition[], double tm4, double newposition[], double newtm4)
{

	double Curr_msx = currentposition[0];
	double Curr_msy = currentposition[1];
	double Curr_msz = currentposition[2];
	double Curr_tm = tm4;


	double New_msx = newposition[0];
	double New_msy = newposition[1];
	double New_msz = newposition[2];
	double New_tm = newtm4;


	double del_x = (New_msx - Curr_msx);
	double del_y = (New_msy - Curr_msy);
	double del_z = (New_msz - Curr_msz);
	double del_tm = (New_tm - Curr_tm) / 5;

  char b[100];

	int n;
	//frame_index = 1;

#ifdef False
	for (n = 1; n <= abs(int(del_z)); ++n) {
		Robot_Plan = planangle(Curr_msx, Curr_msy, Curr_msz + n * (signbit(del_z) ? -1 : 1), tm4);
		addTrajectoryPoint(Robot_Plan[0], Robot_Plan[1], Robot_Plan[2]);

    //printf("delz ->%d ; ", abs(int(del_z)));
		//sprintf(b,"1- %.2f, %.2f, %.2f, %.2f, %.2f, %.2f", Robot_Plan[0], Robot_Plan[1], Robot_Plan[2], Robot_Plan[3], Robot_Plan[4], Robot_Plan[5]);
		//Serial.println(b);
	};
#endif

	// Z is now at attention position
	for (n = 1; n <= abs(int(del_x)); ++n) {
		//printf("delx ->%d ; ", abs(int(del_x)));
		Robot_Plan = planangle(Curr_msx + n * (signbit(del_x) ? -1 : 1), Curr_msy, New_msz, tm4);
		addTrajectoryPoint(Robot_Plan[0], Robot_Plan[1], Robot_Plan[2]);
		//printf("%d,%.2f, %.2f, %.2f, %.2f, %.2f,%.2f", abs(int(del_x)), Robot_Plan[0], Robot_Plan[1], Robot_Plan[2], Robot_Plan[3], Robot_Plan[4], Robot_Plan[5]);
		//sprintf(b,"2- %.2f, %.2f, %.2f, %.2f, %.2f, %.2f", Robot_Plan[0], Robot_Plan[1], Robot_Plan[2], Robot_Plan[3], Robot_Plan[4], Robot_Plan[5]);
		//Serial.println(b);
	};
	//x is now at attention position
	for (n = 1; n <= abs(int(del_y)); ++n) {
		//printf("dely ->%d ; ", abs(int(del_y)));
		Robot_Plan = planangle(New_msx, Curr_msy + n * (signbit(del_y) ? -1 : 1), New_msz, tm4);
		addTrajectoryPoint(Robot_Plan[0], Robot_Plan[1], Robot_Plan[2]);
		//printf("%d,%.2f, %.2f, %.2f, %.2f, %.2f,%.2f", abs(int(del_y)), Robot_Plan[0], Robot_Plan[1], Robot_Plan[2], Robot_Plan[3], Robot_Plan[4], Robot_Plan[5]);
		//sprintf(b,"3- %.2f, %.2f, %.2f, %.2f, %.2f, %.2f", Robot_Plan[0], Robot_Plan[1], Robot_Plan[2], Robot_Plan[3], Robot_Plan[4], Robot_Plan[5]);
		//Serial.println(b);
	};
	//%y is now at attention position
	for (n = 1; n <= abs(int(del_tm)); ++n) {
		//printf("deltm ->%d ;", abs(int(del_tm)));
		Robot_Plan = planangle(New_msx, New_msy, New_msz, Curr_tm + (5 * n * (signbit(del_tm) ? -1 : 1)));
		addTrajectoryPoint(Robot_Plan[0], Robot_Plan[1], Robot_Plan[2]);
		//printf("%d,%.2f, %.2f, %.2f, %.2f, %.2f,%.2f", abs(int(del_tm)), Robot_Plan[0], Robot_Plan[1], Robot_Plan[2], Robot_Plan[3], Robot_Plan[4], Robot_Plan[5]);
		//sprintf(b,"4- %.2f, %.2f, %.2f, %.2f, %.2f, %.2f", Robot_Plan[0], Robot_Plan[1], Robot_Plan[2], Robot_Plan[3], Robot_Plan[4], Robot_Plan[5]);
		//Serial.println(b);
	};
	return 0;
	
	
}

double gotmyposition[3] = { 0 };

double* getcurrentwristposition(double angle1, double angle2, double angle3, double H) {
	double* myposition;
	//double myposition[9] = { 0 };
	double l00 = 4;
	double pi = 3.1415928;

	myposition = getarmpoints(9, 9, angle1, angle2, H, 0); // 9 and 9 are the length of the bicept and arm and needs to be gloalized.

	gotmyposition[0] = myposition[6]; //+l00 * cos(angle3 * pi / 180);
	gotmyposition[1] = myposition[7]; //+l00 * sin(angle3 * pi / 180);
	//gotmyposition[0] = l00 * cos(angle3 * pi / 180);
	//gotmyposition[1] = l00 * sin(angle3 * pi / 180);
	gotmyposition[2] = myposition[8];

	return gotmyposition;
};


POSITION_SET * getPositionSetbyDegree(int motor,int degrees, char increasing)
{
  int i;

  if (increasing)  // look for highest
  {
    for (i = 0; i < NUMBER_POS_SETS; i++)
    {
//      Serial.println("Dposition increasing");
//      Serial.println("Dposition itst :" + String(degrees) + " " + String( positionSets[motor][i].degree));
      if (degrees < positionSets[motor][i].degree){
//        Serial.println("Dposition set :" + String(i));
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
//        Serial.println("Dposition set :" + String(i+1));
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
 //     Serial.println("Aposition itst :" + String(i) + " " + String( positionSets[motor][i].degree) + " " + String( positionSets[motor][i].av));

      if (av < positionSets[motor][i].av){
  //      Serial.println("Aposition set :" + String(positionSets[motor][i].degree));
        return &positionSets[motor][i-1];
      }
    }
  }
  else
  {
    for (i = NUMBER_POS_SETS-1; i >= 0; i--) 
    {
  //    Serial.println("Aposition dtst :" + String(i) + " " + String( positionSets[motor][i].degree) + " " + String( positionSets[motor][i].av));
     if (av < positionSets[motor][i].av)
      {
  //      Serial.println("Aposition set :" + String(positionSets[motor][i].degree));
       
        return &positionSets[motor][i];
      }
    } 
  }
  Serial.println("Aposition failure :" + String(i));
}

ISR(TCA0_OVF_vect) {
  int m;

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
      if (motors[m].ticksSkip < 0)
        Serial.println("negative ticksSkip ");
       
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
             Serial.println("CW2-done " + String(m) + ":"+ String(motors[m].ticksLeft) + " " + String(motors[m].currentAv) + ":" + String( motors[m].targetAv));
             continue;
          }
        } 
        else
        {
          if (motors[m].currentAv < motors[m].targetAv)
          {
             // we are done
             motors[m].stopFlag = 1;
             Serial.println("CW01-Done " + String(m) + ":" + String(motors[m].ticksLeft) + " " + String(motors[m].currentAv) + ":" + String( motors[m].targetAv));
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
             Serial.println("CCW2-Done  " + String(m) + ":" + String(motors[m].ticksLeft) + " " + String(motors[m].currentAv) + ":" + String( motors[m].targetAv));
             continue;
          }
        }
        else
        {
          if (motors[m].currentAv >= motors[m].targetAv) 
          {
             // we are done
             motors[m].stopFlag = 1;
             Serial.println("CCw-01-Done " + String(m) + ":" + String(motors[m].ticksLeft) + " " + String(motors[m].currentAv) + ":" + String( motors[m].targetAv));
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
          
            Serial.println("X5 " + String(m) + ":" + String(motors[m].currentAv) + ":" + String( motors[m].targetAv));
           
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

int * getCurrentPosition(int motor )
{
      POSITION_SET * pset;
      int currentPosition;
      motors[motor].currentAv = analogRead(motors[motor].positionAnalogPin);
      Serial.println("Av " + String(motors[motor].currentAv)); 
         
      if (motor == 2)
      {
         // get current degree position from currentAv
           
         pset = getPositionSetbyAv(motor,  motors[motor].currentAv, true);
         currentPosition = pset->degree +  (motors[motor].currentAv - pset->av) / pset->aVPerDegree;
      }
      else 
      {
        pset = getPositionSetbyAv(motor,  motors[motor].currentAv, false);   
        currentPosition = pset->degree + ( pset->av - motors[motor].currentAv) / pset->aVPerDegree;     
      }     
      return currentPosition;
}

POSITION_SET * getTargetMovement(int motor,int newPosition,int *currentPosition)
{
      POSITION_SET * pset;
      int increasingPosition, t;

      motors[motor].currentAv = analogRead(motors[motor].positionAnalogPin);

      Serial.println("Av " + String(motors[motor].currentAv)); 
         
      if (motor == 2)
      {
           // get current degree position from currentAv
           
           pset = getPositionSetbyAv(motor,  motors[motor].currentAv, true);
           *currentPosition = pset->degree +  (motors[motor].currentAv - pset->av) / pset->aVPerDegree;
           
          increasingPosition =  (newPosition > *currentPosition ? true : false);
          pset = getPositionSetbyDegree(motor,  newPosition,increasingPosition);
       }
      else 
      {
          pset = getPositionSetbyAv(motor,  motors[motor].currentAv, false);         
          *currentPosition = pset->degree + ( pset->av - motors[motor].currentAv) / pset->aVPerDegree;

          increasingPosition =  (newPosition > *currentPosition ? true : false);           
          pset = getPositionSetbyDegree(motor,  newPosition,!increasingPosition);
        }     
        return pset;
}


int moveToPosition(int motor,int newPosition,int start)
{
    int moveAv,moveTicks,increasingPosition,currentPosition;
     POSITION_SET * pset;

       Serial.println("mtp: " + String(motor) + " " + String(newPosition) );

        if (newPosition > motorLimits[motor].upperLimit )
        {
          Serial.println ("ERR - too far: " + String(motor) + " " + String(newPosition));
          newPosition = motorLimits[motor].upperLimit-4 ;
        }
        
        if (newPosition < motorLimits[motor].lowerLimit )
        {
          Serial.println ("ERR - too close: " + String(motor) + " " + String(newPosition));
          newPosition = motorLimits[motor].lowerLimit +4 ;
        }

        pset = getTargetMovement(motor,newPosition,&currentPosition);

        Serial.println("-cp " +  String(motors[motor].currentAv) + " " + String(currentPosition) );
         
         // we have info to calc targetAv
        
        Serial.println("--- " + String(pset->av) + " " + String(pset->degree) + " " + String(pset->aVPerDegree));
        
        motors[motor].targetAv = pset->av + (newPosition - pset->degree) * pset->aVPerDegree;
                
        moveAv = motors[motor].targetAv - motors[motor].currentAv;               
        motors[motor].moveTicks = moveAv * motors[motor].avPerDegree;
        
        Serial.println("--- " + String(motors[motor].targetAv) + " " + String(moveAv) + " " + String(motors[motor].moveTicks));
 
        if (!start)
          return 1;
          
        // start all three motors now
        
        for (int m = 0 ; m < MOTORS_DEFINED; m++)
        {
          if (m == 2)
          {
            startMotor(m,(motors[m].moveTicks >= 0 ? ROT_CW : ROT_CCW),abs(motors[m].moveTicks)+1000,0);
          }
          else
          {
             startMotor(m,(motors[m].moveTicks < 0 ? ROT_CW : ROT_CCW),abs(motors[m].moveTicks) + 1000,0);
          }
        }

  return 1;
}

void startMotor(int motor, char cmd,int ticks, int repeat)
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
  int currentPosition[3];
  double* hereismyposition;
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
  
    trajectoryState = TRAJECTORY_IDLE;
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
      
    trajectoryState = TRAJECTORY_IDLE;
 
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
   trajectoryState = TRAJECTORY_IDLE;

    if (arg1 < MOTORS_DEFINED)
      {
         moveToPosition(arg1,arg2, (arg1 ==2 ? 1 : 0));
         Serial.println("OK");
      }
      else 
         Serial.println("ERR");
      break;
    
case DEMO:
    trajectoryState = TRAJECTORY_IDLE;
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

  case GO_XY:
    {
  	   double mynewposition[3];

       mynewposition[0] = arg1;
       mynewposition[1] = arg2;
       mynewposition[2] = 48;
       
       Serial.println("target: (" + String(mynewposition[0]) + ',' + String(mynewposition[1]) + ','+ String(mynewposition[2]) + ')');

      trajCount = 0;

  	  gotopoint(mycurrentposition, 0, mynewposition, arg3);

      //for (int i = 0; i < trajCount; i++)
      //  Serial.println("--" + String(i) + " (" + String(trajectory[i][0]) + "," + String(trajectory[i][1])+ "," + String(trajectory[i][2]) + ")" );

      Serial.println("OK");
    }
    break;


  case SET_ANGLES:
    // set all three angles
    
    Serial.println("***target: (" + String(arg1) + ',' + String(arg2) + ','+ String(arg3) + ')');

    if (moveToPosition(0,arg1,0) == 0)
    {
      Serial.println("ERR 0");
      break;
    }
    
    if (moveToPosition(1,arg2,0) == 0)
    {
      Serial.println("ERR 1");
      break;
    }
 
    if (moveToPosition(2,arg3,1) == 0)
    {
      Serial.println("ERR 2");
      break;
    }
    Serial.println("OK");
    break;

  case LOCATION:
     if (argv1[0] == 'a')
    {
      Serial.print("OK: " + String(motors[0].position) );
      Serial.print(":"+ String(motors[0].position/motors[0].ticksPerDegree) );
      Serial.print("  "    + String(motors[1].position)) ;
      Serial.print(":" + String(motors[1].position/motors[1].ticksPerDegree) );
      Serial.print("  "  + String(motors[2].position));
      Serial.println(":"+ String(motors[2].position/motors[2].ticksPerDegree) );
    }
    else
      if (argv1[0] == 'p')
      {
         // first get current positionpset
         
         for (char i = 0; i < 3; i++)
         {
             currentPosition[i] = getCurrentPosition(i);
             Serial.println(currentPosition[i]);
         }
    
         // convert to x,y,rot

	       hereismyposition = getcurrentwristposition(double(currentPosition[0]-90), double(mechToKinDegrees(1,currentPosition[1])), double(mechToKinDegrees(2,currentPosition[2])), double(48));  //these angles come from the spreadsheet in MatLab

         mycurrentposition[0] = hereismyposition[0] ;
         mycurrentposition[1] = hereismyposition[1];
	       mycurrentposition[2] = hereismyposition[2];

         Serial.println(  "OK + (" + String(hereismyposition[0]) + ',' + String(hereismyposition[1]) + ',' + String(hereismyposition[2])  + ',' + String(hereismyposition[3]) + ")"  );
         
      }
      break;

  case POINT:
    trajectoryState = TRAJECTORY_IDLE;
    if (argv1[0] == 'g')
    {
      trajIndex = 0;
      
      
      trajectoryState = TRAJECTORY_ACTIVE;
      for (m = 0; m < MOTORS_DEFINED; m++)
        moveToPosition(m, trajectory[0][m], (m == 2 ? 1 : 0));            
    }  
    else if (argv1[0] == 'l')
      {
        trajIndex = 0;
        for (m = 0; m < trajCount; m++)
        {
          Serial.print("(" + String(trajectory[m][0]) ); 
          Serial.print("," + String(trajectory[m][1]) ); 
          Serial.println("," + String(trajectory[m][2]) + ")" ); 
        }                
      } 
 
    Serial.println("OK");
    break;


  case CALIBRATE:
   trajectoryState = TRAJECTORY_IDLE;
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
    int currentPosition[3];
  
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

    avReadMode = MOTORS_DEFINED+1;
    calState = CAL_IDLE;

    // get the current location
    
    for (char i = 0; i < 3; i++)
      currentPosition[i] = getCurrentPosition(i);
          
    // convert to x,y,rot

	  double *hereismyposition = getcurrentwristposition(double(currentPosition[0]-90), double(mechToKinDegrees(1,currentPosition[1])), double(mechToKinDegrees(2,currentPosition[2])), double(48));  //these angles come from the spreadsheet in MatLab
   
    mycurrentposition[0] = hereismyposition[0] ;
    mycurrentposition[1] = hereismyposition[1];
	  mycurrentposition[2] = hereismyposition[2];

    Serial.println(  "OK- Setup Done (" + String(hereismyposition[0]) + ',' + String(hereismyposition[1]) + ',' + String(hereismyposition[2])  + ',' + String(hereismyposition[3]) + ")"  );

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
    for (int m=0; m < MOTORS_DEFINED;m++)
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
   
    for (int  m=0; m < MOTORS_DEFINED;m++)
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

  if (trajectoryState == TRAJECTORY_ACTIVE)
  {
    // we are doing point move, we go on to the next point when all three motors are stopped

    if (motors[0].stopFlag && motors[1].stopFlag && motors[2].stopFlag )
    { 
       // move on to the next point if there is one
  
      trajIndex++;
      
      if (trajIndex < trajCount)
       {
          // got another point
         Serial.print("next point ");
         Serial.println(trajIndex);
         Serial.println( "(" + String(trajectory[trajIndex][0]) + "," +String(trajectory[trajIndex][1]) + "," +String(trajectory[trajIndex][2]) + ")"  );

          for (int m = 0; m < MOTORS_DEFINED; m++)
            moveToPosition(m, trajectory[trajIndex][m], (m ==2 ? 1 : 0));            
       }
       else
       {
         trajectoryState = TRAJECTORY_IDLE;
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
