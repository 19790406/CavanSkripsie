// Arduino Pro Mini, 5V 16MHz, ATMega 328p
//install servo library
//install encoder library

#define KNEE_SERVO_PIN  9
#define HIP_SERVO_PIN   6

#define HIP_MAX_ANGLE 150
#define HIP_MIN_ANGLE 15

#define KNEE_MAX_ANGLE 175
#define KNEE_MIN_ANGLE 15

int hip_servo = 0;
int knee_servo = 1; 
int i = 0;

#define angleAmp 1.15
#define speedUpFactor 1.05
#define freq 333.0
#define loopTime 1000000.0/freq //5000     //(micro seconds)    //Gives control loop 2kHz Frequency
#define N 30     // (number of node points)
unsigned long previousTime = 0;
unsigned long evalTime = 500;

#include <Servo.h>
#include <Encoder.h>
#include "InterpolationLib.h"

Servo hip;
Servo knee; 
Encoder heightEnc(8,7); //green = A phase...... White=B phase

int hip_pos = 0;    // variable to store the servo position
int knee_pos = 0; 

long oldPosition  = -999;
long height;

double wrongtrajHM[] = {1.99999995, 1.99999992, 1.99999998, 2,         0.10000001, 0.10000001,
 0.10000001, 0.10000001, 0.79857308, 0.10000026, 0.12668791, 0.16997029,
 0.2459187,  0.39838937, 0.87793562, 0.10000001, 0.22227763, 0.19938277,
 0.20647247, 0.36171127, 0.10000692, 0.10000005, 0.10000005, 0.10000005,
 0.10000005, 0.10000005, 0.10000006, 0.10000006, 0.10000006, 0.99912479};



double trajHM[N];
double trajHip[N];
double trajKnee[N];
double newtrajHip[N];
 
//float trajTime[] = {31062014.3828914,  31062018.20260987, 31062018.93151221, 31062019.12250382, 31062019.31349592, 31062022.74208436, 31062026.56180179, 31062026.75278831, 31062030.57250667, 31062034.39222522, 31062038.21194384, 31062039.77500819, 31062039.96599422, 31062040.15698044, 31062040.34796758, 31062040.5584713,
// 31062040.81597634, 31062041.14016578, 31062041.55836494, 31062042.22613943,
// 31062043.97894351, 31062044.76479832, 31062045.31094095, 31062045.50192702,
// 31062045.69291683, 31062045.88390679, 31062046.07489705, 31062047.34802445,
// 31062047.53901061, 31062047.72999678};

 
double wrongtrajHip[] = {1.06884171, 1.09475506, 0.95155084, 0.67000152, 0.7839655,  0.8045828,
 0.82515147, 0.84571675, 0.86627646, 1.08942108, 1.10945657, 1.13295663,
 1.16189388, 1.1989536,  1.24489396, 1.24648806, 1.23750739, 1.21676924,
 1.19719253, 1.17649842, 1.14192095, 1.13281724, 1.1238303,  1.115068,
 1.10655978, 1.09833472, 1.0904217,  1.08284942, 1.07564654, 1.06884171,};
 
double wrongtrajKnee[] = {2.06942964, 1.94321547, 2.00794562, 2.12389948, 2.07865044, 2.07042128,
 2.06290906, 2.05571903, 2.0487919,  1.94235699, 1.93225152, 1.92014032,
 1.90506613, 1.88532429, 1.86084613, 1.89320403, 1.90521771, 1.93204953,
 1.9553489,  1.97808086, 2.01200683, 2.01981707, 2.02724468, 2.03438397,
 2.04120304, 2.04767081, 2.05375691, 2.05943156, 2.06466545, 2.06942964,};

 
int j;
int numHops = 2;
int hopCounter = 0;

double totalTime=0;
int d=0;

const int numValues = N;
//double hm[10] = {   5,  12,  30,  50,  60,  70,  74,  84,  92, 100 };//time
//double th1[10] = { 150, 200, 200, 200, 180, 100, 100, 150, 220, 320 };//angle 1
double hm_cumSum[30]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
 
double currentTime=0;
long EncoderHeightValue, oldEncoderHeightValue=-999;

void setup() 
{
  delay(2000);
  Serial.begin(9600);
  hip.attach(HIP_SERVO_PIN);  
  knee.attach(KNEE_SERVO_PIN);

  int a;
  int b = 0;
  for (a=0; a<N; a++)
  {
    //Serial.println(wrongtrajHM[a]); //angles are being read correctly, now check that they are being converted correctly
    //System.out.printf("%5d", arr[row][col]);
      
   trajHM[b] = (wrongtrajHM[a])/30.0;
   trajHip[b] = (wrongtrajHip[a])*180/3.141592654;
   
   trajKnee[b] = (wrongtrajKnee[a])*180/3.141592654;
 
 //Serial.println(trajHM[b]);
// Serial.println(trajHip[b]);
 //Serial.println(trajKnee[b]); //angles are being converted from radians to degrees correctly
    b++;
  }

  for(int i=0;i<numValues;i++)
  {
    trajHM[i] = trajHM[i] / speedUpFactor;  //attempt to execute motion faster, to get robot to jump
    totalTime = totalTime + trajHM[i];
    for(int j=0;j<i;j++)
    {
    
      
      hm_cumSum[i]=hm_cumSum[i]+trajHM[j];
      
    }
  }
  
//    getReady();
//    delay(1000);
}
 
void loop()
{
  
  //FollowTraj();
  double th1_command=Interpolation::Linear(hm_cumSum, trajHip, numValues, currentTime, true);
  double th2_command=Interpolation::Linear(hm_cumSum, trajKnee, numValues, currentTime, true);
  //do for th2
  currentTime=currentTime+1.0/freq;
//    Serial.print(currentTime,'\t');
    EncoderHeightValue = get_height();
    if (EncoderHeightValue != oldEncoderHeightValue){
      Serial.println(EncoderHeightValue);
      oldEncoderHeightValue =  EncoderHeightValue;
    }
//    Serial.print(EncoderHeightValue,"\t\n");
//    Serial.println(EncoderHeightValue*0.1963,'\t');
  if (currentTime>totalTime)
  {
    hopCounter = hopCounter + 1;
     currentTime = 0;
  }

  //send to servos
  if(hopCounter<numHops){
    command_servo(0,th1_command);
    command_servo(1,th2_command-th1_command+90);
  }
  else{
    Serial.print("Done hopping");
    //standStraight();
    while(true);////////////////// CALLEN (to run only once)
  }
  
  
  //command_servo(0,th1_command);
//  Serial.print(th1_command);
//  Serial.print('\t');
//  Serial.println(th2_command);

  
 
  
  //Serial.print("%lf\n",th1_command);
  wait_control_loop();  //waits 5ms if frequency set to 200Hz
//  delay(200);
}

//void loop() 
//{
  // read encoder
//  height = get_height();
  

  //sweep hip
 // sweep_servo(hip_servo,HIP_MIN_ANGLE,HIP_MAX_ANGLE,15);//////////////////////
  
  //sweep knee
  //sweep_servo(knee_servo,KNEE_MIN_ANGLE,KNEE_MAX_ANGLE,15);
//}


//void setup()
//{
  //while (!Serial) { ; }
  //Serial.begin(115200);

  //for (float xValue = 0; xValue <= 30; xValue += .25)
  //{
    
    //Serial.print(Interpolation::Linear(trajHM, trajHip, numValues, xValue, true));
    //Serial.print(',');
    
  //}
//}
//}
//
//void FollowTraj()
//{
//  int numIntervals = 3;
//  int i, c;
//  int delay_time = 5;
//  int angleHip;
//  int angleKnee;
//  int y1h, y2h, y1k, y2k;
//  int numElements = sizeof(trajHip)/sizeof(int);
//  for (i = 0; i < numElements-1; i++){
//    y1k = trajKnee[i];
//    y2k = trajKnee[i+1];
//    y1h = trajHip[i];
//    y2h = trajHip[i+1];
//    for (c=0; c < numIntervals; c++)
//    {
//      //angleKnee = y1k + (y2k-y1k)*c/numIntervals;
//      //angleHip = y1h + (y2h-y1h)*c/numIntervals;   //linear interpolation
//     // angleHip = angleHip - 16;
//      trajHip = th1_command - 16;
//      command_servo(hip_servo, th1_command);
//      //angleKnee = angleKnee-angleHip+90;
//      th2_command = th2_command-th1_command+90;
//      command_servo(knee_servo, th2_command);
//      delay(delay_time);
//    }
//  }
//}

void command_servo(int servo,int angle)
{

  if(servo==hip_servo)
    angle = (angle-90)*angleAmp+90;   //amplify the angular displacements
  if(servo==knee_servo)
    angle = (angle)*angleAmp;

if (servo == hip_servo)
    angle = angle*(165-20)/180.0 + 12;
  if (servo == knee_servo)
  {
    //angle = 194-angle*(194-42)/180;
    
//    angle = 120-angle*(194-42)/180.0; //when run this, hopping motion correct
      angle = 184-angle*(184-42)/180.0; //when run this, hopping motion not correct
  }




         
  
//  if (servo == hip_servo)
//    angle = angle*(165-20)/180 + 20;
//  if (servo == knee_servo)
//  {
//    //angle = 194-angle*(194-42)/180;
//    angle = 120-angle*(194-42)/180;
//  }
    
  if(servo==hip_servo)
  {
    if(angle>HIP_MAX_ANGLE)
    {
      hip.write(HIP_MAX_ANGLE);
    }else if(angle<HIP_MIN_ANGLE)
    {
      hip.write(HIP_MIN_ANGLE);
    } else
    {
      hip.write(angle);
    }
  } else if (servo==knee_servo)
  {
    if(angle>KNEE_MAX_ANGLE)
    {
      knee.write(KNEE_MAX_ANGLE);
    }else if(angle<KNEE_MIN_ANGLE)
    {
      knee.write(KNEE_MIN_ANGLE);
    } else
    {
      knee.write(angle);
    }
  }
}

void sweep_servo(int servo,int min_angle,int max_angle,int delay_time)
{
  int pos;
  for (pos = min_angle; pos <= max_angle; pos += 1) 
  { 
    command_servo(servo,pos);             
    delay(delay_time);                      
  }
  for (pos = max_angle; pos >= min_angle; pos -= 1) 
  { 
    command_servo(servo,pos);               
    delay(delay_time);                       
  }
}

void standStraight()
{
    command_servo(hip_servo,90);
    command_servo(knee_servo,90);
}

void getReady()
{
    command_servo(hip_servo,60);
    command_servo(knee_servo,120);
}

long get_height()
{
  long temp= heightEnc.read();
  if (temp != oldPosition) 
  {
    oldPosition = temp;
  }
  return temp;
}

void wait_control_loop()
{
  evalTime = previousTime + loopTime;
  while (micros() < evalTime)
  {
  }
  previousTime=micros();
}

//void interpolationFunc()
//{
 // #include "InterpolationLib.h"

//const int numValues = 10;
//double xValues[10] = {   5,  12,  30,  50,  60,  70,  74,  84,  92, 100 };
//double yValues[10] = { 150, 200, 200, 200, 180, 100, 100, 150, 220, 320 };
