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
#define speedUpFactor 4.0
#define freq 330.0
#define loopTime 1000000.0/freq //5000     //(micro seconds)    //Gives control loop 2kHz Frequency
#define N 30     // (number of node points)
unsigned long previousTime = 0;
unsigned long evalTime = 500;

#include <Servo.h>
#include <Encoder.h>
#include "InterpolationLib.h"



Servo hip;
Servo knee; 
//Encoder heightEnc(8,7); //green = A phase...... White=B phase int long
Encoder myEnc(7, 8);



int numHops = 1;
int hip_pos = 0;    // variable to store the servo position
int knee_pos = 0; 

long oldPosition  = -999;
long height;



double trajHM[N];
double trajHip[N];
double trajKnee[N];
double newtrajHip[N];

double wrongtrajHip[] = {89.89937272283271, 89.99999239901824, 89.97988162021241, 89.95910159893427, 89.92694207799428, 89.86848507853847, 89.74186513740402, 89.81169501957763, 89.91081228910411, 89.99980656839804, 89.9999875043957, 89.80615694271268, 89.6017701799501, 87.06677862480888, 86.79666760576566, 86.20705156393991, 78.87765445327801, 65.082599677591, 49.90342033563371, 36.169969484092164, 30.219746943518462, 30.0000009649714, 45.202195038671654, 56.782958641095085, 58.34963722644863, 60.53122806553492, 63.47635393283295, 67.14431233289713, 78.7278719326137, 89.89937272283271};
double wrongtrajKnee[] = {90.14894080379415, 90.15499796269471, 90.16511191913038, 90.17594949720466, 90.20914418855655, 90.31952419550677, 90.61805487886124, 93.89971868285306, 93.86747327482668, 93.88215778772259, 93.88217709830164, 94.00191921283997, 94.16638801679318, 98.39911164404622, 98.5414551222838, 98.87147001931656, 104.4043777607837, 111.8637815337298, 119.57560527099002, 125.66373950771829, 128.0355617596424, 128.1111801596421, 122.21281432460327, 116.80792869258205, 116.02561247438173, 114.91288478555853, 113.3707690536046, 111.45262523123137, 99.98191685046656, 90.14894080379415};
double wrongtrajHM[] = {1.9999941566827824, 0.100092649547896, 0.10011817674787847, 0.1001479465300243, 0.14518543051692803, 0.2767185627171569, 1.9999782598327276, 0.10001129398374356, 0.10001264256508528, 0.10000866134859542, 0.10001154474059323, 0.10001822030158725, 1.9999563596401102, 0.10000241370669896, 0.10000242280673949, 1.5162305346041565, 1.999999640801737, 1.9999999019185346, 1.999999975992308, 2.0, 0.10000000276344392, 2.0, 0.6883857919559779, 0.13570247909192534, 0.20446377604407198, 0.30582100391347616, 0.5149024655183062, 1.3632670817129235, 1.9999999155805996, 1.3094289602080853};
 
int j;

int hopCounter = 0;

double totalTime=0;
double sumHM=0;
double sumHMadjusted=0;
int d=0;

const int numValues = N;
//double hm[10] = {   5,  12,  30,  50,  60,  70,  74,  84,  92, 100 };//time
//double th1[10] = { 150, 200, 200, 200, 180, 100, 100, 150, 220, 320 };//angle 1
double hm_cumSum[30]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
 
double currentTime=0;
long EncoderHeightValue, oldEncoderHeightValue=-999;

void setup() 
{
//  delay(2000);
  Serial.begin(9600);
  Serial.println("Basic Encoder Test:");
  hip.attach(HIP_SERVO_PIN);  
  knee.attach(KNEE_SERVO_PIN);

  int a;
  int b = 0;
  int indexMax = 0;
  int angleKneeMax;
  for (a=0; a<N; a++)
  {
    //Serial.println(wrongtrajHM[a]); //angles are being read correctly, now check that they are being converted correctly
    //System.out.printf("%5d", arr[row][col]);
      
   trajHM[b] = (wrongtrajHM[a])/30.0;
   trajHip[b] = (wrongtrajHip[a]); //*180/3.141592654; 
   trajKnee[b] = (wrongtrajKnee[a]);  //*180/3.141592654;
   
    b++;
  }
  
  angleKneeMax = trajKnee[0];     //cddetermin the idnex in the array where the angle of the hip is at the maximum, then speed up the motion after that index by scaling down hm[i]
  for(int i=0;i<numValues;i++){
    if (trajKnee[i] > angleKneeMax){
      angleKneeMax = trajKnee[i];
      indexMax = i; 
    }
  }
  for(int i=0;i<numValues;i++)
  {
    sumHM = sumHM +trajHM[i];

    if (i > indexMax)
      trajHM[i] = trajHM[i] / speedUpFactor;  //attempt to execute motion faster, to get robot to jump
     else
      trajHM[i] = trajHM[i] * 4;  //attempt to execute motion faster, to get robot to jump

    totalTime = totalTime + trajHM[i];
    sumHMadjusted = sumHMadjusted +trajHM[i];

    for(int j=0;j<i;j++)
    {
      hm_cumSum[i]=hm_cumSum[i]+trajHM[j];
      
    }
  }
  Serial.println(indexMax);
  Serial.println(sumHMadjusted);
  Serial.println(sumHM);

  
    //getReady();
//    delay(1000);
}
 
void loop()
{
  double th1_command=Interpolation::Linear(hm_cumSum, trajHip, numValues, currentTime, true);
  double th2_command=Interpolation::Linear(hm_cumSum, trajKnee, numValues, currentTime, true);
  //do for th2
  currentTime=currentTime+1.0/freq;

  
//long newPosition = myEnc.read()*74.0/(360*4);
//  if (newPosition != oldPosition) {
//    oldPosition = newPosition;
//    Serial.println(newPosition);
//  }

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
    standStraight();
    while(true);////////////////// CALLEN (to run only once)
  }

  wait_control_loop();  //waits 5ms if frequency set to 200Hz

  
//  delay(200);


}

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
    command_servo(hip_servo,92);
    command_servo(knee_servo,89);
}

void getReady()
{
    command_servo(hip_servo,60);
    command_servo(knee_servo,120);
}

//long get_height()
//{
//  long temp= heightEnc.read();
//  long scaleFac=1;//0.06/(4*360);
//  temp=temp*scaleFac;
//  if (temp != oldPosition) 
//  {
//    oldPosition = temp;
//  }
//  return temp;
//}

void wait_control_loop()
{
  
  
  evalTime = previousTime + loopTime;
  while (micros() < evalTime)
  {
    long newPosition = myEnc.read()*74.0/(360*4);
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    Serial.println(newPosition);
  }
  }
  previousTime=micros();
}

//void interpolationFunc()
//{
 // #include "InterpolationLib.h"

//const int numValues = 10;
//double xValues[10] = {   5,  12,  30,  50,  60,  70,  74,  84,  92, 100 };
//double yValues[10] = { 150, 200, 200, 200, 180, 100, 100, 150, 220, 320 };
