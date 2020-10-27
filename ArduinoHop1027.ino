// Arduino Pro Mini, 5V 16MHz, ATMega 328p
//install servo library
//install encoder library

#define KNEE_SERVO_PIN  9
#define HIP_SERVO_PIN   6
#define outPin 2

#define HIP_MAX_ANGLE 150
#define HIP_MIN_ANGLE 15

#define KNEE_MAX_ANGLE 175
#define KNEE_MIN_ANGLE 15

int hip_servo = 0;
int knee_servo = 1; 
int i = 0;
int val = 0;

#define angleAmp 1.0
#define speedUpFactor 2.0
#define freq 150.0
#define loopTime 1000000.0/freq //5000     //(micro seconds)    //Gives control loop 2kHz Frequency
#define N 30     // (number of node points)
unsigned long previousTime = 0;
unsigned long evalTime = 500;

#include <Servo.h>
#include <Encoder.h>
#include "InterpolationLib.h"


//digitalRead(2);
Servo hip;
Servo knee; 
//Encoder heightEnc(8,7); //green = A phase...... White=B phase int long
Encoder myEnc(7, 8);
//gpio 2 code as output


int numHops = 1;
int hip_pos = 0;   
int knee_pos = 0; 

long oldPosition  = -999;
long height;
long newPosition=0;


double trajHM[N];
double trajHip[N];
double trajKnee[N];
double newtrajHip[N];
double heightLeg[N];

double wrongtrajHip[] = {84.73677419299099, 74.22912044180912, 72.01039517688871, 58.247945970777934, 44.41393975282023, 44.74049443121921, 33.228372181412595, 29.99999955904741, 30.082817281108227, 45.205428094697744, 46.40961028164074, 47.503409101374, 48.64841111992292, 49.81962116477748, 50.99635144120866, 52.17334132936475, 53.35038464229254, 54.534044249192625, 65.56363850762897, 68.1642056995087, 72.82835355272971, 80.44101392750962, 81.54427078273336, 88.10295273093377, 88.4982173691371, 88.88443834494502, 89.26296960611911, 89.6351797593174, 90.00000061844366, 84.73677419299099};
double wrongtrajKnee[] = {122.6353392756217, 129.5003627492017, 130.89946051224504, 129.34869704669887, 136.8796196362577, 136.70283306758057, 142.55404076186798, 145.00000133715557, 144.98805013806603, 137.64424251405913, 136.97950468387822, 136.32651040013053, 135.5621475903467, 134.71731862653758, 133.81923190495598, 132.88143721364003, 131.91507904206375, 131.04587821488903, 124.71804590127846, 123.18927037044635, 120.39585377035475, 115.74489717046394, 115.07627687414946, 111.88273498504634, 111.77145683131782, 111.67132100871608, 111.58063342768146, 111.49767726826329, 111.42556895260905, 122.6353392756217};
double wrongtrajHM[] = {2.0, 0.1892407320862379, 1.9282493813853683, 1.999999952796782, 0.10000001648202753, 1.9999999955137182, 2.0, 0.1, 2.0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.6817166907208759, 0.2159804445921282, 0.3885153344883906, 0.6566141603406591, 0.1, 0.8604226479128441, 0.1, 0.1, 0.1, 0.1, 0.10000005410916527, 1.99999959757759, 0.7235915311435779};


//0.31
//0.29
//0.29
//0.28


int j;

int hopCounter = 0;

double totalTime=0;
double sumHM=0;
double sumHMadjusted=0;
int d=0;

const int numValues = N;
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
  
  pinMode(outPin, OUTPUT);

  int a;
  int b = 0;
  int indexLegMax = 0;
  int indexLegMin = 0;
  int indexMax = 0;
  int angleKneeMax;
  int heightLegMax;
  int heightLegMin;
  int stretch = 0; 
  
  for (a=0; a<N; a++)
  {
      
   trajHM[b] = (wrongtrajHM[a])/30;
   trajHip[b] = (wrongtrajHip[a]); //*180/3.141592654; 
   trajKnee[b] = (wrongtrajKnee[a]);  //*180/3.141592654;
   
   heightLeg[b] = 0.12*sin(trajHip[b]*(3.14159265/180)) + 0.23*sin((180-trajKnee[b])*(3.14159265/180)); //see sketch on ipad for calculation of imaginary leg height - covert also to degrees since sintakes (rad)
    
    //Serial.println(heightLeg[b]);
    
    b++;

   
    
  }
  ///////////////////////////////////
  angleKneeMax = trajKnee[0];     //determine the index in the array where the angle of the hip is at the maximum, then speed up the motion after that index by scaling down hm[i]
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
      trajHM[i] = trajHM[i] * speedUpFactor;  //attempt to execute motion faster, to get robot to jump
     else
      trajHM[i] = trajHM[i] * 2;  //attempt to execute motion faster, to get robot to jump

    totalTime = totalTime + trajHM[i];
    sumHMadjusted = sumHMadjusted +trajHM[i];

    for(int j=0;j<i;j++)
    {
      hm_cumSum[i]=hm_cumSum[i]+trajHM[j];
      
    }
  }

  //////////////////////////////////
heightLegMax = heightLeg[0];     //find index of max imaginary leg
  for(int p=0;p<numValues;p++){
    if (heightLeg[p] > heightLegMax){
      heightLegMax = heightLeg[p];
      indexLegMax = p; 
    }
  }

  
heightLegMin = heightLeg[0];     //find index of min imaginary leg
  for(int q=0;q<numValues;q++){
    if (heightLeg[q] < heightLegMin){
      heightLegMin = heightLeg[q];
      indexLegMin = q; 
    }
  }

stretch = heightLeg[indexLegMax]-heightLeg[indexLegMin];

Serial.println(indexLegMax);
Serial.println(heightLeg[indexLegMax]);
Serial.println(indexLegMin);
Serial.println(heightLeg[indexLegMin]);
Serial.println(stretch);
 


  //////////////////////////////////
  
//  Serial.println(indexMax);
//  Serial.println(sumHMadjusted);
//  Serial.println(sumHM);

    //getReady();
//    delay(1000);
}
 
void loop()
{
  digitalWrite(outPin, !digitalRead(outPin));

  //Serial.println(currentTime);
  double th1_command=Interpolation::Linear(hm_cumSum, trajHip, numValues, currentTime, true);
  double th2_command=Interpolation::Linear(hm_cumSum, trajKnee, numValues, currentTime, true);
  
  currentTime=currentTime+(1.0/freq);//*speedUpFactor;


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
    //Serial.print("Done hopping");
   standStraight();
//    while(true);////////////////// CALLEN (to run only once)
  }
  //Serial.println(heightLeg);
  //Serial.println(newPosition);
  wait_control_loop();  //waits 5ms if frequency set to 200Hz


}

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
    //delay(delay_time);                      
  }
  for (pos = max_angle; pos >= min_angle; pos -= 1) 
  { 
    command_servo(servo,pos);               
    //delay(delay_time);                       
  }
}

void standStraight()
{
    command_servo(hip_servo,90);
    command_servo(knee_servo,96);
}

void getReady()
{
    command_servo(hip_servo,60);
    command_servo(knee_servo,120);
}

void wait_control_loop()
{
  
  
  evalTime = previousTime + loopTime;
  while (micros() < evalTime)
  {
    newPosition = myEnc.read()*74.0/(360.0*4);
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
//    Serial.println(newPosition);

//Serial.println(heightLeg);
  }
  }
  previousTime=micros();
}
