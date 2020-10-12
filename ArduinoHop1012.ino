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

#include <Servo.h>
#include <Encoder.h>

Servo hip;
Servo knee; 
Encoder heightEnc(7, 8);

int hip_pos = 0;    // variable to store the servo position
int knee_pos = 0; 

long oldPosition  = -999;
long height;

int trajHip[] = {61, 69, 66, 65, 51, 38, 30, 43, 44, 59, 61, 64, 67, 69, 69, 68, 68, 67, 66, 65, 64, 64, 63, 63, 63, 62, 62, 62, 61, 61};
int trajKnee[] = {119, 107, 109, 109, 116, 123, 126, 121, 120, 114, 113, 111, 109, 110, 111, 111, 112, 113, 114, 116, 116, 116, 117, 117, 117, 118, 118, 118, 118, 119};
int j;
int numHops = 4;
void setup() 
{
  Serial.begin(9600);
  hip.attach(HIP_SERVO_PIN);  
  knee.attach(KNEE_SERVO_PIN);
  
//  command_servo(hip_servo,0);
//  command_servo(knee_servo,90);
////  command_servo(knee_servo,106); //do not make this 0-20
//  sweep_servo(hip_servo,0,180,15);
//  delay(1000);
//  command_servo(hip_servo,90);
//  delay(1000);
//  sweep_servo(knee_servo,0,180,15);
//  delay(1000);
//  command_servo(hip_servo,0);
  for (j = 0; j < numHops; j++){
    FollowTraj();
  }
}

//if(i,20)

void loop() 
{
  // read encoder
  height = get_height();
  

  //sweep hip
  
 // sweep_servo(hip_servo,HIP_MIN_ANGLE,HIP_MAX_ANGLE,15);//////////////////////
  
  //sweep knee
  //sweep_servo(knee_servo,KNEE_MIN_ANGLE,KNEE_MAX_ANGLE,15);
}

void FollowTraj()
{
  int numIntervals = 3;
  int i, c;
  int delay_time = 5;
  int angleHip;
  int angleKnee;
  int y1h, y2h, y1k, y2k;
  int numElements = sizeof(trajHip)/sizeof(int);
  for (i = 0; i < numElements-1; i++){
    y1k = trajKnee[i];
    y2k = trajKnee[i+1];
    y1h = trajHip[i];
    y2h = trajHip[i+1];
    for (c=0; c < numIntervals; c++)
    {
      angleKnee = y1k + (y2k-y1k)*c/numIntervals;
      angleHip = y1h + (y2h-y1h)*c/numIntervals;   //linear interpolation
      angleHip = angleHip - 16;
      command_servo(hip_servo, angleHip);
      angleKnee = angleKnee-angleHip+90;
      command_servo(knee_servo, angleKnee);
      delay(delay_time);
    }
  }
}
void command_servo(int servo,int angle)
{
  if (servo == hip_servo)
    angle = angle*(165-20)/180 + 20;
  if (servo == knee_servo)
  {
    angle = 194-angle*(194-42)/180;
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
    delay(delay_time);                      
  }
  for (pos = max_angle; pos >= min_angle; pos -= 1) 
  { 
    command_servo(servo,pos);               
    delay(delay_time);                       
  }
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
