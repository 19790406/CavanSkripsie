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


#include <Servo.h>
#include <Encoder.h>

Servo hip;
Servo knee; 
Encoder heightEnc(7, 8);

int hip_pos = 0;    // variable to store the servo position
int knee_pos = 0; 

long oldPosition  = -999;
long height;

void setup() 
{
  Serial.begin(9600);
  hip.attach(HIP_SERVO_PIN);  
  knee.attach(KNEE_SERVO_PIN);
}

void loop() 
{
  // read encoder
  height = get_height();

  
  //sweep hip
  sweep_servo(hip_servo,HIP_MIN_ANGLE,HIP_MAX_ANGLE,15);
  //sweep knee
  //sweep_servo(knee_servo,KNEE_MIN_ANGLE,KNEE_MAX_ANGLE,15);
}

void command_servo(int servo,int angle)
{
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
