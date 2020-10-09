// Arduino Pro Mini, 5V 16MHz, ATMega 328p
//install servo library
//install encoder library

#define KNEE_SERVO_PIN  9
#define HIP_SERVO_PIN   6
#define CS_PIN 10    ///??? correct for SD card??

#define HIP_MAX_ANGLE 150
#define HIP_MIN_ANGLE 15

#define KNEE_MAX_ANGLE 175
#define KNEE_MIN_ANGLE 15
#define numData 90
int hip_servo = 0;
int knee_servo = 1; 


#include <Servo.h>
#include <Encoder.h>
//#include <File.h>    //for reading data.npy
#include <SPI.h>
#include <SD.h>

File f;
int i;

int theta[numData][2];

int thetaHip[numData];   //angles at different colocation points, whole numbers since sweep_servo() takes in whole numbers
int thetaKnee[numData];

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


  // Initialize the SD.
  if (!SD.begin(CS_PIN)) {
//    errorHalt("begin failed");
  }
  // Create or open the file.
  f = SD.open("test1angledata.rtf", FILE_READ);
  if (!f) {
//    errorHalt("open failed");
  }

//  f = open("test1angledata.rtf", "r")       //where is the data being read from / how does it get onto the Arduino?   Is Arduino running on its own and the data.npy is first transferred to its memory,
//                                  // or is it being connected to computer which sends the data over the cable while arduino already running?
//  for (i = 0; i < numData; i += 1){
//    thetaHip[i] = strt(f.read());
//    thetaKnee[i] = f.read();
//  }


  int i = 0;        // First array index.
  int j = 0;        // Second array index
  size_t n;         // Length of returned field with delimiter.
  char str[20];     // Must hold longest field with delimiter and zero byte.
  char *ptr;        // Test for valid field.
  char delim = 0;   // Delimiter from previous line. Start with no delimiter.
  char *comma = (char *) malloc(3*sizeof(char));
  comma[0] = ',';
  comma[1] = '\n';          // might not need to be \n, could cause crash when reading fields which are not at the end of a line
  comma[2] = '\0';

  // Read the file and store the data.
  while (true) {
//    n = readField(&f, str, sizeof(str), ",\n");
    n = readField(&f, str, sizeof(str), comma);

    // Read error or at EOF.
    if (n == 0) {
      break;
    }
    // Advance indices based on previous delimiter.
    if (delim == '\n') {
      // previous delimiter was endl so start a new row.
      if (++i >= numData) {
//        errorHalt("too many lines");
          break;
      }
      if (j != (2 - 1)) {
//        errorHalt("missing field");
          break;
      }
      j = 0;
    } else if (delim == ',') {
      // previous delimiter was comma so advance column.
      if (++j >= 2) {
//        errorHalt("too many fields");
          break;
      }
    }
    theta[i][j] = strtol(str, &ptr, 10);
    if (ptr == str) {
//      errorHalt("bad number");
        break;
    }
  }
  f.close();
}
/* Read a file one field at a time.
 *
 * file - File to read.
 *
 * str - Character array for the field.
 *
 * size - Size of str array.
 *
 * delim - String containing field delimiters.
 *
 * return - length of field including terminating delimiter.
 *
 * Note, the last character of str will not be a delimiter if
 * a read error occurs, the field is too long, or the file
 * does not end with a delimiter.  Consider this an error
 * if not at end-of-file.
 */
size_t readField(File* file, char* str, size_t size, char* delim) {     //size = how many characters should be read AT MOST, but will stop when delimiter was reached
  char ch;
  size_t n = 0;
  while ((n + 1) < size && file->read(&ch, 1) == 1) {
    if (ch == '\r') { // Delete CR.
      continue;
    }
    str[n++] = ch;
    if (strchr(delim, ch)) {
        break;
    }
  }
  str[n] = '\0';
  return n;
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
