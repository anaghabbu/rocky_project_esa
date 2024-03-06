
// This sketch is provided for you to perform angle calibration of the gyro to remove a robot-dependent
// constant offset to the angle measurement.
// You may have to do this process again periodically if your gyro gets uncalibrated
//
// The procedure for calibration: 
//    Place the robot flat on the ground (this is important for gyro calibration)
//    Compile and load the code onto the robot
//    Open the serial monitor (from the tools menu in the Arduino IDE) and set the baud rate to 9600 baud 
//    Hold the robot upright in the air as if it were balancing (lift it up gently)
//    Record the value displayed on the screen
//    This value corresponds to a constant offset of the angle measuremant
//    You will have to subtract this value out of your angle measurement 
//    in your actual code 
//    by replacing the value 0.25 in the follwing line of code 
//      #define FIXED_ANGLE_CORRECTION (0.25)
//    in your balancing sketch
//
// The procedure for natural frequency measurement: 
//     Carefully remove the wheels of the robot
//     Place robot flat on the ground and compile and load the code
//     After  "done uploading" is reported in the Arduino IDE,
//     and the red LED has flashed, open the serial monitor (9600 baud)
//     Hold your robot upside down, with the axles resting on two supports (e.g. finger and thumb)
//     Swing your robot freely from left to right 
//     Copy and paste the angle measurements into MATLAB 
//     Find the natural frequency of the robot
//     Note that the output is displayed every 50 ms
//     This quantity will tell you the effective length of the pendulum
//     since the natural frequency is approximately sqrt{g/l}

#include <Balboa32U4.h>
#include <Wire.h>
#include <LSM6.h>
#include "Balance.h"


extern int32_t angle_accum;
extern int32_t speedLeft;
extern int32_t driveLeft;
extern int32_t distanceRight;
extern int32_t speedRight;
extern int32_t distanceLeft;
extern int32_t distanceRight;
extern int32_t displacement;
float speedCont = 0;
int32_t prev_displacement=0;
uint32_t cur_time, prev_time;


LSM6 imu;
Balboa32U4Motors motors;
Balboa32U4Encoders encoders;
Balboa32U4Buzzer buzzer;
Balboa32U4ButtonA buttonA;



void setup()
{
  Serial.begin(9600);
  prev_time = 0;
  displacement = 0;
  ledYellow(0);
  ledRed(1);
  balanceSetup();
  ledRed(0);
  angle_accum = 0;

  ledGreen(0);
  ledYellow(0);  
}


extern int16_t angle_prev;
void lyingDown();
extern bool isBalancingStatus;
extern bool balanceUpdateDelayedStatus;

void UpdateSensors()
{
  static uint16_t lastMillis;
  uint16_t ms = millis();

  // Perform the balance updates at 100 Hz.
  balanceUpdateDelayedStatus = ms - lastMillis > UPDATE_TIME_MS + 1;
  lastMillis = ms;

  // call functions to integrate encoders and gyros
  balanceUpdateSensors();

  if (imu.a.x < 0)
  {
    lyingDown();
    isBalancingStatus = false;
  }
  else
  {
    isBalancingStatus = true;
  }
}



// This is the main loop that gets called repeatedly

void loop()
{
  static uint32_t prev_print_time = 0;   // this variable is to control how often we print on the serial monitor
  static float angle_rad;                // this is the angle in radians
  static float del_theta = 0;
  
  cur_time = millis();                   // get the current time in miliseconds

  if(cur_time - prev_time > UPDATE_TIME_MS)  // run this block of code approximately every UPDATE_TIME_MS 
  {
    UpdateSensors();                   // run the sensor updates. 
    // calculate the angle in radians
    angle_rad = ((float)angle)/1000/180*3.14159;
    prev_time = cur_time;
  }  

  if(cur_time - prev_print_time > 50)   // do the printing every 53 ms. Don't want to do it for an integer multiple of 10ms to not hog the processor
  {
       Serial.println(angle_rad);   
       prev_print_time = cur_time;
  }

}
