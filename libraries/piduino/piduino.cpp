#include "Arduino.h"
#include "piduino.h"


/* Piduino PID library
Alexander Brown, Ph.D. 
Feb 2015
All rights reserved.
*/


// Constructor. This function runs when we initiate a PID object.
Piduino::Piduino(int dirpin1,int dirpin2, int speedpin, float deadband, float inkp, float inki, float inkd, float inksum)
{
    //take inputs and assign them to class-owned vars
    _dirpin1 = dirpin1;
    _dirpin2 = dirpin2;
    _speedpin = speedpin;
    _deadband = deadband;
    kp = inkp;
    ki = inki;
    kd = inkd;
    ksum = inksum;
    

    //initialize the rest of the variables we need:
    error=0;
    interror=0;
    _dT=0.01;
    _oldtime=0;
    _time=float(millis())*.001;
    u_tot = 0;
    u_p = 0;
    u_i = 0;
    u_d = 0;
    magnitude = 0;

    //now set up the hardware:
    pinMode(_dirpin1,OUTPUT);
    pinMode(_dirpin2,OUTPUT);

    digitalWrite(_dirpin1,LOW);
    digitalWrite(_dirpin2,HIGH);
}

void Piduino::writePID(float command, float theta, float omega){
//calculate time now
  _time = float(millis())*.001;//get current time
  //calculate timestep
  _dT = _time-_oldtime;
  //re-set last time
  _oldtime = _time;
  
 
 error = command-theta;//calculate current error
 interror+=error*_dT;//calculate integral of error
 float u_p = kp*error;//proportional part of error
 float u_d = -kd*omega;//derivative applied to motion, not to error to save zero craziness.
 float u_i = ki*omega;//integral 
 float u_tot = (u_p+u_d+u_i)*ksum;
 
 if (u_tot<=0){
   digitalWrite(_dirpin1,HIGH);
   digitalWrite(_dirpin2,LOW);
   u_tot = u_tot-_deadband;
   magnitude = int(abs(u_tot));
   if (magnitude>255){
     magnitude==255;
   }
   analogWrite(_speedpin,magnitude);
 }
 else{
   digitalWrite(_dirpin1,LOW);
   digitalWrite(_dirpin2,HIGH);
   u_tot = u_tot+_deadband;
   magnitude = int(abs(u_tot));
   if (magnitude>255){
     magnitude==255;
   }
   analogWrite(_speedpin,magnitude);
 } 

}