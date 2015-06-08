/*
  Encod.h - Library for PID control using a dual H-Bridge like the SEEED studio Motor Shield
  Created by Alexander Brown, Ph.D. Feb. 2015
  Uses d-feedback from the output, not from error, to avoid nutty zeros.
  All rights reserved
*/
#ifndef piduino_h
#define piduino_h

#include "Arduino.h"

class Piduino
{
  public:
    Piduino(int dirpin1,int dirpin2, int speedpin, float deadband, float inkp, float inki, float inkd, float inksum);
    float u_p;
    float u_i;
    float u_d;
    float u_tot;
    float kp;
    float kd;
    float ksum;
    float ki;
    int magnitude;
    float error;
    float interror;
    void writePID(float command, float theta,float omega);
  private:

    float _dT;
    float _oldtime;
    float _time;
    float _speedpin;
    float _dirpin1;
    float _dirpin2;
    float _deadband;

};




#endif