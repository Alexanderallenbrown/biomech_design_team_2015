/*
  Encod.h - Library for reading an encoder using Interrupts
  Created by Alexander Brown, Ph.D. Feb. 2015
*/
#ifndef Encod_h
#define Encod_h

#include "Arduino.h"

class Encod
{
  public:
    Encod(int encnum, float cpr);
    float readPosRad();// reads position in radians
    float readVelRad();// reads velocity in radians/sec
    volatile long unCountShared;
    void channelA();
    void channelB();
    int _interrupt1;//number of first interrupt (0,1,2,3,4)
    int _interrupt2;//number of second interrupt
  private:
    float _cpr;//counts per revolution for radians.
    int _pin1;
    int _pin2;//number of second interrupt pin
    long _oldtime;//used in the velocity calculation
    long _oldpos;


};




#endif