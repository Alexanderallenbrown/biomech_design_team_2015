#include "Arduino.h"
#include "Encod.h"
// #include "avr/interrupt.h"


// Constructor. This function runs when we initiate an encoder object.
Encod::Encod(int encnum, float cpr)
{
    _cpr = cpr;
    unCountShared = 0;//initialize our count
    _oldtime = millis();//initialize old time
    _oldpos = 0;//initialize old position for velocity
    //first we need to see which interrupts our user chose, and set pins.
    if(encnum==0){
        _pin1 = 2;
        _pin2 = 3;
        _interrupt1 = 0;
        _interrupt2 = 1;
    }
    else if(encnum==1){
        _pin1 = 21;
        _pin2 = 20;
        _interrupt1 = 2;
        _interrupt2 = 3;
    }
    else{
        _pin1 = 19;
        _pin2 = 18;
        _interrupt1 = 4;
        _interrupt2 = 5;
    }
}

float Encod::readPosRad(){
    noInterrupts();
    float pos = float(unCountShared)*3.141592654*2/_cpr;
    interrupts();
    return pos;
}

float Encod::readVelRad(){
    noInterrupts();
    long newtime = millis();
    float dT = (newtime-_oldtime)*.001;
    _oldtime = newtime;
    float vel = (float(unCountShared-_oldpos)*2*3.141592654/_cpr)/dT;
    _oldpos = unCountShared;
    interrupts();
    return vel;
}

// simple interrupt service routine
void Encod::channelA()
{
  //Serial.println("A");
  if (digitalRead(_pin1) == HIGH)
  {
    if (digitalRead(_pin2) == LOW)
    {
    unCountShared++;
    }
    else
    {
    unCountShared--;
    }
  }
  else
  {
    if (digitalRead(_pin2) == HIGH)
    {
    unCountShared++;
    }
    else
    {
    unCountShared--;
    }
  }
}

void Encod::channelB()
{
  //Serial.println("B");
  if (digitalRead(_pin2) == HIGH)
  {
    if (digitalRead(_pin1) == HIGH)
    {
    unCountShared++;
    }
    else
    {
    Encod::unCountShared--;
    }
  }
  else
  {
    if (digitalRead(_pin1) == LOW)
    {
    unCountShared++;
    }
    else
    {
    unCountShared--;
    }
  }
}