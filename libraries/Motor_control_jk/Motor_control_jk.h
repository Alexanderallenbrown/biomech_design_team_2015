/*
   Motor_control.h - library for haptic position control of a 4 bar mechanism with a servomotor to follow a certain letter
   Created by: 
         by Thomas S. Todd, Eamon Campolettano, Jessica Ross, Kevin Fasano, 
         "multiMap function was written by ____ "
*/

#ifndef Motor_control_jk_h
#define Motor_control_jk_h

#include <Arduino.h>

class Motor_control_jk
{
  public:
    Motor_control_jk();

    float ftheta_1;
    float ftheta_3;

    //Position data along mapped 'g'
    // a 
    // static const int len_a = 101;
    // static float time_a[len_a];
    // static float theta_ones_a[len_a];
    // static float theta_threes_a[len_a];

     void findMotor_position(float theta_ones[100], float theta_threes[100], float time[100], float currtime);
    float multiMap(float val, const float* _in, const float* _out, uint8_t size);

  private:


};




#endif