#include "Arduino.h"
#include "Motor_control_jk.h"


/*    Servo/DC motor position - library for controlling a DC or servo motor position using a variable resistor 
   Created by: 

         Michal Rinott <http://people.interaction-ivrea.it/m.rinott> 

         modified on 8 Nov 2013
         by Scott Fitzgerald
         http://arduino.cc/en/Tutorial/Knob

         modified Feb 2015
         by Eamon Campolettano, Jessica Ross, Kevin Fasano, Thomas S. Todd

  All Rights Reserved
*/


// Constructor. This function runs when we provide 
Motor_control_jk::Motor_control_jk(){
    //take inputs and assign them to class-owned vars

    ftheta_1 = 0;
    ftheta_3 = 0;


}

 void Motor_control_jk::findMotor_position(float theta_ones[100], float theta_threes[100], float time[100], float currtime) 
{ 

int len = 101;
//float time_a[len_a];
//float theta_threes_a[len_a];
//float theta_ones_a[len_a];

//const float time_l[] = {5,5.0002,5.0066,5.0107,5.0135,5.0159,5.0185,5.0211,5.0241,5.0272,5.0306,5.0342,5.0378,5.0415,5.0458,5.0502,5.0554,5.0609,5.0663,5.0719,5.078,5.0842,5.0905,5.097,5.1036,5.1106,5.1178,5.1249,5.132,5.1389,5.1457,5.1523,5.159,5.1656,5.1725,5.1788,5.1848,5.1908,5.1962,5.2013,5.2065,5.2118,5.2172,5.2224,5.2272,5.2316,5.236,5.2405,5.2449,5.2491,5.2533,5.2575,5.2616,5.2658,5.27,5.2743,5.2783,5.2825,5.2867,5.2912,5.2958,5.3004,5.305,5.31,5.315,5.32,5.3252,5.3302,5.3347,5.3392,5.3435,5.3477,5.352,5.3563,5.3607,5.3653,5.3696,5.3744,5.3793,5.3843,5.3894,5.3944,5.3995,5.4044,5.4106,5.417,5.4238,5.431,5.4386,5.445,5.4473,5.4493,5.4522,5.4601,5.4668,5.4711,5.4791,5.4803,5.4803,5.4803,5.4803};
//const float theta_ones_l[] = {-0.2278,-0.22607,-0.22536,-0.22603,-0.22738,-0.22905,-0.23076,-0.23245,-0.23408,-0.23569,-0.23724,-0.23878,-0.2403,-0.24179,-0.24322,-0.24461,-0.24591,-0.24717,-0.24843,-0.24966,-0.25083,-0.25198,-0.25313,-0.25426,-0.25537,-0.25644,-0.25749,-0.25855,-0.25962,-0.26072,-0.26185,-0.263,-0.26415,-0.26531,-0.26646,-0.26767,-0.26894,-0.27021,-0.27155,-0.27293,-0.27432,-0.27569,-0.27705,-0.27845,-0.2799,-0.28139,-0.28289,-0.28439,-0.28589,-0.28743,-0.28897,-0.29052,-0.29207,-0.29363,-0.29518,-0.29675,-0.29833,-0.29992,-0.3015,-0.30307,-0.30464,-0.30621,-0.30778,-0.30934,-0.3109,-0.31247,-0.31403,-0.31562,-0.31725,-0.31888,-0.32053,-0.3222,-0.32387,-0.32555,-0.32722,-0.3289,-0.33059,-0.33227,-0.33395,-0.33564,-0.33733,-0.33903,-0.34074,-0.34246,-0.34415,-0.34584,-0.34754,-0.34923,-0.35093,-0.35266,-0.35448,-0.35637,-0.35826,-0.36019,-0.36209,-0.36394,-0.36579,-0.36759,-0.36759,-0.36759,-0.36759};
//const float theta_threes_l[] = {0.29452,0.29638,0.2978,0.29921,0.30076,0.30245,0.30437,0.30632,0.30838,0.31049,0.3127,0.31495,0.31722,0.31954,0.32198,0.32448,0.32714,0.32987,0.3326,0.33538,0.33825,0.34116,0.34407,0.34702,0.35,0.35306,0.35615,0.35923,0.36229,0.36531,0.36829,0.37124,0.37418,0.37712,0.38008,0.38294,0.38571,0.38848,0.39113,0.39373,0.39632,0.39894,0.40157,0.40415,0.40665,0.40908,0.4115,0.41393,0.41635,0.41872,0.42109,0.42344,0.42579,0.42815,0.43051,0.43286,0.43517,0.43749,0.43982,0.44218,0.44454,0.4469,0.44926,0.45165,0.45404,0.45642,0.45881,0.46117,0.46345,0.46573,0.46799,0.47023,0.47246,0.47469,0.47692,0.47916,0.48137,0.48361,0.48585,0.48809,0.49032,0.49254,0.49475,0.49695,0.4992,0.50146,0.50372,0.50598,0.50824,0.51044,0.51248,0.5144,0.51632,0.51817,0.5201,0.5221,0.52412,0.52624,0.52624,0.52624,0.52624};


 //int len_o = 101;
 //const float time_o[] = {0.29452,0.29886,0.30412,0.31002,0.3158,0.32213,0.32835,0.3343,0.34037,0.3466,0.35288,0.35908,0.36499,0.3707,0.37611,0.38136,0.38664,0.39173,0.39662,0.40121,0.4056,0.40995,0.41438,0.41897,0.42364,0.42766,0.43135,0.43391,0.43546,0.43364,0.42929,0.42515,0.42115,0.41757,0.41417,0.41063,0.4073,0.4044,0.40182,0.39961,0.3979,0.39693,0.39741,0.39883,0.4005,0.40278,0.40606,0.40996,0.41498,0.42043,0.42683,0.43377,0.44185,0.45037,0.45987,0.46993,0.48084,0.49214,0.50469,0.51769,0.53078,0.54397,0.55736,0.57093,0.58458,0.59818,0.61164,0.62499,0.63822,0.65093,0.6634,0.67561,0.68752,0.69902,0.71001,0.72087,0.73139,0.74124,0.7505,0.75925,0.76754,0.77479,0.78161,0.78769,0.79319,0.79846,0.80282,0.80614,0.80866,0.81097,0.81257,0.81375,0.81473,0.81552,0.81614,0.8168,0.81751,0.81806,0.81862,0.81862,0.81862};
 //const float theta_ones_o[] = {-0.2278,-0.22442,-0.22619,-0.23141,-0.23736,-0.24342,-0.2511,-0.2596,-0.26781,-0.27677,-0.28673,-0.29736,-0.30871,-0.3209,-0.33368,-0.34747,-0.36204,-0.37739,-0.3935,-0.41038,-0.42802,-0.44657,-0.46583,-0.48573,-0.50627,-0.52754,-0.54946,-0.57176,-0.59418,-0.61635,-0.63835,-0.65987,-0.68072,-0.70033,-0.71918,-0.73799,-0.75471,-0.7703,-0.78467,-0.79718,-0.80668,-0.80966,-0.81081,-0.81014,-0.80613,-0.79845,-0.79016,-0.78027,-0.76914,-0.75823,-0.74653,-0.73388,-0.72089,-0.70794,-0.69468,-0.68215,-0.67037,-0.65829,-0.64591,-0.6334,-0.62111,-0.60897,-0.59667,-0.58442,-0.57241,-0.56028,-0.54822,-0.5362,-0.5243,-0.51255,-0.50098,-0.48956,-0.47817,-0.46685,-0.45562,-0.44453,-0.43355,-0.42263,-0.41177,-0.40102,-0.39039,-0.37981,-0.36927,-0.35884,-0.34849,-0.33817,-0.32792,-0.31786,-0.30802,-0.29848,-0.28889,-0.27931,-0.27009,-0.26132,-0.25332,-0.24574,-0.23867,-0.23288,-0.22823,-0.22823,-0.22823};
 //const float theta_threes_o[]= {0.29452,0.29831,0.28771,0.27198,0.25531,0.23842,0.21992,0.20051,0.18114,0.16092,0.13965,0.11764,0.094852,0.071259,0.046937,0.021611,-0.0045252,-0.031494,-0.059287,-0.087842,-0.1172,-0.14732,-0.17807,-0.20923,-0.24083,-0.27335,-0.3065,-0.33957,-0.37211,-0.40352,-0.43416,-0.46346,-0.49116,-0.51631,-0.53991,-0.5633,-0.58287,-0.60034,-0.61559,-0.62755,-0.63429,-0.63004,-0.62273,-0.61244,-0.59676,-0.57553,-0.55344,-0.52912,-0.5033,-0.47814,-0.4521,-0.42516,-0.39826,-0.37182,-0.34556,-0.32024,-0.2961,-0.27208,-0.24815,-0.22466,-0.20172,-0.17952,-0.15785,-0.13656,-0.11574,-0.09545,-0.075735,-0.056544,-0.037758,-0.019214,-0.00089257,0.017035,0.0343,0.051009,0.067562,0.083259,0.098303,0.11312,0.12796,0.14253,0.15629,0.16989,0.1835,0.19669,0.20963,0.22266,0.23534,0.24721,0.25816,0.26788,0.27782,0.2878,0.2967,0.30422,0.30986,0.31444,0.31784,0.31873,0.31729,0.31729,0.31729};

 //c
 // int len_g = 101;
  //const float time_g[] = {5,5.0526,5.1025,5.141,5.1765,5.2092,5.2398,5.2686,5.2963,5.3221,5.346,5.3711,5.4464,5.4729,5.5024,5.5316,5.5594,5.5862,5.6136,5.6412,5.6688,5.6971,5.7258,5.7544,5.7814,5.8029,5.8076,5.8457,5.8786,5.9139,5.9474,5.9803,6.0116,6.0412,6.0717,6.0978,6.1007,6.1243,6.1508,6.1763,6.1999,6.2233,6.2484,6.2744,6.3004,6.3265,6.3539,6.3824,6.4113,6.4405,6.4696,6.4995,6.5273,6.5559,6.5799,6.6077,6.7548,6.8237,6.8679,6.9399,7.1121,7.1139,7.1295,7.161,7.1916,7.2251,7.2578,7.28,7.3032,7.3193,7.3296,7.3435,7.3591,7.4329,7.4705,7.4991,7.5189,7.5377,7.5607,7.5847,7.6096,7.6353,7.6632,7.6924,7.7228,7.754,7.7865,7.8195,7.8558,7.8935,7.925,7.9517,7.9771,8.0073,8.0396,8.0725,8.1071,8.143,8.1806,8.1806,8.1806};
  //const float theta_ones_g[] = {-0.2278,-0.23181,-0.24115,-0.2537,-0.26757,-0.28417,-0.30344,-0.32527,-0.34944,-0.37579,-0.40389,-0.43352,-0.46421,-0.49541,-0.52577,-0.55478,-0.58196,-0.60711,-0.62845,-0.64627,-0.66112,-0.67213,-0.67838,-0.67968,-0.67609,-0.66852,-0.66004,-0.64862,-0.63592,-0.6193,-0.60153,-0.58326,-0.56365,-0.54413,-0.52475,-0.50513,-0.48578,-0.46661,-0.44768,-0.42901,-0.41043,-0.39209,-0.37406,-0.35639,-0.33892,-0.3215,-0.30469,-0.28874,-0.27385,-0.26033,-0.2482,-0.23696,-0.22779,-0.21951,-0.21284,-0.20655,-0.20208,-0.20053,-0.20298,-0.20846,-0.21484,-0.22113,-0.22682,-0.23149,-0.23602,-0.24022,-0.24444,-0.24952,-0.25459,-0.26025,-0.26627,-0.27216,-0.27809,-0.28445,-0.29131,-0.29838,-0.30518,-0.31211,-0.31999,-0.32849,-0.33756,-0.34723,-0.35798,-0.36953,-0.38178,-0.39448,-0.40763,-0.42119,-0.43489,-0.44847,-0.46184,-0.47485,-0.48784,-0.50012,-0.51162,-0.5228,-0.53319,-0.54334,-0.55295,-0.55295,-0.55295};
 // const float theta_threes_g[] = {0.29452,0.28361,0.25716,0.22559,0.19229,0.15624,0.11764,0.07679,0.033988,-0.010346,-0.055676,-0.10207,-0.14822,-0.19352,-0.23495,-0.27222,-0.30515,-0.33376,-0.35474,-0.36904,-0.378,-0.38003,-0.37393,-0.35969,-0.33776,-0.31003,-0.28116,-0.24903,-0.21582,-0.17924,-0.14313,-0.10782,-0.073265,-0.040811,-0.0098524,0.019564,0.048284,0.075383,0.1008,0.12526,0.14972,0.1735,0.19561,0.21617,0.23607,0.25606,0.27366,0.28835,0.30018,0.30835,0.31334,0.31621,0.31507,0.31211,0.30642,0.30033,0.29105,0.28085,0.27551,0.2771,0.28333,0.28985,0.29744,0.30674,0.31628,0.32641,0.33653,0.34529,0.3541,0.362,0.36929,0.37683,0.38432,0.39108,0.39695,0.40252,0.40865,0.41455,0.41859,0.4214,0.42303,0.42335,0.42103,0.41633,0.40922,0.39955,0.38733,0.37188,0.35464,0.33529,0.31484,0.293,0.2706,0.24718,0.22299,0.19834,0.17342,0.14813,0.12278,0.12278,0.12278};

  //switch(letter){
    //case 1 :  
    if(currtime>time[0]){
    if(currtime<time[len-1]){
      ftheta_1 = multiMap(currtime, time, theta_ones, len);
      ftheta_3 = multiMap(currtime, time, theta_threes, len);
  }
      else{
        ftheta_1 = theta_ones[len-1];
        ftheta_3 = theta_threes[len-1];
        }
      }
      else{
        ftheta_1 = theta_ones[0];
        ftheta_3 = theta_threes[0];
      }
/*     case 2:
    if(currtime>time_o[0]){
    if(currtime<time_o[len_o-1]){     
      ftheta_1 = multiMap(currtime, time_o, theta_ones_o, len_o);
      ftheta_3 = multiMap(currtime, time_o, theta_threes_o, len_o);
  }
      else{
        ftheta_1 = theta_ones_o[len_o-1];
        ftheta_3 = theta_threes_o[len_o-1];
        }
      }
      else{
        ftheta_1 = theta_ones_o[0];
        ftheta_3 = theta_threes_o[0];
      }
    case 3:
    if(currtime>time_g[0]){
    if(currtime<time_g[len_g-1]){ 
      ftheta_1 = multiMap(currtime, time_g, theta_ones_g, len_g);
      ftheta_3 = multiMap(currtime, time_g, theta_threes_g, len_g);
   }
      else{
        ftheta_1 = theta_ones_g[len_g-1];
        ftheta_3 = theta_threes_g[len_g-1];
        }
      }
      else{
        ftheta_1 = theta_ones_g[0];
        ftheta_3 = theta_threes_g[0];
      }      

        break;
  } */

  }

// note: the _in array should have increasing values
float Motor_control_jk::multiMap(float val, const float* _in, const float* _out, uint8_t size)
{Serial.print("hello \t");
  // take care the value is within range
  // val = constrain(val, _in[0], _in[size-1]);
  if (val <= _in[0]) {
    return _out[0];
 }
  
  
  if (val >= _in[size-1]){
    return _out[size-1];
    
  }
  
  // search right interval
  uint8_t pos = 1;  // _in[0] allready tested
  while(val > _in[pos]) pos++;
 
  // this will handle all exact "points" in the _in array
  if (val == _in[pos]) return _out[pos];
  // interpolate in the right segment for the rest
  return (val - _in[pos-1]) * (_out[pos] - _out[pos-1]) / (_in[pos] - _in[pos-1]) + _out[pos-1];
}