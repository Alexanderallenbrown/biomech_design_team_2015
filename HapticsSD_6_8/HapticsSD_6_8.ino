#include <Motor_control_jk.h>
#include <piduino.h>
#include <encod.h>
#include <SPI.h>
#include <SD.h>
#include <MenuBackend.h>    //MenuBackend library - copyright by Alexander Brevig
#include <LiquidCrystal.h>  //this library is included in the Arduino IDE

//format: Encod(interrupt number, counts_per_revolution)
Encod enc1(0,50.0*64.0);//initialize an encoder
Encod enc2(1,50.0*64.0);//other encoder
//now initialize a "servo" PID object: format is:
//Piduino(int dirpin1,int dirpin2, int speedpin, float deadband, float inkp, float inki, float inkd, float inksum)
Piduino servo1(8,11,9,25.0,5000,75,100.0,1.0);
Piduino servo2(12,13,10,25.0,5000,75.0,100.0,1.0);

// initializing for SD card
File myFile;
char zero = '0';
//float time[5,5.0526,5.1025,5.141,5.1765,5.2092,5.2398,5.2686,5.2963,5.3221,5.346,5.3711,5.4464,5.4729,5.5024,5.5316,5.5594,5.5862,5.6136,5.6412,5.6688,5.6971,5.7258,5.7544,5.7814,5.8029,5.8076,5.8457,5.8786,5.9139,5.9474,5.9803,6.0116,6.0412,6.0717,6.0978,6.1007,6.1243,6.1508,6.1763,6.1999,6.2233,6.2484,6.2744,6.3004,6.3265,6.3539,6.3824,6.4113,6.4405,6.4696,6.4995,6.5273,6.5559,6.5799,6.6077,6.7548,6.8237,6.8679,6.9399,7.1121,7.1139,7.1295,7.161,7.1916,7.2251,7.2578,7.28,7.3032,7.3193,7.3296,7.3435,7.3591,7.4329,7.4705,7.4991,7.5189,7.5377,7.5607,7.5847,7.6096,7.6353,7.6632,7.6924,7.7228,7.754,7.7865,7.8195,7.8558,7.8935,7.925,7.9517,7.9771,8.0073,8.0396,8.0725,8.1071,8.143,8.1806,8.1806,8.1806,];
float time[100];
float theta1[100];
float theta3[100];
float ktheta1[100];
float ktheta3[100];

//float time[100];
//float theta1[100];
//float theta3[100];
//float ktheta1[100];
//float ktheta3[100];

int i = 0;
float total;
int neg = 1;
int counter;
int type = 0;
char enter = 13;


//instantiate an instance of 
Motor_control_jk mc=Motor_control_jk();
// instantiate a second instance of motor control for ktheta
Motor_control_jk mc2 = Motor_control_jk();


float tvector_scale = .5;

// Second Motor  
int init_1 = 0;   // analog pin used to connect the potentiometer
int val_1;        // variable to read the value from the analog pin 
int init_2 = 0;   // a
int val_2; 
float start_time = 0;//the millis() value where the trial starts.
bool go = false;
int gopin = 5;//this is the pin for a switch to start the trial.
//int theta_1 = 0;
//int theta_3 = 0; 
int L1 = 5;
int L3 = 3;

/// Stuff for filtering force sensor readings.
float tau = 0.030; //seconds, filter time constant
float dT = 0.01;// seconds, initial guess of dT
float alpha = .5; // guess for filter constant.
float sensorValue1_smooth = 0;
float sensorValue1_smooth_old = 0;
float sensorValue2_smooth = 0;
float sensorValue2_smooth_old = 0;
float sensorValue3_smooth = 0;
float sensorValue3_smooth_old = 0;

float floatpos_smooth = 0;
float floatpos_smooth_old = 0;

long reading_sum1 = 0;
long reading_sum2 = 0;
int zero_offset1;
int zero_offset2;

const float pi = 3.14159;
float ftheta_1= 0;
float ftheta_3= 0;
float currtime = 0;
float pos1 = 0;
float pos2 = 0;
float Fx;
float Fy;
float voltage1x;
float voltage2x;
float voltage3x;
float voltage2y;
float voltage3y;
float netx;
float nety;
float theta1star = 0;
float theta3star = 0;
float e1sqt = 0;
float e3sqt = 0;
float e_total = 0;
float T1 = 0;
float T3 = 0;
float fktheta1 = 0;
float fktheta3 = 0;


//for pausing time...
float tottheta1 = 0;
float tottheta3 = 0;
float e1 = 0;
float e3 = 0;

float ethresh = 1.3;//if error is greater than this, we will PAUSE TIME...



// intialize variables for LCD display

const int buttonPinLeft = 4;      // pin for the Up button
const int buttonPinRight = 5;    // pin for the Down button
const int buttonPinEsc = 6;     // pin for the Esc button
const int buttonPinEnter = 7;   // pin for the Enter button

int lastButtonPushed = 0;

int lastButtonEnterState = LOW;   // the previous reading from the Enter input pin
int lastButtonEscState = LOW;   // the previous reading from the Esc input pin
int lastButtonLeftState = LOW;   // the previous reading from the Left input pin
int lastButtonRightState = LOW;   // the previous reading from the Right input pin

long lastEnterDebounceTime = 0;  // the last time the output pin was toggled
long lastEscDebounceTime = 0;  // the last time the output pin was toggled
long lastLeftDebounceTime = 0;  // the last time the output pin was toggled
long lastRightDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 500;    // the debounce time

int letter = 1;
int level = 1;
String dataString = "";

// LiquidCrystal display with:
// rs on pin 2
// rw on 3
// enable on pin 6
// d4, d5, d6, d7 on pins 4, 5, 6, 7

LiquidCrystal lcd(22, 24, 26, 28, 30, 32);

// instantiate a instance of menubackend
MenuBackend menu = MenuBackend(menuUsed,menuChanged);
// initialize menu items
    MenuItem menu1Item1 = MenuItem("Pick a letter");
      MenuItem menuItem1SubItem1 = MenuItem("a");
        MenuItem menuItem1SubSubItem1 = MenuItem("Pick a correction");
              MenuItem menuItem1SubSubSubItem1 = MenuItem("1");
              MenuItem menuItem1SubSubSubItem2 = MenuItem("2");
              MenuItem menuItem1SubSubSubItem3 = MenuItem("3");
              MenuItem menuItem1SubSubSubItem4 = MenuItem("4");
              MenuItem menuItem1SubSubSubItem5 = MenuItem("5");
      MenuItem menuItem1SubItem2 = MenuItem("b");
         MenuItem menuItem1SubSubItem2 = MenuItem("Pick a correction");
              MenuItem menuItem2SubSubSubItem1 = MenuItem("1");
              MenuItem menuItem2SubSubSubItem2 = MenuItem("2");
              MenuItem menuItem2SubSubSubItem3 = MenuItem("3");
              MenuItem menuItem2SubSubSubItem4 = MenuItem("4");
              MenuItem menuItem2SubSubSubItem5 = MenuItem("5");
      MenuItem menuItem1SubItem3 = MenuItem("c");
          MenuItem menuItem1SubSubItem3 = MenuItem("Pick a correction");
              MenuItem menuItem3SubSubSubItem1 = MenuItem("1");
              MenuItem menuItem3SubSubSubItem2 = MenuItem("2");
              MenuItem menuItem3SubSubSubItem3 = MenuItem("3");
              MenuItem menuItem3SubSubSubItem4 = MenuItem("4");
              MenuItem menuItem3SubSubSubItem5 = MenuItem("5");
      MenuItem menuItem1SubItem4 = MenuItem("d");
          MenuItem menuItem1SubSubItem4 = MenuItem("Pick a correction");
              MenuItem menuItem4SubSubSubItem1 = MenuItem("1");
              MenuItem menuItem4SubSubSubItem2 = MenuItem("2");
              MenuItem menuItem4SubSubSubItem3 = MenuItem("3");
              MenuItem menuItem4SubSubSubItem4 = MenuItem("4");
              MenuItem menuItem4SubSubSubItem5 = MenuItem("5");
      MenuItem menuItem1SubItem5 = MenuItem("e");
          MenuItem menuItem1SubSubItem5 = MenuItem("Pick a correction");
              MenuItem menuItem5SubSubSubItem1 = MenuItem("1");
              MenuItem menuItem5SubSubSubItem2 = MenuItem("2");
              MenuItem menuItem5SubSubSubItem3 = MenuItem("3");
              MenuItem menuItem5SubSubSubItem4 = MenuItem("4");
              MenuItem menuItem5SubSubSubItem5 = MenuItem("5");
      MenuItem menuItem1SubItem6 = MenuItem("f");
          MenuItem menuItem1SubSubItem6 = MenuItem("Pick a correction");
              MenuItem menuItem6SubSubSubItem1 = MenuItem("1");
              MenuItem menuItem6SubSubSubItem2 = MenuItem("2");
              MenuItem menuItem6SubSubSubItem3 = MenuItem("3");
              MenuItem menuItem6SubSubSubItem4 = MenuItem("4");
              MenuItem menuItem6SubSubSubItem5 = MenuItem("5");
      MenuItem menuItem1SubItem7 = MenuItem("g");
          MenuItem menuItem1SubSubItem7 = MenuItem("Pick a correction");
              MenuItem menuItem7SubSubSubItem1 = MenuItem("1");
              MenuItem menuItem7SubSubSubItem2 = MenuItem("2");
              MenuItem menuItem7SubSubSubItem3 = MenuItem("3");
              MenuItem menuItem7SubSubSubItem4 = MenuItem("4");
              MenuItem menuItem7SubSubSubItem5 = MenuItem("5");
      MenuItem menuItem1SubItem8 = MenuItem("h");
          MenuItem menuItem1SubSubItem8 = MenuItem("Pick a correction");
              MenuItem menuItem8SubSubSubItem1 = MenuItem("1");
              MenuItem menuItem8SubSubSubItem2 = MenuItem("2");
              MenuItem menuItem8SubSubSubItem3 = MenuItem("3");
              MenuItem menuItem8SubSubSubItem4 = MenuItem("4");
              MenuItem menuItem8SubSubSubItem5 = MenuItem("5");
      MenuItem menuItem1SubItem9 = MenuItem("i");
          MenuItem menuItem1SubSubItem9 = MenuItem("Pick a correction");
              MenuItem menuItem9SubSubSubItem1 = MenuItem("1");
              MenuItem menuItem9SubSubSubItem2 = MenuItem("2");
              MenuItem menuItem9SubSubSubItem3 = MenuItem("3");
              MenuItem menuItem9SubSubSubItem4 = MenuItem("4");
              MenuItem menuItem9SubSubSubItem5 = MenuItem("5");
      MenuItem menuItem1SubItem10 = MenuItem("j");
          MenuItem menuItem1SubSubItem10 = MenuItem("Pick a correction");
              MenuItem menuItem10SubSubSubItem1 = MenuItem("1");
              MenuItem menuItem10SubSubSubItem2 = MenuItem("2");
              MenuItem menuItem10SubSubSubItem3 = MenuItem("3");
              MenuItem menuItem10SubSubSubItem4 = MenuItem("4");
              MenuItem menuItem10SubSubSubItem5 = MenuItem("5");
      MenuItem menuItem1SubItem11 = MenuItem("k");
          MenuItem menuItem1SubSubItem11 = MenuItem("Pick a correction");
              MenuItem menuItem11SubSubSubItem1 = MenuItem("1");
              MenuItem menuItem11SubSubSubItem2 = MenuItem("2");
              MenuItem menuItem11SubSubSubItem3 = MenuItem("3");
              MenuItem menuItem11SubSubSubItem4 = MenuItem("4");
              MenuItem menuItem11SubSubSubItem5 = MenuItem("5");
      MenuItem menuItem1SubItem12 = MenuItem("l");
          MenuItem menuItem1SubSubItem12 = MenuItem("Pick a correction");
              MenuItem menuItem12SubSubSubItem1 = MenuItem("1");
              MenuItem menuItem12SubSubSubItem2 = MenuItem("2");
              MenuItem menuItem12SubSubSubItem3 = MenuItem("3");
              MenuItem menuItem12SubSubSubItem4 = MenuItem("4");
              MenuItem menuItem12SubSubSubItem5 = MenuItem("5");
      MenuItem menuItem1SubItem13 = MenuItem("m");
          MenuItem menuItem1SubSubItem13 = MenuItem("Pick a correction");
              MenuItem menuItem13SubSubSubItem1 = MenuItem("1");
              MenuItem menuItem13SubSubSubItem2 = MenuItem("2");
              MenuItem menuItem13SubSubSubItem3 = MenuItem("3");
              MenuItem menuItem13SubSubSubItem4 = MenuItem("4");
              MenuItem menuItem13SubSubSubItem5 = MenuItem("5");
      MenuItem menuItem1SubItem14 = MenuItem("n");
          MenuItem menuItem1SubSubItem14 = MenuItem("Pick a correction");
              MenuItem menuItem14SubSubSubItem1 = MenuItem("1");
              MenuItem menuItem14SubSubSubItem2 = MenuItem("2");
              MenuItem menuItem14SubSubSubItem3 = MenuItem("3");
              MenuItem menuItem14SubSubSubItem4 = MenuItem("4");
              MenuItem menuItem14SubSubSubItem5 = MenuItem("5");
      MenuItem menuItem1SubItem15 = MenuItem("o");
          MenuItem menuItem1SubSubItem15 = MenuItem("Pick a correction");
              MenuItem menuItem15SubSubSubItem1 = MenuItem("1");
              MenuItem menuItem15SubSubSubItem2 = MenuItem("2");
              MenuItem menuItem15SubSubSubItem3 = MenuItem("3");
              MenuItem menuItem15SubSubSubItem4 = MenuItem("4");
              MenuItem menuItem15SubSubSubItem5 = MenuItem("5");
      MenuItem menuItem1SubItem16 = MenuItem("p");
          MenuItem menuItem1SubSubItem16 = MenuItem("Pick a correction");
              MenuItem menuItem16SubSubSubItem1 = MenuItem("1");
              MenuItem menuItem16SubSubSubItem2 = MenuItem("2");
              MenuItem menuItem16SubSubSubItem3 = MenuItem("3");
              MenuItem menuItem16SubSubSubItem4 = MenuItem("4");
              MenuItem menuItem16SubSubSubItem5 = MenuItem("5");
      MenuItem menuItem1SubItem17 = MenuItem("q");
          MenuItem menuItem1SubSubItem17 = MenuItem("Pick a correction");
              MenuItem menuItem17SubSubSubItem1 = MenuItem("1");
              MenuItem menuItem17SubSubSubItem2 = MenuItem("2");
              MenuItem menuItem17SubSubSubItem3 = MenuItem("3");
              MenuItem menuItem17SubSubSubItem4 = MenuItem("4");
              MenuItem menuItem17SubSubSubItem5 = MenuItem("5");
      MenuItem menuItem1SubItem18 = MenuItem("r");
          MenuItem menuItem1SubSubItem18 = MenuItem("Pick a correction");
              MenuItem menuItem18SubSubSubItem1 = MenuItem("1");
              MenuItem menuItem18SubSubSubItem2 = MenuItem("2");
              MenuItem menuItem18SubSubSubItem3 = MenuItem("3");
              MenuItem menuItem18SubSubSubItem4 = MenuItem("4");
              MenuItem menuItem18SubSubSubItem5 = MenuItem("5");
      MenuItem menuItem1SubItem19 = MenuItem("s");
          MenuItem menuItem1SubSubItem19 = MenuItem("Pick a correction");
              MenuItem menuItem19SubSubSubItem1 = MenuItem("1");
              MenuItem menuItem19SubSubSubItem2 = MenuItem("2");
              MenuItem menuItem19SubSubSubItem3 = MenuItem("3");
              MenuItem menuItem19SubSubSubItem4 = MenuItem("4");
              MenuItem menuItem19SubSubSubItem5 = MenuItem("5");
      MenuItem menuItem1SubItem20 = MenuItem("t");
          MenuItem menuItem1SubSubItem20 = MenuItem("Pick a correction");
              MenuItem menuItem20SubSubSubItem1 = MenuItem("1");
              MenuItem menuItem20SubSubSubItem2 = MenuItem("2");
              MenuItem menuItem20SubSubSubItem3 = MenuItem("3");
              MenuItem menuItem20SubSubSubItem4 = MenuItem("4");
              MenuItem menuItem20SubSubSubItem5 = MenuItem("5");
      MenuItem menuItem1SubItem21 = MenuItem("u");
           MenuItem menuItem1SubSubItem21 = MenuItem("Pick a correction");
              MenuItem menuItem21SubSubSubItem1 = MenuItem("1");
              MenuItem menuItem21SubSubSubItem2 = MenuItem("2");
              MenuItem menuItem21SubSubSubItem3 = MenuItem("3");
              MenuItem menuItem21SubSubSubItem4 = MenuItem("4");
              MenuItem menuItem21SubSubSubItem5 = MenuItem("5");
      MenuItem menuItem1SubItem22 = MenuItem("v");
           MenuItem menuItem1SubSubItem22 = MenuItem("Pick a correction");
              MenuItem menuItem22SubSubSubItem1 = MenuItem("1");
              MenuItem menuItem22SubSubSubItem2 = MenuItem("2");
              MenuItem menuItem22SubSubSubItem3 = MenuItem("3");
              MenuItem menuItem22SubSubSubItem4 = MenuItem("4");
              MenuItem menuItem22SubSubSubItem5 = MenuItem("5");
      MenuItem menuItem1SubItem23 = MenuItem("w");
           MenuItem menuItem1SubSubItem23 = MenuItem("Pick a correction");
              MenuItem menuItem23SubSubSubItem1 = MenuItem("1");
              MenuItem menuItem23SubSubSubItem2 = MenuItem("2");
              MenuItem menuItem23SubSubSubItem3 = MenuItem("3");
              MenuItem menuItem23SubSubSubItem4 = MenuItem("4");
              MenuItem menuItem23SubSubSubItem5 = MenuItem("5");
      MenuItem menuItem1SubItem24 = MenuItem("x");
           MenuItem menuItem1SubSubItem24 = MenuItem("Pick a correction");
              MenuItem menuItem24SubSubSubItem1 = MenuItem("1");
              MenuItem menuItem24SubSubSubItem2 = MenuItem("2");
              MenuItem menuItem24SubSubSubItem3 = MenuItem("3");
              MenuItem menuItem24SubSubSubItem4 = MenuItem("4");
              MenuItem menuItem24SubSubSubItem5 = MenuItem("5");
      MenuItem menuItem1SubItem25 = MenuItem("y");
           MenuItem menuItem1SubSubItem25 = MenuItem("Pick a correction");
              MenuItem menuItem25SubSubSubItem1 = MenuItem("1");
              MenuItem menuItem25SubSubSubItem2 = MenuItem("2");
              MenuItem menuItem25SubSubSubItem3 = MenuItem("3");
              MenuItem menuItem25SubSubSubItem4 = MenuItem("4");
              MenuItem menuItem25SubSubSubItem5 = MenuItem("5");
      MenuItem menuItem1SubItem26 = MenuItem("z");
           MenuItem menuItem1SubSubItem26 = MenuItem("Pick a correction");
              MenuItem menuItem26SubSubSubItem1 = MenuItem("1");
              MenuItem menuItem26SubSubSubItem2 = MenuItem("2");
              MenuItem menuItem26SubSubSubItem3 = MenuItem("3");
              MenuItem menuItem26SubSubSubItem4 = MenuItem("4");
              MenuItem menuItem26SubSubSubItem5 = MenuItem("5");

void setup()
{
  Serial.begin(115200);

  //you have to manually attach interrupts for each encoder (1,2,..)
  attachInterrupt(enc1._interrupt1,e1_ISRA,CHANGE);
  attachInterrupt(enc1._interrupt2,e1_ISRB,CHANGE);
  attachInterrupt(enc2._interrupt1,e2_ISRA,CHANGE);
  attachInterrupt(enc2._interrupt2,e2_ISRB,CHANGE);
  
  //mc = new Motor_control_jk;

  // LCD setup
  pinMode(buttonPinLeft, INPUT);
  pinMode(buttonPinRight, INPUT);
  pinMode(buttonPinEnter, INPUT);
  pinMode(buttonPinEsc, INPUT);
  
  lcd.begin(16, 2);

  //configure menu
  menu.getRoot().add(menu1Item1);
//  menu1Item1.addRight(menu1Item2);
  //.addRight(menu1Item3);
  
  // Letter
  menu1Item1.add(menuItem1SubItem1).add(menuItem1SubSubItem1).add(menuItem1SubSubSubItem1).addRight(menuItem1SubSubSubItem2)\
  .addRight(menuItem1SubSubSubItem3).addRight(menuItem1SubSubSubItem4).addRight(menuItem1SubSubSubItem5);
  
  menuItem1SubItem1.addRight(menuItem1SubItem2).add(menuItem1SubSubItem2).add(menuItem2SubSubSubItem1).addRight(menuItem2SubSubSubItem2)\
  .addRight(menuItem2SubSubSubItem3).addRight(menuItem2SubSubSubItem4).addRight(menuItem2SubSubSubItem5);
  
  menuItem1SubItem2.addRight(menuItem1SubItem3).add(menuItem1SubSubItem3).add(menuItem3SubSubSubItem1).addRight(menuItem3SubSubSubItem2)\
  .addRight(menuItem3SubSubSubItem3).addRight(menuItem3SubSubSubItem4).addRight(menuItem3SubSubSubItem5);
  
  menuItem1SubItem3.addRight(menuItem1SubItem4).add(menuItem1SubSubItem4).add(menuItem4SubSubSubItem1).addRight(menuItem4SubSubSubItem2)\
  .addRight(menuItem4SubSubSubItem3).addRight(menuItem4SubSubSubItem4).addRight(menuItem4SubSubSubItem5);
  
  menuItem1SubItem4.addRight(menuItem1SubItem5).add(menuItem1SubSubItem5).add(menuItem5SubSubSubItem1).addRight(menuItem5SubSubSubItem2)\
  .addRight(menuItem5SubSubSubItem3).addRight(menuItem5SubSubSubItem4).addRight(menuItem5SubSubSubItem5);
  
  menuItem1SubItem5.addRight(menuItem1SubItem6).add(menuItem1SubSubItem6).add(menuItem6SubSubSubItem1).addRight(menuItem6SubSubSubItem2)\
  .addRight(menuItem6SubSubSubItem3).addRight(menuItem6SubSubSubItem4).addRight(menuItem6SubSubSubItem5);
  
  menuItem1SubItem6.addRight(menuItem1SubItem7).add(menuItem1SubSubItem7).add(menuItem7SubSubSubItem1).addRight(menuItem7SubSubSubItem2)\
  .addRight(menuItem7SubSubSubItem3).addRight(menuItem7SubSubSubItem4).addRight(menuItem7SubSubSubItem5);
  
  menuItem1SubItem7.addRight(menuItem1SubItem8).add(menuItem1SubSubItem8).add(menuItem8SubSubSubItem1).addRight(menuItem8SubSubSubItem2)\
  .addRight(menuItem8SubSubSubItem3).addRight(menuItem8SubSubSubItem4).addRight(menuItem8SubSubSubItem5);
  
  menuItem1SubItem8.addRight(menuItem1SubItem9).add(menuItem1SubSubItem9).add(menuItem9SubSubSubItem1).addRight(menuItem9SubSubSubItem2)\
  .addRight(menuItem9SubSubSubItem3).addRight(menuItem9SubSubSubItem4).addRight(menuItem9SubSubSubItem5);
  
  menuItem1SubItem9.addRight(menuItem1SubItem10).add(menuItem1SubSubItem10).add(menuItem10SubSubSubItem1).addRight(menuItem10SubSubSubItem2)\
  .addRight(menuItem10SubSubSubItem3).addRight(menuItem10SubSubSubItem4).addRight(menuItem10SubSubSubItem5);
  
  menuItem1SubItem10.addRight(menuItem1SubItem11).add(menuItem1SubSubItem11).add(menuItem11SubSubSubItem1).addRight(menuItem11SubSubSubItem2)\
  .addRight(menuItem11SubSubSubItem3).addRight(menuItem11SubSubSubItem4).addRight(menuItem11SubSubSubItem5);
  
  menuItem1SubItem11.addRight(menuItem1SubItem12).add(menuItem1SubSubItem12).add(menuItem12SubSubSubItem1).addRight(menuItem12SubSubSubItem2)\
  .addRight(menuItem12SubSubSubItem3).addRight(menuItem12SubSubSubItem4).addRight(menuItem12SubSubSubItem5);
  
  menuItem1SubItem12.addRight(menuItem1SubItem13).add(menuItem1SubSubItem13).add(menuItem13SubSubSubItem1).addRight(menuItem13SubSubSubItem2)\
  .addRight(menuItem13SubSubSubItem3).addRight(menuItem13SubSubSubItem4).addRight(menuItem13SubSubSubItem5);
  
  menuItem1SubItem13.addRight(menuItem1SubItem14).add(menuItem1SubSubItem14).add(menuItem14SubSubSubItem1).addRight(menuItem14SubSubSubItem2)\
  .addRight(menuItem14SubSubSubItem3).addRight(menuItem14SubSubSubItem4).addRight(menuItem14SubSubSubItem5);
  
  menuItem1SubItem14.addRight(menuItem1SubItem15).add(menuItem1SubSubItem15).add(menuItem15SubSubSubItem1).addRight(menuItem15SubSubSubItem2)\
  .addRight(menuItem15SubSubSubItem3).addRight(menuItem15SubSubSubItem4).addRight(menuItem15SubSubSubItem5);
  
  menuItem1SubItem15.addRight(menuItem1SubItem16).add(menuItem1SubSubItem16).add(menuItem16SubSubSubItem1).addRight(menuItem16SubSubSubItem2)\
  .addRight(menuItem16SubSubSubItem3).addRight(menuItem16SubSubSubItem4).addRight(menuItem16SubSubSubItem5);
  
  menuItem1SubItem16.addRight(menuItem1SubItem17).add(menuItem1SubSubItem17).add(menuItem17SubSubSubItem1).addRight(menuItem17SubSubSubItem2)\
  .addRight(menuItem17SubSubSubItem3).addRight(menuItem17SubSubSubItem4).addRight(menuItem17SubSubSubItem5);
  
  menuItem1SubItem17.addRight(menuItem1SubItem18).add(menuItem1SubSubItem18).add(menuItem18SubSubSubItem1).addRight(menuItem18SubSubSubItem2)\
  .addRight(menuItem18SubSubSubItem3).addRight(menuItem18SubSubSubItem4).addRight(menuItem18SubSubSubItem5);
  
  menuItem1SubItem18.addRight(menuItem1SubItem19).add(menuItem1SubSubItem19).add(menuItem19SubSubSubItem1).addRight(menuItem19SubSubSubItem2)\
  .addRight(menuItem19SubSubSubItem3).addRight(menuItem19SubSubSubItem4).addRight(menuItem19SubSubSubItem5);
  
  menuItem1SubItem19.addRight(menuItem1SubItem20).add(menuItem1SubSubItem20).add(menuItem20SubSubSubItem1).addRight(menuItem20SubSubSubItem2)\
  .addRight(menuItem20SubSubSubItem3).addRight(menuItem20SubSubSubItem4).addRight(menuItem20SubSubSubItem5);
  
  menuItem1SubItem20.addRight(menuItem1SubItem21).add(menuItem1SubSubItem21).add(menuItem21SubSubSubItem1).addRight(menuItem21SubSubSubItem2)\
  .addRight(menuItem21SubSubSubItem3).addRight(menuItem21SubSubSubItem4).addRight(menuItem21SubSubSubItem5);
  
  menuItem1SubItem21.addRight(menuItem1SubItem22).add(menuItem1SubSubItem22).add(menuItem22SubSubSubItem1).addRight(menuItem22SubSubSubItem2)\
  .addRight(menuItem22SubSubSubItem3).addRight(menuItem22SubSubSubItem4).addRight(menuItem22SubSubSubItem5);
  
  menuItem1SubItem22.addRight(menuItem1SubItem23).add(menuItem1SubSubItem23).add(menuItem23SubSubSubItem1).addRight(menuItem23SubSubSubItem2)\
  .addRight(menuItem23SubSubSubItem3).addRight(menuItem23SubSubSubItem4).addRight(menuItem23SubSubSubItem5);
  
  menuItem1SubItem23.addRight(menuItem1SubItem24).add(menuItem1SubSubItem24).add(menuItem24SubSubSubItem1).addRight(menuItem24SubSubSubItem2)\
  .addRight(menuItem24SubSubSubItem3).addRight(menuItem24SubSubSubItem4).addRight(menuItem24SubSubSubItem5);
  
  menuItem1SubItem24.addRight(menuItem1SubItem25).add(menuItem1SubSubItem25).add(menuItem25SubSubSubItem1).addRight(menuItem25SubSubSubItem2)\
  .addRight(menuItem25SubSubSubItem3).addRight(menuItem25SubSubSubItem4).addRight(menuItem25SubSubSubItem5);
  
  menuItem1SubItem25.addRight(menuItem1SubItem26).add(menuItem1SubSubItem26).add(menuItem26SubSubSubItem1).addRight(menuItem26SubSubSubItem2)\
  .addRight(menuItem26SubSubSubItem3).addRight(menuItem26SubSubSubItem4).addRight(menuItem26SubSubSubItem5);

  menu.toRoot();
  lcd.setCursor(0,0);  
  lcd.print("HARDO VO.1");
  
  
  Serial.print("Initializing SD card...");
  // On the Ethernet Shield, CS is pin 4. It's set as an output by default.
  // Note that even if it's not used as the CS pin, the hardware SS pin
  // (10 on most Arduino boards, 53 on the Mega) must be left as an output
  // or the SD library functions will not work.
  pinMode(53, OUTPUT);
  
  if (!SD.begin(53)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

}

void loop(){


// LCD Display 
  analogWrite(9,125); 
  readButtons();  //I splitted button reading and navigation in two procedures because 
  navigateMenus();  //in some situations I want to use the button for other purpose (eg. to change some settings)
  // new stuff
  
go = digitalRead(gopin);
 
 float realtime = millis()/1000.0;
    
  int sensorValue1 = analogRead(A0);
  int sensorValue2 = analogRead(A1);
  int sensorValue3 = analogRead(A2);
  
  // We are going to try to use a low pass filter to smooth out the sensor readings
  // RC = tau = dT*(1-alpha)/alpha where alpha is between 0 and 1
  // tau is at the top of this file
  alpha = 1/(tau/dT+1);
  sensorValue1_smooth = sensorValue1_smooth_old + alpha*(sensorValue1-sensorValue1_smooth_old); 
  sensorValue1_smooth_old = sensorValue1_smooth;
  sensorValue2_smooth = sensorValue2_smooth_old + alpha*(sensorValue2-sensorValue2_smooth_old); 
  sensorValue2_smooth_old = sensorValue2_smooth;
  sensorValue3_smooth = sensorValue3_smooth_old + alpha*(sensorValue3-sensorValue3_smooth_old); 
  sensorValue3_smooth_old = sensorValue3_smooth;
  
  if(abs(e1)<=ethresh && abs(e3)<=ethresh){
  currtime = float(millis())*.001-start_time;
  currtime = currtime*tvector_scale;//this scales the time to slow down the letter.
}
else{
  Serial.println("TIME IS PAUSED DUDE");
}
  //currtime = 5.02;
//  Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  voltage1x = float(sensorValue1_smooth)*(5/1023.0);
  voltage2x = 0.5*float(sensorValue2_smooth)*(5/1023.0);
  voltage3x = 0.5*float(sensorValue3_smooth)*(5/1023.0);
  voltage2y = (sqrt(3)/2)*float(sensorValue2_smooth)*(5/1023.0);
  voltage3y = (sqrt(3)/2)*float(sensorValue3_smooth)*(5/1023.0);
  Fx = voltage1x-voltage2x-voltage3x;
  Fy = voltage2y-voltage3y;

//    func(letter);      // choose a number from 1-26 for letter.  <---- Moved to whenever the letter is selected

    //old stuff
    mc.findMotor_position(theta1, theta3, time, currtime);
    float ftheta_1 = mc.ftheta_1;
    float ftheta_3 = mc.ftheta_3;
    mc2.findMotor_position(ktheta1, ktheta3, time, currtime);
    float fktheta1 = mc2.ftheta_1;
    float fktheta3 = mc2.ftheta_3;

    // new stuff

    T1 = Fx*sin(ftheta_1) - Fy*cos(ftheta_1)*L1;
    T3 = -Fx*L3*cos(ftheta_3-pi/2) - Fy*L3*sin(ftheta_3-pi/2);

    float k = 0;//0.015; //-0.015 to 0.015 is our continuum for stopped position.

    theta1star = T1*k/(fktheta1);
    theta3star = T3*k/(fktheta3);
    
// old stuff

    pos1 = enc1.readPosRad();
    pos2 = enc2.readPosRad();
   //query the encoder object for its velocity in rad/s
   float omega1  = enc1.readVelRad();        
   float omega2 = enc2.readVelRad();
  float e1 = tottheta1-pos1;//total error now ADDED BY AAB 6/8/2015
  float e3 = tottheta3-pos2;
  float tottheta1 = -ftheta_1-theta1star;//TOTAL angle sent as command to motor 1 ADDED BY AAB 6/8/2015
  float tottheta3 = ftheta_3-(pi/2)+theta3star;
//use the PIDUINO library to write the command.
  servo1.writePID(tottheta1,pos1,omega1);
  servo2.writePID(tottheta3,pos2,omega2); 
  e1sqt += e1;
  e3sqt += e3;
  e_total = e1sqt + e3sqt; 

  
 // }  
 Serial.print(currtime);
 Serial.print("\t");
Serial.print(realtime,4);
Serial.print("\t");
Serial.print(ftheta_1,4);
Serial.print("\t");
 Serial.print(pos1,4);
 Serial.print("\t");
Serial.print(ftheta_3,4);
Serial.print("\t");
 Serial.print(pos2,4);
 Serial.print("\t");
Serial.print(e1,4);
 Serial.print("\t");
 Serial.print(e3,4);
 Serial.print("\t");

Serial.println();
delay(1);

}

void func(int lett) {
  
  
  for( int p = 0; p < sizeof(time);  p++ ) {
   time[i] = (float)0;
  }
  for( int p = 0; p < sizeof(theta1);  p++ ) {
   theta1[i] = (float)0;
  }
  for( int p = 0; p < sizeof(theta3);  p++ ) {
   theta3[i] = (float)0;
  }
  for( int p = 0; p < sizeof(ktheta1);  p++ ) {
   ktheta1[i] = (float)0;
  }
  for( int p = 0; p < sizeof(ktheta3);  p++ ) {
   ktheta3[i] = (float)0;
  }
  
  char fileStr[6] = {lett+96,'.','t','x','t','\0'};
  myFile = SD.open(fileStr,FILE_READ);

  // re-open the file for reading:
//  myFile = SD.open("a.txt",FILE_READ);
  
  if (myFile) {
//    Serial.println("test.txt:");

    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      
      if(myFile.peek()=='-'){ //if next is a negative
        myFile.read();
        neg = -1; //this is to multiply final total to get negative
      }
        
       total = myFile.read() - zero; //This is the first number
       
       if (myFile.peek()!='.') {
         total = (total * 10) + (myFile.read()-zero);
       }
      
      
      if(myFile.peek()!=','){
      myFile.read(); //jumping over period
      counter = 1;
      
      total = addTotal(total,counter);
      }
      total = total*neg;
      neg = 1;
      //Serial.println(total,5); //print calculated value
      
      i++;
      myFile.read();
      
      //Serial.println(type);
      if(type == 0){
        time[i] = total;
        Serial.print("t: ");
        Serial.println(time[i],5);
      }
      if(type == 1){
        theta1[i] = total;
        Serial.print("t1: ");
        Serial.println(theta1[i],5);
      }
      if(type == 2){
        theta3[i] = total;
        Serial.print("t3: ");
        Serial.println(theta3[i],5);
      }
      if(type == 3){
        ktheta1[i] = total;
        Serial.print("kt1: ");
        Serial.println(ktheta1[i],5);
      }
      if(type == 4){
        ktheta3[i] = total;
        Serial.print("kt3: ");
        Serial.println(ktheta3[i],5);
      }
      if(myFile.peek() == 13){ //If next character is new line character
        myFile.read();
        myFile.read();
        Serial.println();
        Serial.println();
        Serial.println();
        type++; //advance the type of variable
        i = 0; // reset iterative counter
      }
      
      
    }
    
    Serial.println("apparently read the letter");

    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening txt file");
  }
}


float addTotal(float total, int counter){
  
  total = total + (myFile.read()-zero) * pow(0.1,counter);
  
  counter++;
  
  if(myFile.peek()!=','){
    addTotal(total,counter);
  }
  
  else{
    counter = 1;
    return(total);
  }
  
}

void menuChanged(MenuChangeEvent changed){
  
  MenuItem newMenuItem=changed.to; //get the destination menu
  
  lcd.setCursor(0,1); //set the start position for lcd printing to the second row
  
  if(newMenuItem.getName()==menu.getRoot()){
      lcd.print("Main Menu       ");
  }else if(newMenuItem.getName()=="Pick a letter"){
      lcd.print("Pick a letter");
  }else if(newMenuItem.getName()=="a"){
      lcd.print("a              ");
      letter = 1;
  }else if(newMenuItem.getName()=="Pick a correction"){
      lcd.print("Pick a level");
  }else if(newMenuItem.getName()=="b"){
      lcd.print("b               ");
      letter = 2;
  }else if(newMenuItem.getName()=="c"){
      lcd.print("c               ");
      letter = 3;
  }else if(newMenuItem.getName()=="d"){
      lcd.print("d              ");
      letter = 4;
  }else if(newMenuItem.getName()=="e"){
      lcd.print("e               ");
      letter = 5;
  }else if(newMenuItem.getName()=="f"){
      lcd.print("f               ");      
      letter = 6;
  }else if(newMenuItem.getName()=="g"){
      lcd.print("g              ");
      letter = 7;
  }else if(newMenuItem.getName()=="h"){
      lcd.print("h               ");
      letter = 8;
  }else if(newMenuItem.getName()=="i"){
      lcd.print("i               ");
      letter = 9;
  }else if(newMenuItem.getName()=="j"){
      lcd.print("j               ");
      letter = 10;
  }else if(newMenuItem.getName()=="k"){
      lcd.print("k               ");
      letter = 11;
  }else if(newMenuItem.getName()=="l"){
      lcd.print("l               ");
      letter = 12;
  }else if(newMenuItem.getName()=="m"){
      lcd.print("m               ");
      letter = 13;
  }else if(newMenuItem.getName()=="n"){
      lcd.print("n              ");
      letter = 14;
  }else if(newMenuItem.getName()=="o"){
      lcd.print("o               ");
      letter = 15;
  }else if(newMenuItem.getName()=="p"){
      lcd.print("p               ");
      letter = 16;
  }else if(newMenuItem.getName()=="q"){
      lcd.print("q               ");
      letter = 17;
  }else if(newMenuItem.getName()=="r"){
      lcd.print("r               ");
      letter = 18;
  }else if(newMenuItem.getName()=="s"){
      lcd.print("s               ");
      letter = 19;
  }else if(newMenuItem.getName()=="t"){
      lcd.print("t               ");
      letter = 20;
  }else if(newMenuItem.getName()=="u"){
      lcd.print("u               ");
      letter = 21;
  }else if(newMenuItem.getName()=="v"){
      lcd.print("v               ");
      letter = 22;
  }else if(newMenuItem.getName()=="w"){
      lcd.print("w               ");
      letter = 23;
   }else if(newMenuItem.getName()=="x"){
      lcd.print("x               ");
      letter = 24;
  }else if(newMenuItem.getName()=="y"){
      lcd.print("y               ");
      letter = 25;
  }else if(newMenuItem.getName()=="z"){
      lcd.print("z               ");  
      letter = 26;
  }else if(newMenuItem.getName()=="1"){
      lcd.print("1               ");
      level = '1';
  }else if(newMenuItem.getName()=="2"){
      lcd.print("2               ");
      level = '2';
  }else if(newMenuItem.getName()=="3"){
      lcd.print("3               ");
      level = '3';
  }else if(newMenuItem.getName()=="4"){
      lcd.print("4            ");
      level = '4';
  }else if(newMenuItem.getName()=="5"){
      lcd.print("5               ");    
      level = '5';
//      Serial.begin(115200);
//      Serial.print(digitalRead(7));
//      func(letter);      // choose a number from 1-26 for letter. COMMENTED BY AAB 6/8/2015
      
  }
}

void menuUsed(MenuUseEvent used){
  lcd.setCursor(0,0);  
  lcd.print("You've Selected   ");
  lcd.setCursor(0,1); 
  //lcd.print(used.item.getName());
  char var = (char)(letter + 96);
  Serial.println(var);
  char stuff[3]= {var,level,'\0'};
  Serial.println(stuff);
  lcd.print(stuff);
  func(letter);      // choose a number from 1-26 for letter. Moved here by AAB 6/8/2015
  delay(5000);  //delay to allow message reading
  start_time = millis()/1000.0;//added by AAB 6/8/2015 to reset trial time.
  lcd.setCursor(0,0);  
  lcd.print("HARDO VO.1        ");
  
  menu.toRoot();  //back to Main
}


void  readButtons(){  //read buttons status
  int reading;
  int buttonEnterState=LOW;             // the current reading from the Enter input pin
  int buttonEscState=LOW;             // the current reading from the input pin
  int buttonLeftState=LOW;             // the current reading from the input pin
  int buttonRightState=LOW;             // the current reading from the input pin

  //Enter button
                  // read the state of the switch into a local variable:
                  reading = digitalRead(buttonPinEnter);

                  // check to see if you just pressed the enter button 
                  // (i.e. the input went from LOW to HIGH),  and you've waited 
                  // long enough since the last press to ignore any noise:  
                
                  // If the switch changed, due to noise or pressing:
                  if (reading != lastButtonEnterState) {
                    // reset the debouncing timer
                    lastEnterDebounceTime = millis();
                  } 
                  
                  if ((millis() - lastEnterDebounceTime) > debounceDelay) {
                    // whatever the reading is at, it's been there for longer
                    // than the debounce delay, so take it as the actual current state:
                    buttonEnterState=reading;
                    lastEnterDebounceTime=millis();
                  }
                  
                  // save the reading.  Next time through the loop,
                  // it'll be the lastButtonState:
                  lastButtonEnterState = reading;
                  

    //Esc button               
                  // read the state of the switch into a local variable:
                  reading = digitalRead(buttonPinEsc);

                  // check to see if you just pressed the Down button 
                  // (i.e. the input went from LOW to HIGH),  and you've waited 
                  // long enough since the last press to ignore any noise:  
                
                  // If the switch changed, due to noise or pressing:
                  if (reading != lastButtonEscState) {
                    // reset the debouncing timer
                    lastEscDebounceTime = millis();
                  } 
                  
                  if ((millis() - lastEscDebounceTime) > debounceDelay) {
                    // whatever the reading is at, it's been there for longer
                    // than the debounce delay, so take it as the actual current state:
                    buttonEscState = reading;
                    lastEscDebounceTime=millis();
                  }
                  
                  // save the reading.  Next time through the loop,
                  // it'll be the lastButtonState:
                  lastButtonEscState = reading; 
                  
                     
   //Down button               
                  // read the state of the switch into a local variable:
                  reading = digitalRead(buttonPinRight);

                  // check to see if you just pressed the Down button 
                  // (i.e. the input went from LOW to HIGH),  and you've waited 
                  // long enough since the last press to ignore any noise:  
                
                  // If the switch changed, due to noise or pressing:
                  if (reading != lastButtonRightState) {
                    // reset the debouncing timer
                    lastRightDebounceTime = millis();
                  }
                  
                  if ((millis() - lastRightDebounceTime) > debounceDelay) {
                    // whatever the reading is at, it's been there for longer
                    // than the debounce delay, so take it as the actual current state:
                    buttonRightState = reading;
                   lastRightDebounceTime =millis();
                  }
                  
                  // save the reading.  Next time through the loop,
                  // it'll be the lastButtonState:
                  lastButtonRightState = reading;                  
                  
                  
    //Up button               
                  // read the state of the switch into a local variable:
                  reading = digitalRead(buttonPinLeft);

                  // check to see if you just pressed the Down button 
                  // (i.e. the input went from LOW to HIGH),  and you've waited 
                  // long enough since the last press to ignore any noise:  
                
                  // If the switch changed, due to noise or pressing:
                  if (reading != lastButtonLeftState) {
                    // reset the debouncing timer
                    lastLeftDebounceTime = millis();
                  } 
                  
                  if ((millis() - lastLeftDebounceTime) > debounceDelay) {
                    // whatever the reading is at, it's been there for longer
                    // than the debounce delay, so take it as the actual current state:
                    buttonLeftState = reading;
                    lastLeftDebounceTime=millis();;
                  }
                  
                  // save the reading.  Next time through the loop,
                  // it'll be the lastButtonState:
                  lastButtonLeftState = reading;  

                  //records which button has been pressed
                  if (buttonEnterState==HIGH){
                    lastButtonPushed=buttonPinEnter;

                  }else if(buttonEscState==HIGH){
                    lastButtonPushed=buttonPinEsc;

                  }else if(buttonRightState==HIGH){
                    lastButtonPushed=buttonPinRight;

                  }else if(buttonLeftState==HIGH){
                    lastButtonPushed=buttonPinLeft;

                  }else{
                    lastButtonPushed=0;
                  }                  
}

void navigateMenus() {
  MenuItem currentMenu=menu.getCurrent();
  
  switch (lastButtonPushed){
    case buttonPinEnter:
      if(!(currentMenu.moveDown())){  //if the current menu has a child and has been pressed enter then menu navigate to item below
        menu.use();
      }else{  //otherwise, if menu has no child and has been pressed enter the current menu is used
        menu.moveDown();
       }
      break;
    case buttonPinEsc:
      menu.toRoot();  //back to main
      break;
    case buttonPinRight:
      menu.moveRight();
      break;      
    case buttonPinLeft:
      menu.moveLeft();
      break;      
  }
  
  lastButtonPushed=0; //reset the lastButtonPushed variable
}

void e1_ISRA(){
  enc1.channelA();
}
void e1_ISRB(){
  enc1.channelB();
}
 
void e2_ISRA(){
  enc2.channelA();
}
void e2_ISRB(){
  enc2.channelB();
}
 



