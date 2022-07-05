#include "Arduino.h"
#include "teensy_clock.h"

#include <stdlib.h>
#include <Encoder.h>
#define sync 19

using namespace std::chrono;
using timePoint = teensy_clock::time_point; // alias to save typing...


#include "CustomDS.h"
#include "SerialReader.h"
#include "Wire.h"
#include "MPU6050_light_modified.h"
#include "variable.h"
#include "datacomm.h"

Encoder wheel_fr(4, 5); // front right wheel
Encoder wheel_fl(7, 6); // front left wheel
Encoder wheel_rr(10, 11); // rear right wheel
Encoder wheel_rl(9, 8); // rear left wheel



//   avoid using pins with LEDs attached
void setup() {
  Serial1.begin(115200);
  Serial.begin(115200);
  //  Serial.begin(9600);
  teensy_clock::begin(); // this will sync the teensy_clock to the RTC

  Wire.begin();

  pinMode(sync, INPUT);

}

long old_p_fr  = -999;
long old_p_fl  = -999;
long old_p_rr  = -999;
long old_p_rl  = -999;

double a_fr = 0;
double a_fl = 0;
double a_rr = 0;
double a_rl = 0;

float pos_val[2] = {0, 0};


void loop() {


  timePoint now = teensy_clock::now(); // get the current time from the teensy clock with a resolution of 1.67ns (1/600MHz)

  // currently (c++14) std::chrono  doesn't provide much support for transformation of chrono::timepoints to
  // years, day of month etc. c++20 would improve this a lot, but alas...
  // Meanwhile we can work around using the traditional time_t and struct tm to handle this

  time_t rawTime = teensy_clock::to_time_t(now); // convert the std::chrono time_point to a traditional time_t value

  _rawTime = rawTime;
  _mils = millis();

  // getting trigger pulse from motion capture
  _s = digitalRead(sync);
  if (_s == 0) {
    _sync = '0';
  }

  if (_s == 1) {

    _sync = '1';
  }
  Serial.println(_s);

  // encoder program below

  p_fr = wheel_fr.read();
  if (p_fr != old_p_fr) {
    old_p_fr = p_fr;
  }

  p_fl = wheel_fl.read();
  if (p_fl != old_p_fl) {
    old_p_fl = p_fl;
  }

  p_rr = wheel_rr.read();
  if (p_rr != old_p_rr) {
    old_p_rr = p_rr;
  }
  p_rl = wheel_rl.read();
  if (p_rl != old_p_rl) {
    old_p_rl = p_rl;
  }

  // finding angles

  a_fr = p_fr * 0.09;
  a_fl = p_fl * 0.09;
  a_rr = p_rr * 0.09;
  a_rl = p_rl * 0.09;


  //  Serial.print(p_fr); // point front right
  //  Serial.print(" ");
  //  Serial.print(p_fl);
  //  Serial.print(" ");
  //  Serial.print(p_rr);
  //  Serial.print(" ");
  //  Serial.print(rawTime);
  //  Serial.print("\n");

  //  getdat(pos_val, );
  writeEncoderStream();
  delay(10);
}
