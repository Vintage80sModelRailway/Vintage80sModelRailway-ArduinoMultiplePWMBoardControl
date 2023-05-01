// PWMBoard.h
#ifndef PWMBoard_h
#define PWMBoard_h

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include "Turnout.h"
//#include "ListLib.h"

class PWMBoard {
  private:


  public: 
    int numberOfTurnouts;
    int CMRIIndexModifier;
    //List<Turnout> turnouts;
    Turnout turnouts[8];
    Adafruit_PWMServoDriver pwm;  
};

#endif
