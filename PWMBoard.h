// PWMBoard.h
#ifndef PWMBoard_h
#define PWMBoard_h

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include "Turnout.h"

class PWMBoard {
  private:


  public: 
    int numberOfServos;
    int CMRIIndexModifier;
    Turnout turnouts[8];
    Adafruit_PWMServoDriver pwm;  
};

#endif
