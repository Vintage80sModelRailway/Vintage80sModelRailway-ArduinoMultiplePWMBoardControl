// Turnout.h
#ifndef Turnout_h
#define Turnout_h

#include <Arduino.h>

class Turnout {
  private:

  public: 
    //Properties
    int thrownVal;
    int closedVal;
    int currentPWMVal;
    int stepSize;
    int delayTime;
    bool useSlowMotion;
    bool needsFrogPolarityControl;
    bool invertFrog;
    unsigned long previousMillis;

    //Constructors
    Turnout();
    Turnout(int ThrownVal, int ClosedVal);
    Turnout(int ThrownVal, int ClosedVal, bool InvertFrog);
    Turnout(int ThrownVal, int ClosedVal, int StepSize, int DelayTime);
    Turnout(int ThrownVal, int ClosedVal, int StepSize, int DelayTime, bool InvertFrog);
    
};

#endif
