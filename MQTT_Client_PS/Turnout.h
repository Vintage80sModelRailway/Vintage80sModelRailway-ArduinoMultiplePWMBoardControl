// Turnout.h
#ifndef Turnout_h
#define Turnout_h

#include <Arduino.h>

class Turnout {
  private:

  public: 
    int thrownVal;
    int closedVal;
    int currentPWMVal;
    int requiredPWMVal;
    String currentState;
    String requiredState;
    int stepSize;
    int delayTime;
    int feedbackSensorPin;
    bool lastFeedbackSensorReading;
    bool useSlowMotion;
    bool needsFrogPolarityControl;
    bool invertFrog;
    bool hasFeedbackSensor;
    bool invertFeedbackSensor;
    bool motorHasNotMovedYet;
    bool inDebounce;
    bool relayHasNotBeenSetYet;
    bool isAnLED;
    String jMRIId;
    unsigned long millisAtLastChange;
    unsigned long previousMillis;
    
    Turnout();
    Turnout(String JMRIId);
    Turnout(String JMRIId, int ThrownVal, int ClosedVal);
    Turnout(String JMRIId,int ThrownVal, int ClosedVal, int FeedbackSensorPin, bool InvertFeedbackSensor);
    
    Turnout(String JMRIId,int ThrownVal, int ClosedVal, bool InvertFrog);
    Turnout(String JMRIId,int ThrownVal, int ClosedVal, int FeedbackSensorPin, bool InvertFrog, bool InvertFeedbackSensor);
    
    Turnout(String JMRIId,int ThrownVal, int ClosedVal, int StepSize, int DelayTime);
    Turnout(String JMRIId,int ThrownVal, int ClosedVal, int StepSize, int DelayTime, int FeedbackSensorPin, bool InvertFeedbackSensor);
    
    Turnout(String JMRIId,int ThrownVal, int ClosedVal, int StepSize, int DelayTime, bool InvertFrog);
    Turnout(String JMRIId,int ThrownVal, int ClosedVal, int StepSize, int DelayTime, int FeedbackSensorPin, bool InvertFrog, bool InvertFeedbackSensor);
    
};

#endif
