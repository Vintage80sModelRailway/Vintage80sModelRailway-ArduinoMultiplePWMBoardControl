// Turnout.cpp
#include "Turnout.h"
#include <Adafruit_PWMServoDriver.h>

Turnout::Turnout() {
  needsFrogPolarityControl = false;
  useSlowMotion = false;
  hasFeedbackSensor = false;
  motorHasNotMovedYet  = true;
  currentPWMVal = 1500;
}

Turnout::Turnout(int ThrownVal, int ClosedVal) {
  thrownVal = ThrownVal;
  closedVal = ClosedVal;
  needsFrogPolarityControl = false;
  useSlowMotion = false;
  hasFeedbackSensor = false;
  lastKnownBitValue = 0;
  motorHasNotMovedYet  = true;
  currentPWMVal = 1500;
}

Turnout::Turnout(int ThrownVal, int ClosedVal, bool InvertFrog) {
  thrownVal = ThrownVal;
  closedVal = ClosedVal;
  invertFrog = InvertFrog;
  needsFrogPolarityControl = true;
  hasFeedbackSensor = false;
  useSlowMotion = false;
  lastKnownBitValue = 0;
  motorHasNotMovedYet  = true;
  relayHasNotBeenSetYet = true;
  currentPWMVal = 1500;
}

Turnout::Turnout(int ThrownVal, int ClosedVal, int FeedbackSensorPin, bool InvertFeedbackSensor) {
  thrownVal = ThrownVal;
  closedVal = ClosedVal;
  feedbackSensorPin = FeedbackSensorPin;
  hasFeedbackSensor = true;
  needsFrogPolarityControl = false;
  useSlowMotion = false;
  lastKnownBitValue = 0;
  motorHasNotMovedYet  = true;
  inDebounce = false;
  millisAtLastChange = 5000;
  currentPWMVal = 1500;
  invertFeedbackSensor = InvertFeedbackSensor;
}

Turnout::Turnout(int ThrownVal, int ClosedVal, int FeedbackSensorPin, bool InvertFrog, bool InvertFeedbackSensor) {
  thrownVal = ThrownVal;
  closedVal = ClosedVal;
  feedbackSensorPin = FeedbackSensorPin;
  hasFeedbackSensor = true;
  invertFrog = InvertFrog;
  needsFrogPolarityControl = true;
  useSlowMotion = false;
  lastKnownBitValue = 0;
  motorHasNotMovedYet  = true;
  millisAtLastChange = 5000;
  inDebounce = false;
  relayHasNotBeenSetYet = true;
  currentPWMVal = 1500;
  invertFeedbackSensor = InvertFeedbackSensor;
}

Turnout::Turnout(int ThrownVal, int ClosedVal, int StepSize, int DelayTime) {
  thrownVal = ThrownVal;
  closedVal = ClosedVal;
  useSlowMotion = true;
  stepSize = StepSize;
  delayTime = DelayTime;
  needsFrogPolarityControl = false;
  hasFeedbackSensor = false;
  lastKnownBitValue = 0;
  motorHasNotMovedYet  = true;
  currentPWMVal = 1500;
}

Turnout::Turnout(int ThrownVal, int ClosedVal, int StepSize, int DelayTime, int FeedbackSensorPin, bool InvertFeedbackSensor) {
  thrownVal = ThrownVal;
  closedVal = ClosedVal;
  useSlowMotion = true;
  stepSize = StepSize;
  delayTime = DelayTime;
  needsFrogPolarityControl = false;
  lastKnownBitValue = 0;
  feedbackSensorPin = FeedbackSensorPin;
  hasFeedbackSensor = true;
  motorHasNotMovedYet  = true;
  millisAtLastChange = 5000;
  inDebounce = false;
  currentPWMVal = 1500;
  invertFeedbackSensor = InvertFeedbackSensor;
}

Turnout::Turnout(int ThrownVal, int ClosedVal, int StepSize, int DelayTime, bool InvertFrog) {
  thrownVal = ThrownVal;
  closedVal = ClosedVal;
  invertFrog = InvertFrog;
  needsFrogPolarityControl = true;
  useSlowMotion = true;
  hasFeedbackSensor = false;
  stepSize = StepSize;
  delayTime = DelayTime;
  lastKnownBitValue = 0;
  motorHasNotMovedYet  = true;
  relayHasNotBeenSetYet = true;
  currentPWMVal = 1500;
}

Turnout::Turnout(int ThrownVal, int ClosedVal, int StepSize, int DelayTime, int FeedbackSensorPin, bool InvertFrog, bool InvertFeedbackSensor) {
  thrownVal = ThrownVal;
  closedVal = ClosedVal;
  invertFrog = InvertFrog;
  needsFrogPolarityControl = true;
  useSlowMotion = true;
  stepSize = StepSize;
  delayTime = DelayTime;
  lastKnownBitValue = 0;
  feedbackSensorPin = FeedbackSensorPin;
  hasFeedbackSensor = true;
  motorHasNotMovedYet  = true;
  millisAtLastChange = 5000;
  inDebounce = false;
  relayHasNotBeenSetYet = true;
  currentPWMVal = 1500;
  invertFeedbackSensor = InvertFeedbackSensor;
}

Turnout::ProcessPointsMove(Adafruit_PWMServoDriver board, int cmriBitValue, int pin) {
  //Serial.println("Points move");
  //Only instruct the servo to move if its required PWM value is different from its current one
  int requiredPosition = closedVal;
  if (cmriBitValue == 1) {
    requiredPosition = thrownVal;
  }
  if (requiredPosition != currentPWMVal || motorHasNotMovedYet)
  {
    //Protect thee motors from accidentally silly, potentially damaging values
    if (requiredPosition > 800 && requiredPosition < 2300)
    {
      board.writeMicroseconds(pin, requiredPosition);
    }

      //Frog polarity - setting bssed on feedback sensor value handled elsewhere
    if (needsFrogPolarityControl) {
        bool fullyOn = cmriBitValue != 0;
        if (fullyOn == true)
        {
          if (invertFrog == false)
          {
            setPWMStateFullyOn(board, pin + 8);
          }
          else
          {
            setPWMStateFullyOff(board, pin + 8);
          }
        }
        else
        {
          if (invertFrog == false)
          {
            setPWMStateFullyOff(board, pin + 8);
          }
          else
          {
            setPWMStateFullyOn(board, pin + 8);
          }
        }
    }
      currentPWMVal = requiredPosition;  
      lastKnownBitValue = cmriBitValue;    
      motorHasNotMovedYet  = false; 
    }
  }

bool Turnout::ProcessPointsMoveWithSpeedControl(Adafruit_PWMServoDriver board, int cmriBitValue, int pin, int numberOfServosMoving, int servoSpeedMultiplier) {
    bool moveIsComplete = false;
    int requiredPosition = closedVal;
    if (cmriBitValue == 1) {
      requiredPosition = thrownVal;
    }
    int calculatedStepSize = numberOfServosMoving;
    unsigned long currentMillis = millis();
    for (int i = 0; i<servoSpeedMultiplier; i++) {
      calculatedStepSize = calculatedStepSize * numberOfServosMoving;
    }
    int totaldStepSize =  stepSize + calculatedStepSize;

    if ((requiredPosition != currentPWMVal && currentMillis - previousMillis >= delayTime))
    {
        previousMillis = currentMillis;
        
        //Protect thee motors from accidentally silly, potentially damaging values
        if (requiredPosition > 800 && requiredPosition < 2251)
        {
            if (requiredPosition > currentPWMVal)
            {              
                int intendedPWMValue = currentPWMVal + totaldStepSize;
                if (intendedPWMValue > requiredPosition)
                {
                  intendedPWMValue = requiredPosition;                  
                }
                
                currentPWMVal = intendedPWMValue;
                board.writeMicroseconds(pin, intendedPWMValue);

                //if required position is now equal to current position, set frog polarity too
                if (requiredPosition == intendedPWMValue)
                {
                  moveIsComplete = true;
                  motorHasNotMovedYet  = false; 
                  lastKnownBitValue = cmriBitValue;  
                }
            }
            else// (requiredPosition < CurrentPWMValue[pin])
            {
                int intendedPWMValue = currentPWMVal - totaldStepSize;
                if (intendedPWMValue < requiredPosition)
                {                  
                  intendedPWMValue = requiredPosition;
                }
                
                currentPWMVal = intendedPWMValue;
                board.writeMicroseconds(pin, intendedPWMValue);

                //if required position is now equal to current position, set frog polarity too
                if (requiredPosition == intendedPWMValue)
                {
                  moveIsComplete = true;
                  motorHasNotMovedYet  = false; 
                  lastKnownBitValue = cmriBitValue;  
                }
            }
        }

        //Set frog polarity by bit value not servo position as some turnouts are inverted
        if (moveIsComplete && needsFrogPolarityControl) {
          bool fullyOn = cmriBitValue != 0;
          if (fullyOn == true)
          {        
            if (invertFrog == false)
            {
              setPWMStateFullyOn(board, pin + 8);
            }
            else
            {
              setPWMStateFullyOff(board, pin + 8);
            }
          }
          else
          {        
            if (invertFrog == false)
            {
              setPWMStateFullyOff(board, pin + 8);
            }
            else
            {
              setPWMStateFullyOn(board, pin + 8);
            }
          }
        }        
    }
    return moveIsComplete;
}

Turnout::SetRelayAccordingToCMRIBitValue(Adafruit_PWMServoDriver board, int pin, int cmriBitValue) {
  bool fullyOn = cmriBitValue != 0;
  if (fullyOn == true && needsFrogPolarityControl)
  {
    if (invertFrog == false)
    {
      setPWMStateFullyOn(board, pin + 8);
    }
    else
    {
      setPWMStateFullyOff(board, pin + 8);
    }
  }
  else
  {
    if (invertFrog == false)
    {
      setPWMStateFullyOff(board, pin + 8);
    }
    else
    {
      setPWMStateFullyOn(board, pin + 8);
    }
  }
}

Turnout::SetRelayAccordingToFeedbackSensor(Adafruit_PWMServoDriver board, int pin, bool pinValue, int feedbackDebounce) {
  //This could be the initial JMRI startup sequence, called from setup -there may be a route set after this
  //So not setting 'motothasnotmovedyet' to false - this will be done on the first run through the turnouts in loop
  //Serial.println("Set relay board "+String(board)+" pin "+String(pin) + "value "+String(pinValue)+" relay not moved "+String(PWMBoards[board].turnouts[pin].relayHasNotBeenSetYet));
  if ((hasFeedbackSensor && lastFeedbackSensorReading != pinValue) || relayHasNotBeenSetYet)
  {
    if (inDebounce == false)
    {
      millisAtLastChange = millis();
      inDebounce = true;
    }
    unsigned long millisSinceLastChange = (millis() - millisAtLastChange);
    if (millisSinceLastChange >= feedbackDebounce)
    {
        bool fullyOn = pinValue != 0;
        if (fullyOn == true && needsFrogPolarityControl)
        {
          if (invertFrog == false)
          {
            //Serial.println("Setting relay fully on, false frog for Board "+String(board)+" Pin "+String(pin)+" Switch position "+String(pinValue)+" last sensor reading "+String(PWMBoards[board].turnouts[pin].lastFeedbackSensorReading));
            setPWMStateFullyOn(board, pin + 8);
          }
          else
          {
            //Serial.println("Setting relay fully off true frog for Board "+String(board)+" Pin "+String(pin)+" Switch position "+String(pinValue)+" last sensor reading "+String(PWMBoards[board].turnouts[pin].lastFeedbackSensorReading));
            setPWMStateFullyOff(board, pin + 8);
          }
        }
        else
        {
          if (invertFrog == false)
          {
            //Serial.println("Setting relay fully off frog false for Board "+String(board)+" Pin "+String(pin)+" Switch position "+String(pinValue)+" last sensor reading "+String(PWMBoards[board].turnouts[pin].lastFeedbackSensorReading));
            setPWMStateFullyOff(board, pin + 8);
          }
          else
          {
            //Serial.println("Setting relay fully off frog truefor Board "+String(board)+" Pin "+String(pin)+" Switch position "+String(pinValue)+" last sensor reading "+String(PWMBoards[board].turnouts[pin].lastFeedbackSensorReading));
            setPWMStateFullyOn(board, pin + 8);
          }
        }

        if (lastKnownBitValue == 0) {
           currentPWMVal = closedVal;
        }
        else {
          currentPWMVal = thrownVal;
        }

      lastFeedbackSensorReading = pinValue;
      lastKnownBitValue = pinValue;
      inDebounce = false;
      relayHasNotBeenSetYet = false;
    }
  }
  else
  {
    //Status change has flicked on then off, or off then on, within the debounce period so don't change anything and set inDebounce back to false
    inDebounce = false;
  }
}

void Turnout::setPWMStateFullyOn(Adafruit_PWMServoDriver board, int pin)
{
  if (pin < 8) return;
    board.setPWM(pin, 4096, 0);
}

void Turnout::setPWMStateFullyOff(Adafruit_PWMServoDriver board, int pin)
{
  if (pin < 8) return;
    board.setPWM(pin, 0, 4096);
}
