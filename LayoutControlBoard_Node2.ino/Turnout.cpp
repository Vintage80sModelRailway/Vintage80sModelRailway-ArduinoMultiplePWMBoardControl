// Turnout.cpp
#include "Turnout.h"

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
