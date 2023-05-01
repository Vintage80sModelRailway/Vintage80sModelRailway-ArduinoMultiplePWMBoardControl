// Turnout.cpp
#include "Turnout.h"

Turnout::Turnout() {
  needsFrogPolarityControl = false;
  useSlowMotion = false;
  hasFeedbackSensor = false;
  motorHasNotMovedYet  = true;
  currentPWMVal = 1500;
}

Turnout::Turnout(String JMRIId, int ThrownVal, int ClosedVal) {
  jMRIId = JMRIId;
  thrownVal = ThrownVal;
  closedVal = ClosedVal;
  needsFrogPolarityControl = false;
  useSlowMotion = false;
  hasFeedbackSensor = false;
  motorHasNotMovedYet  = true;
  currentPWMVal = 1500;
}

Turnout::Turnout(String JMRIId, int ThrownVal, int ClosedVal, bool InvertFrog) {
  jMRIId = JMRIId;
  thrownVal = ThrownVal;
  closedVal = ClosedVal;
  invertFrog = InvertFrog;
  needsFrogPolarityControl = true;
  hasFeedbackSensor = false;
  useSlowMotion = false;
  motorHasNotMovedYet  = true;
  relayHasNotBeenSetYet = true;
  currentPWMVal = 1500;
}

Turnout::Turnout(String JMRIId, int ThrownVal, int ClosedVal, int FeedbackSensorPin, bool InvertFeedbackSensor) {
  jMRIId = JMRIId;
  thrownVal = ThrownVal;
  closedVal = ClosedVal;
  feedbackSensorPin = FeedbackSensorPin;
  hasFeedbackSensor = true;
  needsFrogPolarityControl = false;
  useSlowMotion = false;
  motorHasNotMovedYet  = true;
  inDebounce = false;
  millisAtLastChange = 5000;
  currentPWMVal = 1500;
  invertFeedbackSensor = InvertFeedbackSensor;
}

Turnout::Turnout(String JMRIId, int ThrownVal, int ClosedVal, int FeedbackSensorPin, bool InvertFrog, bool InvertFeedbackSensor) {
  jMRIId = JMRIId;
  thrownVal = ThrownVal;
  closedVal = ClosedVal;
  feedbackSensorPin = FeedbackSensorPin;
  hasFeedbackSensor = true;
  invertFrog = InvertFrog;
  needsFrogPolarityControl = true;
  useSlowMotion = false;
  motorHasNotMovedYet  = true;
  millisAtLastChange = 5000;
  inDebounce = false;
  relayHasNotBeenSetYet = true;
  currentPWMVal = 1500;
  invertFeedbackSensor = InvertFeedbackSensor;
}

Turnout::Turnout(String JMRIId, int ThrownVal, int ClosedVal, int StepSize, int DelayTime) {
  jMRIId = JMRIId;
  thrownVal = ThrownVal;
  closedVal = ClosedVal;
  useSlowMotion = true;
  stepSize = StepSize;
  delayTime = DelayTime;
  needsFrogPolarityControl = false;
  hasFeedbackSensor = false;
  motorHasNotMovedYet  = true;
  currentPWMVal = 1500;
}

Turnout::Turnout(String JMRIId, int ThrownVal, int ClosedVal, int StepSize, int DelayTime, int FeedbackSensorPin, bool InvertFeedbackSensor) {
  jMRIId = JMRIId;
  thrownVal = ThrownVal;
  closedVal = ClosedVal;
  useSlowMotion = true;
  stepSize = StepSize;
  delayTime = DelayTime;
  needsFrogPolarityControl = false;
  feedbackSensorPin = FeedbackSensorPin;
  hasFeedbackSensor = true;
  motorHasNotMovedYet  = true;
  millisAtLastChange = 5000;
  inDebounce = false;
  currentPWMVal = 1500;
  invertFeedbackSensor = InvertFeedbackSensor;
}

Turnout::Turnout(String JMRIId, int ThrownVal, int ClosedVal, int StepSize, int DelayTime, bool InvertFrog) {
  jMRIId = JMRIId;
  thrownVal = ThrownVal;
  closedVal = ClosedVal;
  invertFrog = InvertFrog;
  needsFrogPolarityControl = true;
  useSlowMotion = true;
  hasFeedbackSensor = false;
  stepSize = StepSize;
  delayTime = DelayTime;
  motorHasNotMovedYet  = true;
  relayHasNotBeenSetYet = true;
  currentPWMVal = 1500;
}

Turnout::Turnout(String JMRIId, int ThrownVal, int ClosedVal, int StepSize, int DelayTime, int FeedbackSensorPin, bool InvertFrog, bool InvertFeedbackSensor) {
  jMRIId = JMRIId;
  thrownVal = ThrownVal;
  closedVal = ClosedVal;
  invertFrog = InvertFrog;
  needsFrogPolarityControl = true;
  useSlowMotion = true;
  stepSize = StepSize;
  delayTime = DelayTime;
  feedbackSensorPin = FeedbackSensorPin;
  hasFeedbackSensor = true;
  motorHasNotMovedYet  = true;
  millisAtLastChange = 5000;
  inDebounce = false;
  relayHasNotBeenSetYet = true;
  currentPWMVal = 1500;
  invertFeedbackSensor = InvertFeedbackSensor;
}

Turnout::Turnout(String JMRIId) {
  jMRIId = JMRIId;
  isAnLED = true;
}
