// Turnout.cpp
#include "Turnout.h"

Turnout::Turnout() {
  needsFrogPolarityControl = false;
  useSlowMotion = false;
  hasFeedbackSensor = false;
}

Turnout::Turnout(int ThrownVal, int ClosedVal) {
  thrownVal = ThrownVal;
  closedVal = ClosedVal;
  needsFrogPolarityControl = false;
  useSlowMotion = false;
  hasFeedbackSensor = false;
  lastKnownBitValue = 0;
}

Turnout::Turnout(int ThrownVal, int ClosedVal, bool InvertFrog) {
  thrownVal = ThrownVal;
  closedVal = ClosedVal;
  invertFrog = InvertFrog;
  needsFrogPolarityControl = true;
  hasFeedbackSensor = false;
  useSlowMotion = false;
  lastKnownBitValue = 0;
}

Turnout::Turnout(int ThrownVal, int ClosedVal, int FeedbackSensorPin, bool InvertFeesbackSensor) {
  thrownVal = ThrownVal;
  closedVal = ClosedVal;
  feedbackSensorPin = FeedbackSensorPin;
  hasFeedbackSensor = true;
  invertFeedbackSensor = InvertFeesbackSensor;
  needsFrogPolarityControl = false;
  useSlowMotion = false;
  lastKnownBitValue = 0;
}

Turnout::Turnout(int ThrownVal, int ClosedVal, int FeedbackSensorPin, bool InvertFrog, bool InvertFeesbackSensor) {
  thrownVal = ThrownVal;
  closedVal = ClosedVal;
  feedbackSensorPin = FeedbackSensorPin;
  hasFeedbackSensor = true;
  invertFeedbackSensor = InvertFeesbackSensor;
  invertFrog = InvertFrog;
  needsFrogPolarityControl = true;
  useSlowMotion = false;
  lastKnownBitValue = 0;
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
}

Turnout::Turnout(int ThrownVal, int ClosedVal, int StepSize, int DelayTime, int FeedbackSensorPin, bool InvertFrog, bool InvertFeesbackSensor) {
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
  invertFeedbackSensor = InvertFeesbackSensor;
}
