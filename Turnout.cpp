// Turnout.cpp
#include "Turnout.h"

Turnout::Turnout() {
  needsFrogPolarityControl = false;
  useSlowMotion = false;
}

Turnout::Turnout(int ThrownVal, int ClosedVal) {
  thrownVal = ThrownVal;
  closedVal = ClosedVal;
  needsFrogPolarityControl = false;
  useSlowMotion = false;
}

Turnout::Turnout(int ThrownVal, int ClosedVal, bool InvertFrog) {
  thrownVal = ThrownVal;
  closedVal = ClosedVal;
  invertFrog = InvertFrog;
  needsFrogPolarityControl = true;
  useSlowMotion = false;
}

Turnout::Turnout(int ThrownVal, int ClosedVal, int StepSize, int DelayTime) {
  thrownVal = ThrownVal;
  closedVal = ClosedVal;
  useSlowMotion = true;
  stepSize = StepSize;
  delayTime = DelayTime;
  needsFrogPolarityControl = false;
}

Turnout::Turnout(int ThrownVal, int ClosedVal, int StepSize, int DelayTime, bool InvertFrog) {
  thrownVal = ThrownVal;
  closedVal = ClosedVal;
  invertFrog = InvertFrog;
  needsFrogPolarityControl = true;
  useSlowMotion = true;
  stepSize = StepSize;
  delayTime = DelayTime;
}
