#include <Auto485.h>
#include <CMRI.h>
#include <Adafruit_PWMServoDriver.h>

#include "PWMBoard.h"
#include "Turnout.h"

#define CMRI_ADDR 1 //Corner 2
#define NumberOfPWMBoards 2

int feedbackDebounce = 150;
unsigned long startupDelayMillis = 10000;
unsigned long startupMillis;
bool startupDelayComplete;
const bool ProtectMotors = false;
const bool OutputToSerial = false;
int numberOfServosMoving = 0;
int numberOfServosMovingLastLoop = 0;
int servoSpeedMultipluer = 1;

////Pin connected to DS of 74HC595 - SERIN
int dataPinOut = 44;
//Pin connected to ST_CP of 74HC595 - RCLCK
int latchPinOut = 43;
//Pin connected to SH_CP of 74HC595 - SRCLCK
int clockPinOut = 42;

byte previousData1;
byte previousData2;

int load = 38;
int clockEnablePin = 39;
int dataIn = 40;
int clockIn = 41;

#define    DE_PIN 2
#define   LED_PIN 13

PWMBoard PWMBoards[NumberOfPWMBoards];

//Only use Serial1 on a Mega as it has multiple serial buses
Auto485 bus(DE_PIN, DE_PIN, Serial1);
CMRI cmri(CMRI_ADDR, 72, 72, bus);

void setup() {
  
  bus.begin(19200, SERIAL_8N2);
  InitialiseConfig();
  SetPinModes();
  startupDelayComplete = false;
  Serial.begin(19200);
  if (OutputToSerial) {
    Serial.begin(19200);
    Serial.println("CMRI Address " + String(CMRI_ADDR));
  }

  digitalWrite(LED_PIN, 0);
  bool cmriIsInitialised = false;

  while (!cmriIsInitialised) {
    digitalWrite(load, LOW);
    //Get CMRI data ready for when CMRI does come online
    SetSensorFeedbackReadings();
    ProcessInputs();
    ReadFromShiftRegister();
    cmriIsInitialised = cmri.process();
  }

  digitalWrite(LED_PIN, 1);
  startupMillis = millis();

  if (OutputToSerial) {
    Serial.println("CMRI / RS485 Ready");
  }

}

void loop() {
  //At startup, JMRI / CMRI just send a load of 0s in the data packet, which sends all motors crazy.
  //It also sends that data before it polls, which means all motors move
  //Trying here to delay acting on CMRI data until adter it's polled and recieved statuses from the feedback sensors
  //Then everything should get set to how it currently is with no craziness
  bool cmriIsRunning = cmri.process();
  numberOfServosMoving = 0;
  digitalWrite(load, LOW);

  if (OutputToSerial) {
    // Serial.println("Result of process: "+String(cmriIsRunning));
  }

  if (!startupDelayComplete)
  {
    unsigned long millisSinceCMRIStarted = millis() - startupMillis;

    if (millisSinceCMRIStarted < startupDelayMillis)
    {
      //Serial.println("Pin test "+String(digitalRead(4)));
      SetSensorFeedbackReadings();
      ProcessInputs();
      ReadFromShiftRegister();

      if (OutputToSerial) {
        //Serial.println("CMRI start delay - waiting for "+String(startupDelayMillis)+" millis, currently "+String(millisSinceCMRIStarted));
      }

      //Now exit the loop
      return;
    }
    else
    {
      startupDelayComplete = true;

      if (OutputToSerial) {
        Serial.println("Delay complete");
      }
    }
  }
  
  //for each PWM board
  for (int pwmIndex = 0; pwmIndex < NumberOfPWMBoards; pwmIndex++) {
    //for each servo on this PWM board
    for (int i = 0; i < PWMBoards[pwmIndex].numberOfTurnouts; i++)
    {
      bool moveIsComplete=false;
      int deviceStatusFromJMRI = cmri.get_bit(i + PWMBoards[pwmIndex].CMRIIndexModifier);
      if (deviceStatusFromJMRI != PWMBoards[pwmIndex].turnouts[i].lastKnownBitValue || PWMBoards[pwmIndex].turnouts[i].motorHasNotMovedYet)
      {
        numberOfServosMoving++;
        //Serial.println("Bit value change on board "+String(pwmIndex)+", device " + String(i) + " JMRI status "+String(deviceStatusFromJMRI)+" last known bit "+String( PWMBoards[pwmIndex].turnouts[i].lastKnownBitValue)+" motor not moved yet "+String(PWMBoards[pwmIndex].turnouts[i].motorHasNotMovedYet));
        if (deviceStatusFromJMRI == 1)
        {
          if (OutputToSerial) {
            //Serial.println("Throw");
          }

          int throwValue = PWMBoards[pwmIndex].turnouts[i].thrownVal;
          if (PWMBoards[pwmIndex].turnouts[i].useSlowMotion == true) {
             moveIsComplete = ProcessPointsMoveWithSpeedControl(pwmIndex, i, throwValue, deviceStatusFromJMRI, numberOfServosMovingLastLoop);
          }
          else {
            ProcessPointsMove(pwmIndex, i, throwValue, deviceStatusFromJMRI);
            PWMBoards[pwmIndex].turnouts[i].lastKnownBitValue = deviceStatusFromJMRI;
          }
        }
        else
        {
          if (OutputToSerial) {
            //Serial.println("Close");
          }

          int closeValue = PWMBoards[pwmIndex].turnouts[i].closedVal;
          if (PWMBoards[pwmIndex].turnouts[i].useSlowMotion == true) {
           moveIsComplete = ProcessPointsMoveWithSpeedControl(pwmIndex, i, closeValue, deviceStatusFromJMRI, numberOfServosMovingLastLoop);
          }
          else {
            ProcessPointsMove(pwmIndex, i, closeValue, deviceStatusFromJMRI);
            PWMBoards[pwmIndex].turnouts[i].lastKnownBitValue = deviceStatusFromJMRI;
          }
        }

        if (OutputToSerial) {
          //Serial.println("Set last known value for board "+String(pwmIndex)+" turnout "+String(i)+" to "+String(deviceStatusFromJMRI));
        }

        PWMBoards[pwmIndex].turnouts[i].motorHasNotMovedYet  = false;

        
      }
      else
      {
        if (OutputToSerial) {
          //Serial.println("Get CMRI bit "+String(i+PWMBoards[pwmIndex].CMRIIndexModifier)+" for board "+String(pwmIndex)+" turnout "+String(i)+" value "+String(deviceStatusFromJMRI));
        }

      }
      if (moveIsComplete) {
        PWMBoards[pwmIndex].turnouts[i].lastKnownBitValue = deviceStatusFromJMRI;  
       }
      //set feedback sensor bit if using - only need to set when a points motor has not moved
        if (PWMBoards[pwmIndex].turnouts[i].hasFeedbackSensor) {
          int inputPin = PWMBoards[pwmIndex].turnouts[i].feedbackSensorPin;
          bool pinValue = false;
          if (PWMBoards[pwmIndex].turnouts[i].invertFeedbackSensor == true) {
            pinValue = !digitalRead(inputPin);
          }
          else {
            pinValue = digitalRead(inputPin);
          }
          if (OutputToSerial) {
            //Serial.println("Setting feedback sensor for CMRI bit "+String(i+PWMBoards[pwmIndex].CMRIIndexModifier)+" to "+String(pinValue));
          }
          SetTurnoutFeedback(pwmIndex, i, pinValue);
        }      
    }
  }
  ProcessInputs();
  ReadFromShiftRegister();
  ProcessShiftOut();
  numberOfServosMovingLastLoop = numberOfServosMoving;
}

void ProcessPointsMove(int board, int pin, int requiredPosition, int cmriBitValue)
{
  //Serial.println("Points move");
  //Only instruct the servo to move if its required PWM value is different from its current one
  if (requiredPosition != PWMBoards[board].turnouts[pin].currentPWMVal || PWMBoards[board].turnouts[pin].motorHasNotMovedYet)
  {
    //Protect thee motors from accidentally silly, potentially damaging values
    if (requiredPosition > 800 && requiredPosition < 2300)
    {
      if (OutputToSerial) {
        Serial.println("Points move required on board " + String(board) + ", device " + String(pin) + " to position " + String(requiredPosition) + ".");
      }

      if (!ProtectMotors) {
        PWMBoards[board].pwm.writeMicroseconds(pin, requiredPosition);
      }

      //Frog polarity - setting bssed on feedback sensor value handled elsewhere
      //if (!PWMBoards[board].turnouts[pin].hasFeedbackSensor) {
        bool fullyOn = cmriBitValue != 0;

        if (fullyOn == true)
        {
          if (OutputToSerial) {
            Serial.println("Fully on for board " + String(board) + " turnout " + String(pin) + " CMRI bit value " + String(cmriBitValue));
          }

          if (PWMBoards[board].turnouts[pin].invertFrog == false)
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
          if (OutputToSerial) {
            Serial.println("Fully off for board " + String(board) + " turnout " + String(pin) + " CMRI bit value " + String(cmriBitValue));
          }

          if (PWMBoards[board].turnouts[pin].invertFrog == false)
          {
            setPWMStateFullyOff(board, pin + 8);
          }
          else
          {
            setPWMStateFullyOn(board, pin + 8);
          }
        }
      //}
      PWMBoards[board].turnouts[pin].currentPWMVal = requiredPosition;
      
    }
    else
    {
      if (OutputToSerial) {
        //Serial.println("Points move not required on board " + String(board) + ", device " + String(pin) + " to position " + String(requiredPosition) + ".");
      }

    }
  }
}

bool ProcessPointsMoveWithSpeedControl(int board, int pin, int requiredPosition, int cmriBitValue, int numberOfServosMoving)
{
    bool moveIsComplete = false;
    int calculatedStepSize = numberOfServosMoving;
    unsigned long currentMillis = millis();
    for (int i = 0; i<servoSpeedMultipluer; i++) {
      calculatedStepSize = calculatedStepSize * numberOfServosMoving;
    }
    int totaldStepSize =  PWMBoards[board].turnouts[pin].stepSize + calculatedStepSize;

    if ((requiredPosition != PWMBoards[board].turnouts[pin].currentPWMVal && currentMillis - PWMBoards[board].turnouts[pin].previousMillis >= PWMBoards[board].turnouts[pin].delayTime))//| PWMBoards[board].turnouts[pin].motorHasNotMovedYet)
    {
        //Serial.println("Inside first check");
        PWMBoards[board].turnouts[pin].previousMillis = currentMillis;
        
        //Protect thee motors from accidentally silly, potentially damaging values
        if (requiredPosition > 800 && requiredPosition < 2251)
        {
            if (requiredPosition > PWMBoards[board].turnouts[pin].currentPWMVal)
            {
              
                int intendedPWMValue = PWMBoards[board].turnouts[pin].currentPWMVal + totaldStepSize;
                if (intendedPWMValue > requiredPosition)
                {
                  intendedPWMValue = requiredPosition;
                  
                }
                
                PWMBoards[board].turnouts[pin].currentPWMVal = intendedPWMValue;
                //Serial.println("Points move required on board "+String(board)+", device " + String(pin) + " to position " + String(intendedPWMValue) + ".");
                PWMBoards[board].pwm.writeMicroseconds(pin, intendedPWMValue);

                //if required position is now equal to current position, set frog polarity too
                if (requiredPosition == intendedPWMValue)
                {
//                  if (PWMBoards[board].turnouts[pin].invertFrog)
//                  {
//                    //Serial.println("Setting frog polarity for board "+String(board)+", pin "+String(pin));
//                    setPWMStateFullyOn(board, pin + 8);
//                  }
//                  else
//                  {
//                    //Serial.println("Setting frog polarity inverted for board "+String(board)+", pin "+String(pin));
//                    setPWMStateFullyOff(board, pin + 8);
//                  }
                  moveIsComplete = true;
                }
            }
            else// (requiredPosition < CurrentPWMValue[pin])
            {
                int intendedPWMValue = PWMBoards[board].turnouts[pin].currentPWMVal - totaldStepSize;
                if (intendedPWMValue < requiredPosition)
                {                  
                  intendedPWMValue = requiredPosition;
                }
                
                PWMBoards[board].turnouts[pin].currentPWMVal = intendedPWMValue;
                //Serial.println("Points move required on board "+String(board)+", device " + String(pin) + " to position " + String(intendedPWMValue) + ".");
                PWMBoards[board].pwm.writeMicroseconds(pin, intendedPWMValue);

                //if required position is now equal to current position, set frog polarity too
                if (requiredPosition == intendedPWMValue)
                {
//                  if (PWMBoards[board].turnouts[pin].invertFrog)
//                  {
//                    //Serial.println("Setting frog polarity for board "+String(board)+", pin "+String(pin));
//                    setPWMStateFullyOff(board, pin + 8);
//                  }
//                  else
//                  {
//                    //Serial.println("Setting frog polarity inverted forr board "+String(board)+", pin "+String(pin));
//                    setPWMStateFullyOn(board, pin + 8);
//                  }
                  moveIsComplete = true;
                }
            }
        }

        //Set frog polarity by bit value not servo position as some turnouts are inverted
        if (moveIsComplete) {
          bool fullyOn = cmriBitValue != 0;
          if (fullyOn == true)
          {
            if (OutputToSerial) {
              Serial.println("Fully on at startup for board " + String(board) + " turnout " + String(pin) + " CMRI bit value " + String(cmriBitValue));
            }
        
            if (PWMBoards[board].turnouts[pin].invertFrog == false)
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
            if (OutputToSerial) {
              Serial.println("Fully off at startup for board " + String(board) + " turnout " + String(pin) + " CMRI bit value " + String(cmriBitValue));
            }
        
            if (PWMBoards[board].turnouts[pin].invertFrog == false)
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

void SetSensorFeedbackReadings() {

  for (int pwmIndex = 0; pwmIndex < NumberOfPWMBoards; pwmIndex++) {
    //for each servo on this PWM board
    for (int i = 0; i < PWMBoards[pwmIndex].numberOfTurnouts; i++)
    {
      int inputPin = PWMBoards[pwmIndex].turnouts[i].feedbackSensorPin;
      bool pinValue = false;
      if (PWMBoards[pwmIndex].turnouts[i].invertFeedbackSensor == true) {
        pinValue = !digitalRead(inputPin);
      }
      else {
        pinValue = digitalRead(inputPin);
      }
      SetRelayAccordingToFeedbackSensor(pwmIndex, i, pinValue);
      cmri.set_bit(i + PWMBoards[pwmIndex].CMRIIndexModifier, pinValue);
      PWMBoards[pwmIndex].turnouts[i].lastFeedbackSensorReading = pinValue;
      PWMBoards[pwmIndex].turnouts[i].lastKnownBitValue = pinValue;
      if (PWMBoards[pwmIndex].turnouts[i].lastKnownBitValue == 0) {
           PWMBoards[pwmIndex].turnouts[i].currentPWMVal =  PWMBoards[pwmIndex].turnouts[i].closedVal;
        }
        else {
          PWMBoards[pwmIndex].turnouts[i].currentPWMVal =  PWMBoards[pwmIndex].turnouts[i].thrownVal;
        }
      if (OutputToSerial) {
        //Serial.println("Set sensor feedback just set lastKnownBitValue");
        //Serial.println("Set CMRI bit " + String(i + PWMBoards[pwmIndex].CMRIIndexModifier) + " for board " + String(pwmIndex) + " turnout " + String(i) + " Arduino pin " + String(inputPin) + " value " + String(pinValue));
        //Serial.println("Get CMRI bit during startup pause "+String(i+PWMBoards[pwmIndex].CMRIIndexModifier)+" for board "+String(pwmIndex)+" turnout "+String(i)+" value "+String(cmri.get_bit(i+PWMBoards[pwmIndex].CMRIIndexModifier)));
      }

    }
  }
}

void SetTurnoutFeedback(int board, int pin, bool pinValue) {
  if ((PWMBoards[board].turnouts[pin].hasFeedbackSensor && PWMBoards[board].turnouts[pin].lastFeedbackSensorReading != pinValue) || PWMBoards[board].turnouts[pin].relayHasNotBeenSetYet)
  {
    //Serial.println("Relay not moved");
    if (PWMBoards[board].turnouts[pin].inDebounce == false)
    {
      if (OutputToSerial) {
        //Serial.println("Setting to in debounce");
      }

      PWMBoards[board].turnouts[pin].millisAtLastChange = millis();
      PWMBoards[board].turnouts[pin].inDebounce = true;
    }
    unsigned long millisSinceLastChange = (millis() - PWMBoards[board].turnouts[pin].millisAtLastChange);
    if (OutputToSerial) {
      //Serial.println("Millis since last change "+String(millisSinceLastChange)+" pin value "+String(pinValue));
    }

    if (millisSinceLastChange >= feedbackDebounce)
    {
      cmri.set_bit(pin + PWMBoards[board].CMRIIndexModifier, pinValue);
      PWMBoards[board].turnouts[pin].lastFeedbackSensorReading = pinValue;
      PWMBoards[board].turnouts[pin].inDebounce = false;
      PWMBoards[board].turnouts[pin].relayHasNotBeenSetYet = false;
    }
  }
  else
  {
    //Status change has flicked on then off, or off then on, within the debounce period so don't change anything and set inDebounce back to false
    PWMBoards[board].turnouts[pin].inDebounce = false;
  }
}

void SetRelayAccordingToFeedbackSensor(int board, int pin, bool pinValue) {
  //This could be the initial JMRI startup sequence, called from setup -there may be a route set after this
  //So not setting 'motothasnotmovedyet' to false - this will be done on the first run through the turnouts in loop
  Serial.println("Set relay board "+String(board)+" pin "+String(pin) + "value "+String(pinValue)+" relay not moved "+String(PWMBoards[board].turnouts[pin].relayHasNotBeenSetYet));
  if ((PWMBoards[board].turnouts[pin].hasFeedbackSensor && PWMBoards[board].turnouts[pin].lastFeedbackSensorReading != pinValue) || PWMBoards[board].turnouts[pin].relayHasNotBeenSetYet)
  {
    //Serial.println("Relay not moved");
    if (PWMBoards[board].turnouts[pin].inDebounce == false)
    {
      if (OutputToSerial) {
        //Serial.println("Setting to in debounce");
      }

      PWMBoards[board].turnouts[pin].millisAtLastChange = millis();
      PWMBoards[board].turnouts[pin].inDebounce = true;
    }
    unsigned long millisSinceLastChange = (millis() - PWMBoards[board].turnouts[pin].millisAtLastChange);
    if (OutputToSerial) {
      //Serial.println("Millis since last change "+String(millisSinceLastChange)+" pin value "+String(pinValue));
    }

    if (millisSinceLastChange >= feedbackDebounce)
    {
        bool fullyOn = pinValue != 0;
        if (fullyOn == true)
        {
          if (PWMBoards[board].turnouts[pin].invertFrog == false)
          {
            Serial.println("Setting relay fully on, false frog for Board "+String(board)+" Pin "+String(pin)+" Switch position "+String(pinValue)+" last sensor reading "+String(PWMBoards[board].turnouts[pin].lastFeedbackSensorReading));
            setPWMStateFullyOn(board, pin + 8);
          }
          else
          {
            Serial.println("Setting relay fully off true frog for Board "+String(board)+" Pin "+String(pin)+" Switch position "+String(pinValue)+" last sensor reading "+String(PWMBoards[board].turnouts[pin].lastFeedbackSensorReading));
            setPWMStateFullyOff(board, pin + 8);
          }
        }
        else
        {
          if (PWMBoards[board].turnouts[pin].invertFrog == false)
          {
            Serial.println("Setting relay fully off frog false for Board "+String(board)+" Pin "+String(pin)+" Switch position "+String(pinValue)+" last sensor reading "+String(PWMBoards[board].turnouts[pin].lastFeedbackSensorReading));
            setPWMStateFullyOff(board, pin + 8);
          }
          else
          {
            Serial.println("Setting relay fully off frog truefor Board "+String(board)+" Pin "+String(pin)+" Switch position "+String(pinValue)+" last sensor reading "+String(PWMBoards[board].turnouts[pin].lastFeedbackSensorReading));
            setPWMStateFullyOn(board, pin + 8);
          }
        }




      
//      if (PWMBoards[board].turnouts[pin].invertFrog == true) {
//        if (OutputToSerial) {
//          //Serial.println("Setting relay for Board "+String(board)+" Pin "+String(pin)+" Switch position "+String(pinValue)+" last sensor reading "+String(PWMBoards[board].turnouts[pin].lastFeedbackSensorReading));
//        }
//
//        setRelay(board, pin + 8, !pinValue);
//      }
//      else {
//        if (OutputToSerial) {
//          //Serial.println("Setting relay inverted for Board "+String(board)+" Pin "+String(pin)+" Switch position "+String(pinValue)+" last sensor reading "+String(PWMBoards[board].turnouts[pin].lastFeedbackSensorReading));
//        }
//
//        setRelay(board, pin + 8, pinValue);
//      }

      cmri.set_bit(pin + PWMBoards[board].CMRIIndexModifier, pinValue);
      PWMBoards[board].turnouts[pin].lastFeedbackSensorReading = pinValue;
      PWMBoards[board].turnouts[pin].inDebounce = false;
      PWMBoards[board].turnouts[pin].relayHasNotBeenSetYet = false;
    }
    else
    {
      if (OutputToSerial) {
        //Serial.println("Board "+String(board)+"Pin "+String(pin)+" Switch position "+String(pinValue)+" last sensor reading "+String(PWMBoards[board].turnouts[pin].lastFeedbackSensorReading));
      }

    }
  }
  else
  {
    //Status change has flicked on then off, or off then on, within the debounce period so don't change anything and set inDebounce back to false
    PWMBoards[board].turnouts[pin].inDebounce = false;
  }
}

void SetRelayAccordingToCMRIBitValue(int board, int pin, int cmriBitValue)
{
  bool fullyOn = cmriBitValue != 0;
  if (fullyOn == true)
  {
    if (OutputToSerial) {
      Serial.println("Fully on at startup for board " + String(board) + " turnout " + String(pin) + " CMRI bit value " + String(cmriBitValue));
    }

    if (PWMBoards[board].turnouts[pin].invertFrog == false)
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
    if (OutputToSerial) {
      Serial.println("Fully off at startup for board " + String(board) + " turnout " + String(pin) + " CMRI bit value " + String(cmriBitValue));
    }

    if (PWMBoards[board].turnouts[pin].invertFrog == false)
    {
      setPWMStateFullyOff(board, pin + 8);
    }
    else
    {
      setPWMStateFullyOn(board, pin + 8);
    }
  }
}

void setPWMStateFullyOn(int board, int pin)
{
  if (pin < 8) return;
  if (OutputToSerial) {
    //Serial.println("Set relay fully on for board "+String(board)+",  pin "+String(pin));
  }

  if (!ProtectMotors) {
    PWMBoards[board].pwm.setPWM(pin, 4096, 0);
  }
}

void setPWMStateFullyOff(int board, int pin)
{
  if (pin < 8) return;
  if (!ProtectMotors) {
    PWMBoards[board].pwm.setPWM(pin, 0, 4096);
  }
  if (OutputToSerial) {
    //Serial.println("Set relay fully off for board "+String(board)+",  pin "+String(pin));
  }
}

void setRelay(int board, int pin, bool setting) {
  if (pin < 8) return;
  if (!ProtectMotors) {
    PWMBoards[board].pwm.setPWM(pin, setting == true ? 4096 : 0, setting == true ? 0 : 4096);
  }
  if (OutputToSerial) {
    Serial.println("Set relay board " + String(board) + "pin " + String(pin) + " setting " + String(setting));
  }
}

void ReadFromShiftRegister() {  
  //digitalWrite(load, LOW);
  //delayMicroseconds(5);
  digitalWrite(load, HIGH);
  //delayMicroseconds(5);

  // Get data from 74HC165
  digitalWrite(clockIn, HIGH);
  digitalWrite(clockEnablePin, LOW);
  byte data1 = shiftIn(dataIn, clockIn, MSBFIRST);  
  byte data2 = shiftIn(dataIn, clockIn, MSBFIRST);
  digitalWrite(clockEnablePin, HIGH);
  
  cmri.set_byte(5,data1);
  cmri.set_byte(6,data2);

  if (OutputToSerial) {
    if (data1 != previousData1) {
        Serial.println("Byte 1");
        Serial.println(data1, BIN);
        previousData1 = data1;
    }

    if (data2 != previousData2) {
        Serial.println("Byte 2");
        Serial.println(data2, BIN);
        previousData2= data2;
    }
  }
}

void ProcessShiftOut() {
  digitalWrite(latchPinOut, LOW);
  int val1 = cmri.get_byte(6);
  shiftOut(dataPinOut, clockPinOut, MSBFIRST, val1);
  val1 = cmri.get_byte(5);
  shiftOut(dataPinOut, clockPinOut, MSBFIRST, val1);
  digitalWrite(latchPinOut, HIGH);  
}


void SetPinModes()
{
  pinMode(LED_PIN, OUTPUT);

  // Setup 74HC165 connections
  pinMode(load, OUTPUT);
  pinMode(clockEnablePin, OUTPUT);
  pinMode(clockIn, OUTPUT);
  pinMode(dataIn, INPUT);
  pinMode(latchPinOut, OUTPUT);
  pinMode(clockPinOut, OUTPUT);
  pinMode(dataPinOut, OUTPUT); 


  for (int i = 3; i <= 17; i++)
  {
    pinMode(i, INPUT_PULLUP); //Microsensors need pullup
  }
   pinMode(32,INPUT_PULLUP);
  pinMode(33,INPUT_PULLUP);
  pinMode(30,INPUT_PULLUP);
  pinMode(28,INPUT_PULLUP);
  pinMode(22,INPUT_PULLUP);
  pinMode(23,INPUT_PULLUP);
  pinMode(24,INPUT_PULLUP);
  pinMode(25,INPUT_PULLUP);
  //pinMode(30,INPUT_PULLUP);
  pinMode(27,INPUT);
  pinMode(26,INPUT);
  pinMode(36,INPUT);
}

void InitialiseConfig() {
  //Board 1 - under upper incline junction
  PWMBoards[0].pwm = Adafruit_PWMServoDriver(0x40);
  PWMBoards[0].numberOfTurnouts = 6;
  PWMBoards[0].CMRIIndexModifier = 0;
//  PWMBoards[0].turnouts[0] = Turnout(2000, 1100, 12, false,true); //thrown, closed, feedback pin number, invertfrog
//  PWMBoards[0].turnouts[1] = Turnout(1900, 1150, 14, false,true);
//  PWMBoards[0].turnouts[2] = Turnout(1350, 2150, true); //thrown, closed, invertfrog
//  PWMBoards[0].turnouts[3] = Turnout(1250, 2100, false);
//  PWMBoards[0].turnouts[4] = Turnout(2100, 1250, 11, true,false);
//  PWMBoards[0].turnouts[5] = Turnout(1200, 1900, true);

  PWMBoards[0].turnouts[0] = Turnout(2000, 1100,1,0,12, false,true); //thrown, closed, feedback pin number, invertfrog
  PWMBoards[0].turnouts[1] = Turnout(1900, 1150,1,0,14, true,true);
  PWMBoards[0].turnouts[2] = Turnout(1350, 2150,1,0,true); //thrown, closed, invertfrog
  PWMBoards[0].turnouts[3] = Turnout(1250, 2100,1,0,false);
  PWMBoards[0].turnouts[4] = Turnout(2100, 1250,1,0,11, false,false);
  PWMBoards[0].turnouts[5] = Turnout(1200, 1900,1,0, true);

  //Board 2 - by Arduino, corner 2
  PWMBoards[1].pwm = Adafruit_PWMServoDriver(0x41);
  PWMBoards[1].numberOfTurnouts = 8;
  PWMBoards[1].CMRIIndexModifier = 8; //add this value to the standard identifier to get the correct bit
//  PWMBoards[1].turnouts[0] = Turnout(1000, 2100, 4, true,true);
//  PWMBoards[1].turnouts[1] = Turnout(1200, 1800, 3, true,true);
//  PWMBoards[1].turnouts[2] = Turnout(1900, 1100, 7, false,false);
//  PWMBoards[1].turnouts[3] = Turnout(2000, 1200, 6, true,false);
//  PWMBoards[1].turnouts[4] = Turnout(1250, 2150, 8, true,true);
//  PWMBoards[1].turnouts[5] = Turnout(1250, 2100, 9, true,false);
//  PWMBoards[1].turnouts[6] = Turnout(1400, 2250, 5, false,true);
//  PWMBoards[1].turnouts[7] = Turnout(1200, 1900, 10, true,true);

  PWMBoards[1].turnouts[0] = Turnout(1000, 2100,1,0, 4, true,true);
  PWMBoards[1].turnouts[1] = Turnout(1200, 1800,1,0, 3, true,true);
  PWMBoards[1].turnouts[2] = Turnout(1900, 1100,1,0, 7, true,false);
  PWMBoards[1].turnouts[3] = Turnout(2000, 1200,1,0, 6, false,false);
  PWMBoards[1].turnouts[4] = Turnout(1250, 2150,1,0, 8, true,true);
  PWMBoards[1].turnouts[5] = Turnout(1250, 2100,1,0, 9, true,false);
  PWMBoards[1].turnouts[6] = Turnout(1400, 2250,1,0, 5, false,true);
  PWMBoards[1].turnouts[7] = Turnout(1200, 1900,1,0, 10, true,true);

  PWMBoards[0].pwm.begin();
  PWMBoards[0].pwm.setPWMFreq(50);  // This is the maximum PWM frequency
  PWMBoards[0].pwm.setOscillatorFrequency(25000000);

  PWMBoards[1].pwm.begin();
  PWMBoards[1].pwm.setPWMFreq(50);  // This is the maximum PWM frequency
  PWMBoards[1].pwm.setOscillatorFrequency(25000000);
}

void ProcessInputs() {
  //cmri.set_bit(0,digitalRead(12));
  //cmri.set_bit(1,digitalRead(22));

  cmri.set_bit(16,digitalRead(32));  //CD Pi end cw
  cmri.set_bit(17,digitalRead(33));  //CD Pi end AC
  //space for pin 31
  cmri.set_bit(18,digitalRead(28));  //CD Pi end incline
  cmri.set_bit(19,digitalRead(29));  //CW yard entry IR
  cmri.set_bit(20,digitalRead(27));  //CW yard entry lower IR
  cmri.set_bit(21,digitalRead(26));  //AC Yard exut IR

  cmri.set_bit(22,digitalRead(36)); //IR AC yard bypass stopping sensor
  cmri.set_bit(23,digitalRead(25)); //CD CW filler
  cmri.set_bit(24,digitalRead(24)); //CD AC curved point
  cmri.set_bit(25,digitalRead(22)); //CD Incline back
  cmri.set_bit(26,digitalRead(23)); //CD Unused
  //cmri.set_bit(35,digitalRead(36));  //C2AC3IR1
  //cmri.set_bit(36,digitalRead(37));  //C2AC4IR1
  //cmri.set_bit(37,digitalRead(38));  //C2AC3IR1

}
