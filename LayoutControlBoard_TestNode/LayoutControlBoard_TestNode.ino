#include <Auto485.h>
#include <CMRI.h>
#include <Adafruit_PWMServoDriver.h>

#include "PWMBoard.h"
#include "Turnout.h"

#define CMRI_ADDR 1 //Test node
#define NumberOfPWMBoards 1

int feedbackDebounce = 0;
unsigned long startupDelayMillis = 0;
unsigned long startupMillis;
bool startupDelayComplete;
const bool ProtectMotors = false;
const bool OutputToSerial = true;
int numberOfServosMoving = 0;
int numberOfServosMovingLastLoop = 0;
int servoSpeedMultiplier = 1;

////Pin connected to DS of 74HC595 - SERIN
int dataPinOut = 32;
//Pin connected to ST_CP of 74HC595 - RCLCK
int latchPinOut = 31;
//Pin connected to SH_CP of 74HC595 - SRCLCK
int clockPinOut = 30;

#define    DE_PIN 2
#define    RE_PIN 3
#define   LED_PIN 13

PWMBoard PWMBoards[NumberOfPWMBoards];

//Only use Serial1 on a Mega as it has multiple serial buses
Auto485 bus(DE_PIN, RE_PIN, Serial1); // Arduino pin 2 -> MAX485 DE and RE pins
CMRI cmri(CMRI_ADDR, 72, 72, bus); // defaults to a SMINI with address 0. SMINI = 24 inputs, 48 outputs

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
    //Get CMRI data ready for when CMRI does come online
    SetSensorFeedbackReadings();
    ProcessInputs();
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
        if (PWMBoards[pwmIndex].turnouts[i].useSlowMotion == true) {
             moveIsComplete = PWMBoards[pwmIndex].turnouts[i].ProcessPointsMoveWithSpeedControl(PWMBoards[pwmIndex].pwm, deviceStatusFromJMRI, i, numberOfServosMovingLastLoop, servoSpeedMultiplier);
          }
          else {
            PWMBoards[pwmIndex].turnouts[i].ProcessPointsMove(PWMBoards[pwmIndex].pwm, deviceStatusFromJMRI,i);
            moveIsComplete=true;
          }
        if (OutputToSerial) {
          //Serial.println("Set last known value for board "+String(pwmIndex)+" turnout "+String(i)+" to "+String(deviceStatusFromJMRI));
        }   
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
          cmri.set_bit(i + PWMBoards[pwmIndex].CMRIIndexModifier, pinValue);
        }      
    }
  }
  ProcessInputs();
  ProcessShiftOut();
  numberOfServosMovingLastLoop = numberOfServosMoving;
}

void SetSensorFeedbackReadings() {
  for (int pwmIndex = 0; pwmIndex < NumberOfPWMBoards; pwmIndex++) {
    //for each servo on this PWM board
    for (int i = 0; i < PWMBoards[pwmIndex].numberOfTurnouts; i++)
    {
      if (PWMBoards[pwmIndex].turnouts[i].hasFeedbackSensor) {
      int inputPin = PWMBoards[pwmIndex].turnouts[i].feedbackSensorPin;
      bool pinValue = false;
      if (PWMBoards[pwmIndex].turnouts[i].invertFeedbackSensor == true) {
        pinValue = !digitalRead(inputPin);
      }
      else {
        pinValue = digitalRead(inputPin);
      }
      PWMBoards[pwmIndex].turnouts[i].SetRelayAccordingToFeedbackSensor(PWMBoards[pwmIndex].pwm, i, pinValue, feedbackDebounce);
      cmri.set_bit(i + PWMBoards[pwmIndex].CMRIIndexModifier, pinValue);

      if (OutputToSerial) {
        //Serial.println("Set sensor feedback just set lastKnownBitValue");
        //Serial.println("Set CMRI bit " + String(i + PWMBoards[pwmIndex].CMRIIndexModifier) + " for board " + String(pwmIndex) + " turnout " + String(i) + " Arduino pin " + String(inputPin) + " value " + String(pinValue));
        //Serial.println("Get CMRI bit during startup pause "+String(i+PWMBoards[pwmIndex].CMRIIndexModifier)+" for board "+String(pwmIndex)+" turnout "+String(i)+" value "+String(cmri.get_bit(i+PWMBoards[pwmIndex].CMRIIndexModifier)));
      }
      }
    }
  }
}

void ProcessShiftOut() {
  digitalWrite(latchPinOut, LOW);
  int val1 = cmri.get_byte(5);
  int val2 = cmri.get_byte(6);
  //Serial.println(String(val));

  shiftOut(dataPinOut, clockPinOut, MSBFIRST, val2);
  shiftOut(dataPinOut, clockPinOut, MSBFIRST, val1);
  digitalWrite(latchPinOut, HIGH);  
}

void SetPinModes()
{

}

void InitialiseConfig() {
  //Board 1 - under lower incline junction
  PWMBoards[0].pwm = Adafruit_PWMServoDriver();
  PWMBoards[0].numberOfTurnouts = 8;
  PWMBoards[0].CMRIIndexModifier = 0;
  PWMBoards[0].turnouts[0] = Turnout(1000,1800,1,10); //thrown, closed,slow speed step, slow speed delay
  PWMBoards[0].turnouts[1] = Turnout(1150,1650,1,10); 
  PWMBoards[0].turnouts[2] = Turnout(1200,2100,1,10); 
  PWMBoards[0].turnouts[3] = Turnout(1150,2000,1,10); 
  PWMBoards[0].turnouts[4] = Turnout(1250,1700,1,10);
  PWMBoards[0].turnouts[5] = Turnout(1200,1810,1,10);
  PWMBoards[0].turnouts[6] = Turnout(1200,1850,1,10);
  PWMBoards[0].turnouts[7] = Turnout(1250,2000,1,10);

  PWMBoards[0].pwm.begin();
  PWMBoards[0].pwm.setPWMFreq(50);  // This is the maximum PWM frequency
  PWMBoards[0].pwm.setOscillatorFrequency(25000000);

}

void ProcessInputs() {

}
