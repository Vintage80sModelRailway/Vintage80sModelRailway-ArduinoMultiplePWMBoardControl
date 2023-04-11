#include <Auto485.h>
#include <CMRI.h>
#include <Adafruit_PWMServoDriver.h>

#include "PWMBoard.h"
#include "Turnout.h"

#define CMRI_ADDR 2 //Under low incline
#define NumberOfPWMBoards 1

int feedbackDebounce = 0;
unsigned long startupDelayMillis = 10000;
unsigned long startupMillis;
bool startupDelayComplete;
const bool OutputToSerial = false;
const bool ProtectMotors = false;
int numberOfServosMoving = 0;
int numberOfServosMovingLastLoop = 0;
int servoSpeedMultipluer = 1;

////Pin connected to DS of 74HC595 - SERIN
int dataPinOut = 32;
//Pin connected to ST_CP of 74HC595 - RCLCK
int latchPinOut = 31;
//Pin connected to SH_CP of 74HC595 - SRCLCK
int clockPinOut = 30;

#define    DE_PIN 2
#define   LED_PIN 13

PWMBoard PWMBoards[NumberOfPWMBoards];

//Only use Serial1 on a Mega as it has multiple serial buses
Auto485 bus(DE_PIN, DE_PIN, Serial1); // Arduino pin 2 -> MAX485 DE and RE pins
CMRI cmri(CMRI_ADDR, 72, 72, bus); // defaults to a SMINI with address 0. SMINI = 24 inputs, 48 outputs

void setup() {
  
  bus.begin(19200, SERIAL_8N2);
  InitialiseConfig();
  SetPinModes();
  startupDelayComplete = false;
  //Serial.begin(19200);
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
        if (deviceStatusFromJMRI == 1)
        {
          if (OutputToSerial) {
            Serial.println("Throw");
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
            Serial.println("Close");
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
          Serial.println("Set last known value for board "+String(pwmIndex)+" turnout "+String(i)+" to "+String(deviceStatusFromJMRI));
        }
        PWMBoards[pwmIndex].turnouts[i].motorHasNotMovedYet  = false;        
      }
      else
      {
        if (OutputToSerial) {
          Serial.println("Get CMRI bit "+String(i+PWMBoards[pwmIndex].CMRIIndexModifier)+" for board "+String(pwmIndex)+" turnout "+String(i)+" value "+String(deviceStatusFromJMRI));
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
            Serial.println("Setting feedback sensor for CMRI bit "+String(i+PWMBoards[pwmIndex].CMRIIndexModifier)+" to "+String(pinValue));
          }
          SetTurnoutFeedback(pwmIndex, i, pinValue);
        }      
    }
  }
  ProcessInputs();
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
      PWMBoards[board].turnouts[pin].currentPWMVal = requiredPosition;      
    }
    else
    {
      if (OutputToSerial) {
        Serial.println("Points move not required on board " + String(board) + ", device " + String(pin) + " to position " + String(requiredPosition) + ".");
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

                if (requiredPosition == intendedPWMValue)
                {
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

                if (requiredPosition == intendedPWMValue)
                {
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
      PWMBoards[pwmIndex].turnouts[i].lastKnownBitValue = pinValue;// cmri.get_bit(i + PWMBoards[pwmIndex].CMRIIndexModifier);
      if (PWMBoards[pwmIndex].turnouts[i].lastKnownBitValue == 0) {
          if (PWMBoards[pwmIndex].turnouts[i].currentPWMVal != PWMBoards[pwmIndex].turnouts[i].closedVal) {
             //PWMBoards[pwmIndex].pwm.writeMicroseconds(i, PWMBoards[pwmIndex].turnouts[i].closedVal);
          }
           PWMBoards[pwmIndex].turnouts[i].currentPWMVal =  PWMBoards[pwmIndex].turnouts[i].closedVal;           
        }
        else {
          if (PWMBoards[pwmIndex].turnouts[i].currentPWMVal != PWMBoards[pwmIndex].turnouts[i].thrownVal) {
             //PWMBoards[pwmIndex].pwm.writeMicroseconds(i, PWMBoards[pwmIndex].turnouts[i].thrownVal);
          }          
          PWMBoards[pwmIndex].turnouts[i].currentPWMVal =  PWMBoards[pwmIndex].turnouts[i].thrownVal;
        }
      if (OutputToSerial) {
        Serial.println("Set sensor feedback just set lastKnownBitValue");
        Serial.println("Set CMRI bit " + String(i + PWMBoards[pwmIndex].CMRIIndexModifier) + " for board " + String(pwmIndex) + " turnout " + String(i) + " Arduino pin " + String(inputPin) + " value " + String(pinValue));
        Serial.println("Get CMRI bit during startup pause "+String(i+PWMBoards[pwmIndex].CMRIIndexModifier)+" for board "+String(pwmIndex)+" turnout "+String(i)+" value "+String(cmri.get_bit(i+PWMBoards[pwmIndex].CMRIIndexModifier)));
      }
    }
  }
}

void SetTurnoutFeedback(int board, int pin, bool pinValue) {
  if ((PWMBoards[board].turnouts[pin].hasFeedbackSensor && PWMBoards[board].turnouts[pin].lastFeedbackSensorReading != pinValue) || PWMBoards[board].turnouts[pin].relayHasNotBeenSetYet)
  {
    if (PWMBoards[board].turnouts[pin].inDebounce == false)
    {
      if (OutputToSerial) {
        Serial.println("Setting to in debounce");
      }

      PWMBoards[board].turnouts[pin].millisAtLastChange = millis();
      PWMBoards[board].turnouts[pin].inDebounce = true;
    }
    unsigned long millisSinceLastChange = (millis() - PWMBoards[board].turnouts[pin].millisAtLastChange);
    if (OutputToSerial) {
      Serial.println("Millis since last change "+String(millisSinceLastChange)+" pin value "+String(pinValue));
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
    if (PWMBoards[board].turnouts[pin].inDebounce == false)
    {
      if (OutputToSerial) {
        Serial.println("Setting to in debounce");
      }

      PWMBoards[board].turnouts[pin].millisAtLastChange = millis();
      PWMBoards[board].turnouts[pin].inDebounce = true;
    }
    unsigned long millisSinceLastChange = (millis() - PWMBoards[board].turnouts[pin].millisAtLastChange);
    if (OutputToSerial) {
      Serial.println("Millis since last change "+String(millisSinceLastChange)+" pin value "+String(pinValue));
    }

    if (millisSinceLastChange >= feedbackDebounce)
    {
        bool fullyOn = pinValue != 0;
        if (fullyOn == true)
        {
          if (PWMBoards[board].turnouts[pin].invertFrog == false)
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
          if (PWMBoards[board].turnouts[pin].invertFrog == false)
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

      cmri.set_bit(pin + PWMBoards[board].CMRIIndexModifier, pinValue);
      PWMBoards[board].turnouts[pin].lastFeedbackSensorReading = pinValue;
      PWMBoards[board].turnouts[pin].inDebounce = false;
      PWMBoards[board].turnouts[pin].relayHasNotBeenSetYet = false;
    }
    else
    {
      if (OutputToSerial) {
        Serial.println("Board "+String(board)+"Pin "+String(pin)+" Switch position "+String(pinValue)+" last sensor reading "+String(PWMBoards[board].turnouts[pin].lastFeedbackSensorReading));
      }

    }
  }
  else
  {
    //Status change has flicked on then off, or off then on, within the debounce period so don't change anything and set inDebounce back to false
    PWMBoards[board].turnouts[pin].inDebounce = false;
  }
}

void setPWMStateFullyOn(int board, int pin)
{
  if (pin < 8) return;
  if (OutputToSerial) {
    Serial.println("Set relay fully on for board "+String(board)+",  pin "+String(pin));
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
    Serial.println("Set relay fully off for board "+String(board)+",  pin "+String(pin));
  }
}

void ProcessShiftOut() {
  digitalWrite(latchPinOut, LOW);
  int val1 = cmri.get_byte(5);
  int val2 = cmri.get_byte(6);

  shiftOut(dataPinOut, clockPinOut, MSBFIRST, val2);
  shiftOut(dataPinOut, clockPinOut, MSBFIRST, val1);
  digitalWrite(latchPinOut, HIGH);
  
}

void SetPinModes() {

    pinMode(latchPinOut, OUTPUT);
    pinMode(clockPinOut, OUTPUT);
    pinMode(dataPinOut, OUTPUT);
    
    pinMode(LED_PIN, OUTPUT);
    
    pinMode(34, INPUT_PULLUP);
    pinMode(35, INPUT_PULLUP);
    pinMode(36, INPUT_PULLUP);
    pinMode(37, INPUT_PULLUP);
    pinMode(38, INPUT_PULLUP);
    pinMode(39, INPUT_PULLUP);

    for (int i = 3; i <= 19; i++)
    {
        pinMode(i, INPUT_PULLUP);
    }
}

void InitialiseConfig() {
  //Board 1 - under lower incline junction
  PWMBoards[0].pwm = Adafruit_PWMServoDriver();
  PWMBoards[0].numberOfTurnouts = 8;
  PWMBoards[0].CMRIIndexModifier = 0;
  PWMBoards[0].turnouts[0] = Turnout(1000,1800,1,0,3,false,true); //thrown, closed,feedback sensor pin, invertfrog, invert sensor
  PWMBoards[0].turnouts[1] = Turnout(1150,1650,1,0,4,false,false); //thrown, closed, invertfrog
  PWMBoards[0].turnouts[2] = Turnout(1200,2100,1,0,7,true,true); //thrown, closed, invertfrog
  PWMBoards[0].turnouts[3] = Turnout(1150,2000,1,0,10,false,false); //thrown, closed, invertfrog 
  PWMBoards[0].turnouts[4] = Turnout(1250,1700,1,0,8,false,false); //thrown, closed, invertfrog 
  PWMBoards[0].turnouts[5] = Turnout(1200,1810,1,0,9,true,false); //thrown, closed, invertfrog  
  PWMBoards[0].turnouts[6] = Turnout(1200,1850,1,0,6,false,false); //thrown, closed, invertfrog  
  PWMBoards[0].turnouts[7] = Turnout(1250,2000,1,0,5,true,false); //thrown, closed, invertfrog

  PWMBoards[0].pwm.begin();
  PWMBoards[0].pwm.setPWMFreq(50);  // This is the maximum PWM frequency
  PWMBoards[0].pwm.setOscillatorFrequency(25000000);

}

void ProcessInputs() {
   // PROCESS SENSORS
   
     // Do not read 0, 1 or 2
     //Do not read 13
     //Do not read 20 or 21  

     cmri.set_bit(35,digitalRead(36));  //AC lower junction BLUE
     cmri.set_bit(36,digitalRead(37));  //incline junction PURPLE
     cmri.set_bit(34,digitalRead(35));  //CW lower junction PC End
     cmri.set_bit(33,digitalRead(34));  //CW lower junction Pi End
     cmri.set_bit(32,digitalRead(33));  //CW lower junction Pi End
     cmri.set_bit(37,digitalRead(38));  //Split AC
     cmri.set_bit(38,digitalRead(39));  //Spare

}
