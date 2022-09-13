#include <Auto485.h>
#include <CMRI.h>
#include <Adafruit_PWMServoDriver.h>

#include "PWMBoard.h"
#include "Turnout.h"

#define CMRI_ADDR 2 //Under low incline
#define NumberOfPWMBoards 1

int feedbackDebounce = 50;
unsigned long startupDelayMillis = 40000;
unsigned long startupMillis;
bool startupDelayComplete;

#define    DE_PIN 2
#define   LED_PIN 13

PWMBoard PWMBoards[NumberOfPWMBoards];

//Only use Serial1 on a Mega as it has multiple serial buses
Auto485 bus(DE_PIN); // Arduino pin 2 -> MAX485 DE and RE pins
CMRI cmri(CMRI_ADDR, 64, 32, bus); // defaults to a SMINI with address 0. SMINI = 24 inputs, 48 outputs

void setup() {
    InitialiseConfig();
    SetPinModes();    
    startupDelayComplete = false;
    
    bus.begin(19200, SERIAL_8N2);
    //Serial.println("CMRI Address "+String(CMRI_ADDR));    

//    int testPin = 12;
//    int originalPinValue = digitalRead(testPin);
//    bool pinHasChanged = false;

    bool cmriIsInitialised = false;
    
    while (!cmriIsInitialised) {
//      Test stuff
//      int pinVal = digitalRead(testPin);
//      if (pinVal != originalPinValue)
//      {
//        pinHasChanged = true;
//      }
//      Serial.println("Value of pin "+String(testPin)+" input sensor is "+String(pinVal));
//      if (pinHasChanged)
//      {
//        Serial.println("Pin has changed");
//      }

      
        //Get CMRI data ready for when CMRI does come online
        //Serial.println("Pin test "+String(digitalRead(4)));
        SetSensorFeedbackReadings();
        ProcessInputs();
      
       cmriIsInitialised = cmri.process();
       //Serial.println("Not ready");
    }
    startupMillis = millis();
    //Serial.println("CMRI / RS485 Ready");
}

void loop() {
  //unsigned long millisSinceCMRIStarted = millis() - startupMillis;
  //At startup, JMRI / CMRI just send a load of 0s in the data packet, which sends all motors crazy.
  //It also sends that data before it polls, which means all motors move
  //Trying here to delay acting on CMRI data until adter it's polled and recieved statuses from the feedback sensors
  //Then everything should get set to how it currently is with no craziness
  bool cmriIsRunning = cmri.process();
  if (!startupDelayComplete)
  {
    unsigned long millisSinceCMRIStarted = millis() - startupMillis;    

    if (millisSinceCMRIStarted < startupDelayMillis)
    {
      //Serial.println("Pin test "+String(digitalRead(4)));
      SetSensorFeedbackReadings();
      ProcessInputs();
      //Serial.println("CMRI start delay - waiting for "+String(startupDelayMillis)+" millis, currently "+String(millisSinceCMRIStarted));
      //Now exit the loop
      return;
    }
    else
    {
      startupDelayComplete = true;
      //Serial.println("Delay complete");
      for (int pwmIndex = 0; pwmIndex < NumberOfPWMBoards; pwmIndex++) {
        //for each servo on this PWM board
        for (int i = 0; i < PWMBoards[pwmIndex].numberOfServos; i++)
        {
          if (!PWMBoards[pwmIndex].turnouts[i].hasFeedbackSensor) 
          {
            SetRelayAccordingToCMRIBitValue(pwmIndex,i,cmri.get_bit(i+PWMBoards[pwmIndex].CMRIIndexModifier));

          }
        }
      }      
    }
  }
  
  
//  if (!cmriIsRunning)
//  {
//    SetSensorFeedbackReadings();
//    ProcessInputs();
//    return;
//  }  

  //for each PWM board
  for (int pwmIndex = 0; pwmIndex < NumberOfPWMBoards; pwmIndex++) {
    //for each servo on this PWM board
    for (int i = 0; i < PWMBoards[pwmIndex].numberOfServos; i++)
    {
        int deviceStatusFromJMRI = cmri.get_bit(i+PWMBoards[pwmIndex].CMRIIndexModifier);
        if (deviceStatusFromJMRI != PWMBoards[pwmIndex].turnouts[i].lastKnownBitValue)
        {
          //Serial.println("Bit value change on board "+String(pwmIndex)+", device " + String(i) + " JMRI status "+String(deviceStatusFromJMRI)+" last known bit "+String( PWMBoards[pwmIndex].turnouts[i].lastKnownBitValue)+" motor not moved yet "+String(PWMBoards[pwmIndex].turnouts[i].motorHasNotMovedYet));
          if (deviceStatusFromJMRI == 1)
          {
            //Serial.println("Throw");
              int throwValue = PWMBoards[pwmIndex].turnouts[i].thrownVal;
              if (PWMBoards[pwmIndex].turnouts[i].useSlowMotion == true) {
                ProcessPointsMoveWithSpeedControl(pwmIndex, i, throwValue);
              }
              else {
                ProcessPointsMove(pwmIndex, i, throwValue, deviceStatusFromJMRI);
              }              
          }
          else
          {
              //Serial.println("Close");
              int closeValue = PWMBoards[pwmIndex].turnouts[i].closedVal;
              if (PWMBoards[pwmIndex].turnouts[i].useSlowMotion == true) {
                ProcessPointsMoveWithSpeedControl(pwmIndex, i, closeValue);
              }
              else {
                ProcessPointsMove(pwmIndex, i, closeValue, deviceStatusFromJMRI);
              }
          }
          
          PWMBoards[pwmIndex].turnouts[i].lastKnownBitValue = deviceStatusFromJMRI;
          //Serial.println("Set last known value for board "+String(pwmIndex)+" turnout "+String(i)+" to "+String(deviceStatusFromJMRI));
          PWMBoards[pwmIndex].turnouts[i].motorHasNotMovedYet  = false;
       }
       else
       {
          //Serial.println("Get CMRI bit "+String(i+PWMBoards[pwmIndex].CMRIIndexModifier)+" for board "+String(pwmIndex)+" turnout "+String(i)+" value "+String(deviceStatusFromJMRI));
       }
      
      //set feedback sensor bit if using - only need to set when a points motor has not moved
      if (PWMBoards[pwmIndex].turnouts[i].hasFeedbackSensor) {
              int inputPin = PWMBoards[pwmIndex].turnouts[i].feedbackSensorPin;
              bool pinValue = digitalRead(inputPin);
    
              //Serial.println("Setting feedback sensor for CMRI bit "+String(i+PWMBoards[pwmIndex].CMRIIndexModifier)+" to "+String(pinValue));
              SetRelayAccordingToFeedbackSensor(pwmIndex,i,pinValue);         
     }         
    }
  }
  ProcessInputs();  
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
            //Serial.println("Points move required on board "+String(board)+", device " + String(pin) + " to position " + String(requiredPosition) + ".");
            PWMBoards[board].pwm.writeMicroseconds(pin, requiredPosition);
             //Frog polarity - setting bssed on feedback sensor value handled elsewhere
             if (!PWMBoards[board].turnouts[pin].hasFeedbackSensor) {
                bool fullyOn = cmriBitValue != 0;

                if (fullyOn == true)
                {
                  //Serial.println("Fully on");
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
                  //Serial.println("Fully off");
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
            PWMBoards[board].turnouts[pin].currentPWMVal = requiredPosition;
        }
        else
        {
          //Serial.println("Points move not required on board "+String(board)+", device " + String(pin) + " to position " + String(requiredPosition) + ".");
        }
    }
}

void ProcessPointsMoveWithSpeedControl(int board, int pin, int requiredPosition)
{
    unsigned long currentMillis = millis();
    if (requiredPosition != PWMBoards[board].turnouts[pin].currentPWMVal && currentMillis - PWMBoards[board].turnouts[pin].previousMillis >= PWMBoards[board].turnouts[pin].delayTime)
    {
        //Serial.println("Inside first check");
        PWMBoards[board].turnouts[pin].previousMillis = currentMillis;
        
        //Protect thee motors from accidentally silly, potentially damaging values
        if (requiredPosition > 800 && requiredPosition < 2200)
        {
            if (requiredPosition > PWMBoards[board].turnouts[pin].currentPWMVal)
            {
                int intendedPWMValue = PWMBoards[board].turnouts[pin].currentPWMVal + PWMBoards[board].turnouts[pin].stepSize;
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
                  if (PWMBoards[board].turnouts[pin].invertFrog)
                  {
                    //Serial.println("Setting frog polarity for board "+String(board)+", pin "+String(pin));
                    setPWMStateFullyOn(board, pin + 8);
                  }
                  else
                  {
                    //Serial.println("Setting frog polarity inverted for board "+String(board)+", pin "+String(pin));
                    setPWMStateFullyOff(board, pin + 8);
                  }
                    
                }
            }
            else// (requiredPosition < CurrentPWMValue[pin])
            {
                int intendedPWMValue = PWMBoards[board].turnouts[pin].currentPWMVal - PWMBoards[board].turnouts[pin].stepSize;
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
                  if (PWMBoards[board].turnouts[pin].invertFrog)
                  {
                    //Serial.println("Setting frog polarity for board "+String(board)+", pin "+String(pin));
                    setPWMStateFullyOff(board, pin + 8);
                  }
                  else
                  {
                    //Serial.println("Setting frog polarity inverted forr board "+String(board)+", pin "+String(pin));
                    setPWMStateFullyOn(board, pin + 8);
                  }
                }
            }
        }
    }
}

void SetSensorFeedbackReadings() {
    
    for (int pwmIndex = 0; pwmIndex < NumberOfPWMBoards; pwmIndex++) {
    //for each servo on this PWM board
    for (int i = 0; i < PWMBoards[pwmIndex].numberOfServos; i++)
    {
        int inputPin = PWMBoards[pwmIndex].turnouts[i].feedbackSensorPin;
        bool pinValue = digitalRead(inputPin);
        SetRelayAccordingToFeedbackSensor(pwmIndex,i,pinValue);
        cmri.set_bit(i+PWMBoards[pwmIndex].CMRIIndexModifier, pinValue);
        PWMBoards[pwmIndex].turnouts[i].lastFeedbackSensorReading = pinValue;
        PWMBoards[pwmIndex].turnouts[i].lastKnownBitValue = cmri.get_bit(i+PWMBoards[pwmIndex].CMRIIndexModifier);
        //Serial.println("Set sensor feedback just set lastKnownBitValue");
        //Serial.println("Set CMRI bit "+String(i+PWMBoards[pwmIndex].CMRIIndexModifier)+" for board "+String(pwmIndex)+" turnout "+String(i)+" Arduino pin "+String(inputPin)+" value "+String(pinValue));
        //Serial.println("Get CMRI bit during startup pause "+String(i+PWMBoards[pwmIndex].CMRIIndexModifier)+" for board "+String(pwmIndex)+" turnout "+String(i)+" value "+String(cmri.get_bit(i+PWMBoards[pwmIndex].CMRIIndexModifier)));
    }
  }
} 

void SetRelayAccordingToFeedbackSensor(int board, int pin, bool pinValue) {
    //This could be the initial JMRI startup sequence, called from setup -there may be a route set after this
    //So not setting 'motothasnotmovedyet' to false - this will be done on the first run through the turnouts in loop
    //Serial.println("Set relay board "+String(board)+" pin "+String(pin) +" relay not moved "+String(PWMBoards[board].turnouts[pin].relayHasNotBeenSetYet));
    if ((PWMBoards[board].turnouts[pin].hasFeedbackSensor && PWMBoards[board].turnouts[pin].lastFeedbackSensorReading != pinValue) || PWMBoards[board].turnouts[pin].relayHasNotBeenSetYet)  
    {
      //Serial.println("Relay not moved");
      if (PWMBoards[board].turnouts[pin].inDebounce == false)
      {
        //Serial.println("Setting to in debounce");
        PWMBoards[board].turnouts[pin].millisAtLastChange = millis();
        PWMBoards[board].turnouts[pin].inDebounce = true;
      }
      unsigned long millisSinceLastChange = (millis() - PWMBoards[board].turnouts[pin].millisAtLastChange);
      //Serial.println("Millis since last change "+String(millisSinceLastChange)+" pin value "+String(pinValue));
      if (millisSinceLastChange >= feedbackDebounce)
      {                
        if (PWMBoards[board].turnouts[pin].invertFrog == true) {
         // Serial.println("Setting relay for Board "+String(board)+" Pin "+String(pin)+" Switch position "+String(pinValue)+" last sensor reading "+String(PWMBoards[board].turnouts[pin].lastFeedbackSensorReading));
          setRelay(board,pin+8, pinValue);
        }
        else {
          //Serial.println("Setting relay inverted for Board "+String(board)+" Pin "+String(pin)+" Switch position "+String(pinValue)+" last sensor reading "+String(PWMBoards[board].turnouts[pin].lastFeedbackSensorReading));
          setRelay(board,pin+8,!pinValue);
        }

        cmri.set_bit(pin+PWMBoards[board].CMRIIndexModifier, pinValue);
        PWMBoards[board].turnouts[pin].lastFeedbackSensorReading = pinValue;
        PWMBoards[board].turnouts[pin].inDebounce = false;
        PWMBoards[board].turnouts[pin].relayHasNotBeenSetYet = false;
      }
      else
      {
        //Serial.println("Board "+String(board)+"Pin "+String(pin)+" Switch position "+String(pinValue)+" last sensor reading "+String(PWMBoards[board].turnouts[pin].lastFeedbackSensorReading));
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
    //Serial.println("Fully on ");
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
    //Serial.println("Fully off");
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
    //Serial.println("Set relay fully on for board "+String(board)+",  pin "+String(pin));
    PWMBoards[board].pwm.setPWM(pin, 4096, 0);
}

void setPWMStateFullyOff(int board, int pin)
{
  if (pin < 8) return;
    PWMBoards[board].pwm.setPWM(pin, 0, 4096);
    //Serial.println("Set relay fully off for board "+String(board)+",  pin "+String(pin));
}

void setRelay(int board, int pin, bool setting) {
  if (pin < 8) return;
  PWMBoards[board].pwm.setPWM(pin, setting == true ? 4096 : 0, setting == true ? 0 :4096);
  //Serial.println("Set relay board "+String(board)+ "pin "+String(pin)+" setting "+String(setting));
}

void SetPinModes()
{
    pinMode(LED_PIN, OUTPUT);
    // Set pins 3-19 and 22-69 as input pins for sensors

    for (int i = 3; i <= 19; i++)
    {
        pinMode(i, INPUT_PULLUP);       // set sensor shield pins 3-19 as inputs
    }

    for (int i = 22; i <= 53; i++)
    {
        pinMode(i, INPUT_PULLUP);       // set sensor shield pins 22-53 as inputs
    }
}

void InitialiseConfig() {
  //Board 1 - under lower incline junction
  PWMBoards[0].pwm = Adafruit_PWMServoDriver();
  PWMBoards[0].numberOfServos = 8;
  PWMBoards[0].CMRIIndexModifier = 0;
  PWMBoards[0].turnouts[0] = Turnout(1000,1800,3,false); //thrown, closed,feedback sensor pin, invertfrog, invert sensor
  PWMBoards[0].turnouts[1] = Turnout(1150,1650,4,false); //thrown, closed, invertfrog
  PWMBoards[0].turnouts[2] = Turnout(1200,2100,7,true); //thrown, closed, invertfrog
  PWMBoards[0].turnouts[3] = Turnout(1150,2000,10,true); //thrown, closed, invertfrog 
  PWMBoards[0].turnouts[4] = Turnout(1250,1700,8,false); //thrown, closed, invertfrog 
  PWMBoards[0].turnouts[5] = Turnout(1300,1810,9,true); //thrown, closed, invertfrog  
  PWMBoards[0].turnouts[6] = Turnout(1200,1850,6,false); //thrown, closed, invertfrog  
  PWMBoards[0].turnouts[7] = Turnout(1250,2000,5,true); //thrown, closed, invertfrog  

  PWMBoards[0].pwm.begin();
  PWMBoards[0].pwm.setPWMFreq(50);  // This is the maximum PWM frequency
  PWMBoards[0].pwm.setOscillatorFrequency(25000000);

}

void ProcessInputs() {
   // PROCESS SENSORS
   
     // Do not read 0, 1 or 2
     //Do not read 13
     //Do not read 20 or 21  

     cmri.set_bit(33,digitalRead(34));  //Bit 1 = address 1002 in JMRI

}
