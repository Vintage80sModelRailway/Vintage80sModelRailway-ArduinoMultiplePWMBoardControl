#include <Auto485.h>
#include <CMRI.h>
#include <Adafruit_PWMServoDriver.h>

#include "PWMBoard.h"
#include "Turnout.h"

#define CMRI_ADDR 1 //Corner 2
#define NumberOfPWMBoards 2
int feedbackDebounce = 50;
unsigned long startupDelayMillis = 20000;
unsigned long startupMillis;

#define    DE_PIN 2
#define   LED_PIN 13

PWMBoard PWMBoards[NumberOfPWMBoards];

//Only use Serial1 on a Mega as it has multiple serial buses
Auto485 bus(DE_PIN); // Arduino pin 2 -> MAX485 DE and RE pins
CMRI cmri(CMRI_ADDR, 64, 32, bus); // defaults to a SMINI with address 0. SMINI = 24 inputs, 48 outputs

void setup() {
    InitialiseConfig();
    SetPinModes();    
    
    bus.begin(19200, SERIAL_8N2);

    //Serial.println("CMRI Address "+String(CMRI_ADDR));    
    //Serial.println("Test");

    bool cmriIsInitialised = false;

//    int testPin = 12;
//    int originalPinValue = digitalRead(testPin);
//    bool pinHasChanged = false;
    
    while (!cmriIsInitialised) {
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
        SetSensorFeedbackReadings();
        ProcessInputs();
      
       cmriIsInitialised = cmri.process();
       //Serial.println("Not ready");
    }

    //Log what time connection was established with CMRI
    //startupMillis = millis();
    //Serial.println("Ready");
    //SetSensorFeedbackReadings();
}

void loop() {
  //unsigned long millisSinceCMRIStarted = millis() - startupMillis;
  //At startup, JMRI / CMRI just send a load of 0s in the data packet, which sends all motors crazy.
  //It also sends that data before it polls, which means all motors move
  //Trying here to delay acting on CMRI data until adter it's polled and recieved statuses from the feedback sensors
  //Then everything should get set to how it currently is with no craziness
  
  bool cmriIsRunning = cmri.process();
//  if (millisSinceCMRIStarted < startupDelayMillis)
//  {
//    SetSensorFeedbackReadings();
//    ProcessInputs();
//    //Serial.println("CMRI start delay - waiting for "+String(startupDelayMillis)+" millis, currently "+String(millisSinceCMRIStarted));
//    //Now exit the loop
//    return;
//  }

  if (!cmriIsRunning)
  {
    SetSensorFeedbackReadings();
    ProcessInputs();
    return;
  }

  //Serial.println("Start delay complete");

  //for each PWM board
  for (int pwmIndex = 0; pwmIndex < NumberOfPWMBoards; pwmIndex++) {
    //for each servo on this PWM board
    for (int i = 0; i < PWMBoards[pwmIndex].numberOfServos; i++)
    {
        int deviceStatusFromJMRI = cmri.get_bit(i+PWMBoards[pwmIndex].CMRIIndexModifier);
        if (deviceStatusFromJMRI != PWMBoards[pwmIndex].turnouts[i].lastKnownBitValue)// || PWMBoards[pwmIndex].turnouts[i].motorHasNotMovedYet)
        {
          //Serial.println("Bit value change on board "+String(pwmIndex)+", device " + String(i) + " JMRI status "+String(deviceStatusFromJMRI));

          if (deviceStatusFromJMRI == 1)
          {
            //Serial.println("Throw");
              int throwValue = PWMBoards[pwmIndex].turnouts[i].thrownVal;
              if (PWMBoards[pwmIndex].turnouts[i].useSlowMotion == true) {
                ProcessPointsMoveWithSpeedControl(pwmIndex, i, throwValue);
              }
              else {
                ProcessPointsMove(pwmIndex, i, throwValue);
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
                ProcessPointsMove(pwmIndex, i, closeValue);
              }
          }
          
          PWMBoards[pwmIndex].turnouts[i].lastKnownBitValue = deviceStatusFromJMRI;
          PWMBoards[pwmIndex].turnouts[i].motorHasNotMovedYet  = false;
       }
       else
       {
          //Serial.println("Get CMRI bit "+String(i+PWMBoards[pwmIndex].CMRIIndexModifier)+" for board "+String(pwmIndex)+" turnout "+String(i)+" value "+String(cmri.get_bit(i+PWMBoards[pwmIndex].CMRIIndexModifier)));
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

void ProcessPointsMove(int board, int pin, int requiredPosition)
{
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
                bool fullyOn = PWMBoards[board].turnouts[pin].currentPWMVal > requiredPosition;

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
        PWMBoards[pwmIndex].turnouts[i].lastKnownBitValue = cmri.get_bit(cmri.get_bit(i+PWMBoards[pwmIndex].CMRIIndexModifier));
        //Serial.println("Set CMRI bit "+String(i+PWMBoards[pwmIndex].CMRIIndexModifier)+" for board "+String(pwmIndex)+" turnout "+String(i)+" Arduino pin "+String(inputPin)+" value "+String(pinValue));
        //Serial.println("Get CMRI bit "+String(i+PWMBoards[pwmIndex].CMRIIndexModifier)+" for board "+String(pwmIndex)+" turnout "+String(i)+" value "+String(cmri.get_bit(i+PWMBoards[pwmIndex].CMRIIndexModifier)));
    }
  }
} 

void SetRelayAccordingToFeedbackSensor(int board, int pin, bool pinValue) {
    //This could be the initial JMRI startup sequence, called from setup -there may be a route set after this
    //So not setting 'motothasnotmovedyet' to false - this will be done on the first run through the turnouts in loop
    
    if ((PWMBoards[board].turnouts[pin].hasFeedbackSensor && PWMBoards[board].turnouts[pin].lastFeedbackSensorReading != pinValue) || PWMBoards[board].turnouts[pin].relayHasNotBeenSetYet)  
    {
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
          //Serial.println("Setting relay for Board "+String(board)+" Pin "+String(pin)+" Switch position "+String(pinValue)+" last sensor reading "+String(PWMBoards[board].turnouts[pin].lastFeedbackSensorReading));
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

void ProcessInputs() {
   // PROCESS SENSORS
   
     // Do not read 0, 1 or 2
     //Do not read 13
     //Do not read 20 or 21  
    //cmri.set_bit(0,digitalRead(12));
    //cmri.set_bit(1,digitalRead(22));
     cmri.set_bit(33,digitalRead(34));  //First IR sensor on Yard line AC 3

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
  Serial.println("Set relay board "+String(board)+ "pin "+String(pin)+" setting "+String(setting));
}

void InitialiseConfig() {
  //Board 1 - under upper incline junction
  PWMBoards[0].pwm = Adafruit_PWMServoDriver(0x40);
  PWMBoards[0].numberOfServos = 6;
  PWMBoards[0].CMRIIndexModifier = 0;
  PWMBoards[0].turnouts[0] = Turnout(1100,2000,12,false); //thrown, closed, feedback pin number, invertfrog
  PWMBoards[0].turnouts[1] = Turnout(1250,1800,22,true);
  PWMBoards[0].turnouts[2] = Turnout(1350,2150,true); //thrown, closed, invertfrog
  PWMBoards[0].turnouts[3] = Turnout(1350,2200,false);
  PWMBoards[0].turnouts[4] = Turnout(1250,2100,11,true);
  PWMBoards[0].turnouts[5] = Turnout(1200,1900,true); 

  //Board 2 - by Arduino, corner 2
  PWMBoards[1].pwm = Adafruit_PWMServoDriver(0x41);
  PWMBoards[1].numberOfServos = 8;
  PWMBoards[1].CMRIIndexModifier = 8; //add this value to the standard identifier to get the correct bit 
  PWMBoards[1].turnouts[0] = Turnout(1000,2100,4,true);
  PWMBoards[1].turnouts[1] = Turnout(1200,1800,3,true);
  PWMBoards[1].turnouts[2] = Turnout(1100,1900,7,false);
  PWMBoards[1].turnouts[3] = Turnout(1200,2000,6,true);
  PWMBoards[1].turnouts[4] = Turnout(1250,2150,8,true);
  PWMBoards[1].turnouts[5] = Turnout(1250,2100,9,false);
  PWMBoards[1].turnouts[6] = Turnout(1400,2250,33,false);
  PWMBoards[1].turnouts[7] = Turnout(1200,1900,10,true);

  PWMBoards[0].pwm.begin();
  PWMBoards[0].pwm.setPWMFreq(50);  // This is the maximum PWM frequency
  PWMBoards[0].pwm.setOscillatorFrequency(25000000);
  
  PWMBoards[1].pwm.begin();
  PWMBoards[1].pwm.setPWMFreq(50);  // This is the maximum PWM frequency
  PWMBoards[1].pwm.setOscillatorFrequency(25000000);

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