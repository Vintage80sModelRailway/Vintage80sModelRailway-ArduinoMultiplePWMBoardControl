#include <Auto485.h>
#include <CMRI.h>
#include <Adafruit_PWMServoDriver.h>

#include "PWMBoard.h"
#include "Turnout.h"

#define CMRI_ADDR 1 //Corner 2
#define NumberOfPWMBoards 2

#define    DE_PIN 2
#define   LED_PIN 13

PWMBoard PWMBoards[NumberOfPWMBoards];

//Only use Serial1 on a Mega as it has multiple serial buses
Auto485 bus(DE_PIN,Serial1); // Arduino pin 2 -> MAX485 DE and RE pins
CMRI cmri(CMRI_ADDR, 64, 32, bus); // defaults to a SMINI with address 0. SMINI = 24 inputs, 48 outputs

void setup() {
    InitialiseConfig();
    SetPinModes();    
    
    bus.begin(115200, SERIAL_8N2);
    Serial.begin(115200);

    SetSensorFeedbackReadings();
    //Serial.println("CMRI Address "+String(CMRI_ADDR));    
    //Serial.println("Test");

    bool cmriIsInitialised = false;
    while (!cmriIsInitialised) {
       cmriIsInitialised = cmri.process();
       Serial.println("Not ready");
    }
}

void loop() {
  // put your main code here, to run repeatedly:

  //debug
  Serial.println("Ready");
  //Serial.println("Value of pin 34 input sensor is "+String(digitalRead(8)));
  
  cmri.process();
  //for each PWM board
  for (int pwmIndex = 0; pwmIndex < NumberOfPWMBoards; pwmIndex++) {
    //for each servo on this PWM board
    for (int i = 0; i < PWMBoards[pwmIndex].numberOfServos; i++)
    {
        int deviceStatusFromJMRI = cmri.get_bit(i+PWMBoards[pwmIndex].CMRIIndexModifier);
        if (deviceStatusFromJMRI != PWMBoards[pwmIndex].turnouts[i].lastKnownBitValue) {
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
       }

      //debug
      if (pwmIndex == 1 && i == 0) {
        //Serial.println("No change on board "+String(pwmIndex)+", device " + String(i) + " JMRI status "+String(deviceStatusFromJMRI));
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
          
          bool lastReadSensorValue = PWMBoards[pwmIndex].turnouts[i].lastFeedbackSensorReading;
          if (lastReadSensorValue != pinValue) {
            //Serial.println();
            //Serial.println("Setting feedback sensor for CMRI bit "+String(i+PWMBoards[pwmIndex].CMRIIndexModifier)+" to "+String(pinValue));
            cmri.set_bit(i+PWMBoards[pwmIndex].CMRIIndexModifier, pinValue);
          }
          PWMBoards[pwmIndex].turnouts[i].lastFeedbackSensorReading = pinValue;              
      }                     
    }
  }
  //bool testbit = digitalRead(34);
  //cmri.set_bit(33, testbit);  //Yard IR Sensor 1
  //ProcessInputs();
  
}

void ProcessPointsMove(int board, int pin, int requiredPosition)
{
    //Only instruct the servo to move if its required PWM value is different from its current one
    if (requiredPosition != PWMBoards[board].turnouts[pin].currentPWMVal)
    {

        //Protect thee motors from accidentally silly, potentially damaging values
        if (requiredPosition > 800 && requiredPosition < 2300)
        {
            bool fullyOn = PWMBoards[board].turnouts[pin].currentPWMVal > requiredPosition;
            //Serial.println("Points move required on board "+String(board)+", device " + String(pin) + " to position " + String(requiredPosition) + ".");
            PWMBoards[board].pwm.writeMicroseconds(pin, requiredPosition);
             //Frog polarity
            if (fullyOn == true)
            {
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
            //Serial.println("Points move required but denied on board "+String(board)+", device " + String(pin) + " to position " + String(requiredPosition) + " due to invalid defined value in settings.");
        }
    }
    else {
      
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
        else
        {
            //Serial.println("Points move required but denied on board "+String(board)+", device " + String(pin) + " to position " + String(requiredPosition) + " due to invalid defined value in settings.");
        }
    }
    else {
      //set feedback sensor bit if using
      if (PWMBoards[board].turnouts[pin].hasFeedbackSensor) {
        int inputPin = PWMBoards[board].turnouts[pin].feedbackSensorPin;
        bool pinValue = false;
        if (PWMBoards[board].turnouts[pin].invertFeedbackSensor == true) {
           pinValue = !digitalRead(inputPin);
        }
        else {
          pinValue = digitalRead(inputPin);
        }

        //Serial.println("Setting feedback sensor for CMRI bit "+String(pin+PWMBoards[board].CMRIIndexModifier)+" to "+String(pinValue));
        cmri.set_bit(pin+PWMBoards[board].CMRIIndexModifier, pinValue);
        PWMBoards[board].turnouts[pin].lastFeedbackSensorReading = pinValue;
      }
    }
}

void SetSensorFeedbackReadings() {
    for (int pwmIndex = 0; pwmIndex < NumberOfPWMBoards; pwmIndex++) {
    //for each servo on this PWM board
    for (int i = 0; i < PWMBoards[pwmIndex].numberOfServos; i++)
    {
       if (PWMBoards[pwmIndex].turnouts[i].hasFeedbackSensor) {
          int inputPin = PWMBoards[pwmIndex].turnouts[i].feedbackSensorPin;
          if (PWMBoards[pwmIndex].turnouts[i].invertFeedbackSensor == true) {
              bool pinValue = !digitalRead(inputPin);
              PWMBoards[pwmIndex].turnouts[i].lastFeedbackSensorReading = pinValue;
              //Serial.println("Setting feedback sensor for board "+String(pwmIndex)+", turnout  "+String(i)+" to "+String(pinValue));
          }
          else {
            bool pinValue = digitalRead(inputPin);
            PWMBoards[pwmIndex].turnouts[i].lastFeedbackSensorReading = pinValue;
            //Serial.println("Setting feedback sensor for board "+String(pwmIndex)+", turnout  "+String(i)+" to "+String(pinValue));
          }
       }
               
    }
  }
}

void ProcessInputs() {
   // PROCESS SENSORS
   
     // Do not read 0, 1 or 2
     //Do not read 13
     //Do not read 20 or 21  

     cmri.set_bit(33,digitalRead(34));  //Bit 1 = address 1002 in JMRI

}

void setPWMStateFullyOn(int board, int pin)
{
    //Serial.println("Set relay fully on for board "+String(board)+",  pin "+String(pin));
    PWMBoards[board].pwm.setPWM(pin, 4096, 0);
}

void setPWMStateFullyOff(int board, int pin)
{
    PWMBoards[board].pwm.setPWM(pin, 0, 4096);
    //Serial.println("Set relay fully off for board "+String(board)+",  pin "+String(pin));
}

void InitialiseConfig() {
  //Board 1 - under upper incline junction
  PWMBoards[0].pwm = Adafruit_PWMServoDriver(0x40);
  PWMBoards[0].numberOfServos = 6;
  PWMBoards[0].CMRIIndexModifier = 0;
  PWMBoards[0].turnouts[0] = Turnout(1100,2000,false); //thrown, closed, invertfrog
  PWMBoards[0].turnouts[1] = Turnout(1250,1800,false); //thrown, closed, invertfrog
  PWMBoards[0].turnouts[2] = Turnout(1350,2150,true); //thrown, closed, invertfrog
  PWMBoards[0].turnouts[3] = Turnout(1350,1700,false); //thrown, closed, invertfrog 
  PWMBoards[0].turnouts[4] = Turnout(1250,2100,true); //thrown, closed, invertfrog 
  PWMBoards[0].turnouts[5] = Turnout(1200,1900,true); //thrown, closed, invertfrog  

  //Board 2 - by Arduino, corner 2
  PWMBoards[1].pwm = Adafruit_PWMServoDriver(0x41);
  PWMBoards[1].numberOfServos = 8;
  PWMBoards[1].CMRIIndexModifier = 8; //add this value to the standard identifier to get the correct bit 
  PWMBoards[1].turnouts[0] = Turnout(1000,1800,4,true,false); //thrown, closed, invertfrog
  PWMBoards[1].turnouts[1] = Turnout(1200,1800,3,true,false); //thrown, closed, invertfrog
  PWMBoards[1].turnouts[2] = Turnout(1100,1900,false); //thrown, closed, invertfrog
  PWMBoards[1].turnouts[3] = Turnout(1200,2000,6,true,false); //thrown, closed, invertfrog
  PWMBoards[1].turnouts[4] = Turnout(1250,2150,true); //thrown, closed, invertfrog
  PWMBoards[1].turnouts[5] = Turnout(1250,2100,true); //thrown, closed, invertfrog
  PWMBoards[1].turnouts[6] = Turnout(1400,2250,5,false,false); //thrown, closed, invertfrog
  PWMBoards[1].turnouts[7] = Turnout(1200,1900,true); //thrown, closed, invertfrog

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
