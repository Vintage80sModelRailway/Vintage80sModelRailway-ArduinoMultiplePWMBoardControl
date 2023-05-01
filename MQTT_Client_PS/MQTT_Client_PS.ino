#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include "PWMBoard.h"
#include "Turnout.h"
#include "Sensor.h"
#include "ShiftRegister.h"

#define NumberOfPWMBoards 2
#define NumberOfSensors 22
#define NumberOfShiftInRegisters 2
#define NumberOfShiftOutRegisters 2
#define OutputTurnoutThresholdID 1041

// Update these with values suitable for your network.
byte mac[6] = { 0x90, 0xA2, 0xDA, 0x3A, 0xD4, 0x8D };
IPAddress ip(192, 168, 100, 131);
IPAddress myDns(192, 168, 100, 1);
IPAddress gateway(192, 168, 100, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress server(192, 168, 100, 29);

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
EthernetClient ethClient;
PubSubClient client(ethClient);

PWMBoard PWMBoards[NumberOfPWMBoards];
Sensor Sensors[NumberOfSensors];

int numberOfServosMoving = 0;
int servoSpeedMultipluer = 1;

ShiftRegister shiftInRegisters[NumberOfShiftInRegisters];
ShiftRegister shiftOutRegisters[NumberOfShiftOutRegisters];

//Shiftout pins
//Pin connected to DS of 74HC595 - SERIN
int dataPinOut = 44;
//Pin connected to ST_CP of 74HC595 - RCLCK
int latchPinOut = 43;
//Pin connected to SH_CP of 74HC595 - SRCLCK
int clockPinOut = 42;

//ShiftIn pins
int load = 38;
int clockEnablePin = 39;
int dataIn = 40;
int clockIn = 41;

void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message = message + (char)payload[i];
  }
  Serial.println("New message "+String(topic)+" - "+message);
  ProcessIncomingMessage(message, String(topic));
}

void setup()
{
  Serial.begin(19200);
  
  client.setServer(server, 1883);
  client.setCallback(callback);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates
  Ethernet.init(53);
  Ethernet.begin(mac, ip, gateway, subnet);
  // Allow the hardware to sort itself out
  delay(1500);

  if (!client.connected()) {
    reconnect();
  }
  
  InitialiseConfig();
}

void loop()
{
  
  if (!client.connected()) {
    reconnect();
  }

  int numberOfServosMoving = 0;

  for (int i = 0; i < NumberOfSensors; i++) {
    UpdateSensor(i);
  }

  ProcessShiftIn();

  for (int board = 0; board < NumberOfPWMBoards; board++) {
    for (int pin = 0; pin < PWMBoards[board].numberOfTurnouts; pin++) {
      if (PWMBoards[board].turnouts[pin].useSlowMotion) {
        ProcessPointsMoveWithSpeedControl(board, pin);
      }
    }
  }
  client.loop();
}

void ProcessIncomingMessage(String message, String topic) {
  String strTopic = (String)topic;
  int pos = strTopic.lastIndexOf("/");
  int pin = -1;
  int boardId = -1;
  if (pos >= 0 && strTopic.indexOf("turnout") >= 0) {
    int turnoutStatus = 0;
    String justTheID = strTopic.substring(pos + 1);
    int iID = justTheID.toInt();

    if (iID > 0 && iID < OutputTurnoutThresholdID) {
      //THis is a PWM connected turnout
      //Find turnout with this ID
      for (int board = 0; board < NumberOfPWMBoards; board++) {
        for (int pin = 0; pin < PWMBoards[board].numberOfTurnouts; pin++) {
          if (PWMBoards[board].turnouts[pin].jMRIId == justTheID) {
            PWMBoards[board].turnouts[pin].requiredState = message;
            Serial.println(justTheID+" - needs to go "+message);
            if (PWMBoards[board].turnouts[pin].useSlowMotion) {
              numberOfServosMoving++;
              PWMBoards[board].turnouts[pin].requiredState = message;
              if (message == "CLOSED") {
                PWMBoards[board].turnouts[pin].requiredPWMVal = PWMBoards[board].turnouts[pin].closedVal;
                
              } else {
                PWMBoards[board].turnouts[pin].requiredPWMVal = PWMBoards[board].turnouts[pin].thrownVal;
              }
              Serial.println("Required PWM VAL "+String(PWMBoards[board].turnouts[pin].requiredPWMVal));
              Serial.println("Current PWM VAL "+String(PWMBoards[board].turnouts[pin].currentPWMVal));
            }
            else {
              MoveServoFast(board, pin, PWMBoards[board].turnouts[pin], message);
            }
            break;
          }
        }
      }
    }
    else {
      //This is a shift register connected LED
      //get a matching ID;
      Serial.println("Looking in SRs for "+justTheID);
      int foundRegisterIndex = -1;
      int foundPinIndex = -1;
      for (int sr = 0; sr < NumberOfShiftOutRegisters; sr++) {
        for (int pin = 0; pin < 8; pin++) {
          if (shiftOutRegisters[sr].LEDs[pin].JMRIId == justTheID) {
            foundRegisterIndex = sr;
            foundPinIndex = pin;
            Serial.println("Found at register "+String(sr)+" pin "+String(pin));
            break;
          }
        }
        if (foundRegisterIndex >= 0 && foundPinIndex >= 0) break;
      }
      if (foundRegisterIndex >= 0 && foundPinIndex >= 0) {
        int boolVal = 0;
        if (message == "THROWN") boolVal = 1;

        int pv = shiftOutRegisters[foundRegisterIndex].PreviousValues;
        Serial.print("Bitwriting "+String(pv,BIN)+" adding "+String(boolVal)+" to ");
        bitWrite(pv, foundPinIndex, boolVal);
        Serial.println(String(pv,BIN));
        shiftOutRegisters[foundRegisterIndex].PreviousValues = pv;
        ProcessShiftOut();
      }
    }
  }
}

void PublishToMQTT(String topic, String message)  {
  byte topicBuffer[topic.length() + 1];
  byte messageBuffer[message.length() + 1];

  topic.toCharArray(topicBuffer, topic.length() + 1);
  message.toCharArray(messageBuffer, message.length() + 1);

  Serial.println("Publish: " + topic + " - " + message);
  client.publish(topicBuffer, messageBuffer, message.length(), true);
}

void MoveServoFast(int board, int pin, Turnout thisTurnout, String message) {
  Serial.println("Closed " + String(thisTurnout.closedVal) + " Thrown " + String(thisTurnout.thrownVal));
  bool servoMoved = false;
  if (message == "CLOSED") {
    if (thisTurnout.closedVal > 800 && thisTurnout.closedVal < 2200) {
      PWMBoards[board].pwm.writeMicroseconds(pin, thisTurnout.closedVal);
      servoMoved = true;
    }
  }
  else if (message == "THROWN" && thisTurnout.thrownVal > 800 && thisTurnout.thrownVal < 2200) {
    PWMBoards[board].pwm.writeMicroseconds(pin, thisTurnout.thrownVal);
    servoMoved = true;
  }

  if (servoMoved) PWMBoards[board].turnouts[pin - PWMBoards[board].CMRIIndexModifier].currentState = message;

  if (servoMoved && thisTurnout.needsFrogPolarityControl) {
    bool fullyOn = message != "CLOSED";
    if (fullyOn == true)
    {
      Serial.println("Fully on for board " + String(board) + " turnout " + String(pin) + " value " + message);
      if (thisTurnout.invertFrog == false)
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
      Serial.println("Fully off for board " + String(board) + " turnout " + String(pin) + " value " + message);
      if (thisTurnout.invertFrog == false)
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

bool ProcessPointsMoveWithSpeedControl(int board, int pin)
{
  //Serial.println("Number of servos moving "+String(numberOfServosMoving));
  bool moveIsComplete = false;
  int calculatedStepSize = numberOfServosMoving;
  unsigned long currentMillis = millis();
  int requiredPosition = PWMBoards[board].turnouts[pin].requiredPWMVal;
  for (int i = 0; i < servoSpeedMultipluer; i++) {
    calculatedStepSize = calculatedStepSize * numberOfServosMoving;
  }

  //    int totaldStepSize =  PWMBoards[board].turnouts[pin].stepSize + calculatedStepSize;
  int totaldStepSize =  PWMBoards[board].turnouts[pin].stepSize;
  if ((requiredPosition != PWMBoards[board].turnouts[pin].currentPWMVal && currentMillis - PWMBoards[board].turnouts[pin].previousMillis >= PWMBoards[board].turnouts[pin].delayTime))
  {
    //Serial.println(String(PWMBoards[board].turnouts[pin].currentPWMVal && currentMillis - PWMBoards[board].turnouts[pin].previousMillis)+" delay time "+String(PWMBoards[board].turnouts[pin].delayTime));
    PWMBoards[board].turnouts[pin].previousMillis = currentMillis;

    //Protect thee motors from accidentally silly, potentially damaging values
    if (requiredPosition > 800 && requiredPosition < 2351)
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
          moveIsComplete = true;
        }
      }
    }
    else {
      //Serial.println(String(PWMBoards[board].turnouts[pin].currentPWMVal && currentMillis - PWMBoards[board].turnouts[pin].previousMillis)+" delay out of  time "+String(PWMBoards[board].turnouts[pin].delayTime));
    }

    //Set frog polarity by bit value not servo position as some turnouts are inverted
    if (moveIsComplete) {
      PWMBoards[board].turnouts[pin].currentState = PWMBoards[board].turnouts[pin].requiredState;
      PWMBoards[board].turnouts[pin].currentPWMVal = PWMBoards[board].turnouts[pin].requiredPWMVal;
      numberOfServosMoving--;
      bool fullyOn = PWMBoards[board].turnouts[pin].currentState != "CLOSED";
      if (fullyOn == true)
      {
        Serial.println("Fully on in slow for board " + String(board) + " turnout " + String(pin) + " message " + PWMBoards[board].turnouts[pin].currentState);
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
        Serial.println("Fully off in slow for board " + String(board) + " turnout " + String(pin) + " message " + PWMBoards[board].turnouts[pin].currentState);
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

void setPWMStateFullyOn(int board, int pin)
{
  if (pin < 8) return;
  PWMBoards[board].pwm.setPWM(pin, 4096, 0);
}

void setPWMStateFullyOff(int board, int pin)
{
  if (pin < 8) return;
  PWMBoards[board].pwm.setPWM(pin, 0, 4096);
}

void UpdateSensor(int i) {
  bool hasChanged = Sensors[i].UpdateSensor();
  if (hasChanged) {
    String publishMessage = Sensors[i].State;
    String topic = Sensors[i].GetSensorPublishTopic();

    PublishToMQTT(topic, publishMessage);

  }
}

void ProcessShiftIn() {
  digitalWrite(load, LOW);
  digitalWrite(load, HIGH);

  // Get data from 74HC165
  digitalWrite(clockIn, HIGH);
  digitalWrite(clockEnablePin, LOW);
  for (int i = 0; i < NumberOfShiftInRegisters; i++) {
    byte data = shiftIn(dataIn, clockIn, MSBFIRST);
    if (data != shiftInRegisters[i].PreviousValues) {
      UpdateShiftInputs(data, shiftInRegisters[i].PreviousValues, i);
      shiftInRegisters[i].PreviousValues = data;
    }
  }
  digitalWrite(clockEnablePin, HIGH);
  digitalWrite(load, LOW);
}

void UpdateShiftInputs(int data, int previousData, int shiftRegisterIndex) {
  int bits[8];
  int previousBits[8];
  for (int i = 0; i < 8; i++) {
    //bits[i] = (bool)((data >> (i % 8)) & 0x01);
    bits [i] = bitRead(data, i);
    previousBits[i] = bitRead(previousData, i);
    bool hasChanged = shiftInRegisters[shiftRegisterIndex].Sensors[i].UpdateShiftRegisterSensor(bits[i]);
    if (hasChanged) {
      String publishMessage = shiftInRegisters[shiftRegisterIndex].Sensors[i].State;
      String topic = shiftInRegisters[shiftRegisterIndex].Sensors[i].GetSensorPublishTopic();
      PublishToMQTT(topic, publishMessage);
    }
  }
}

void ProcessShiftOut() {
  digitalWrite(latchPinOut, LOW);

  for (int i = NumberOfShiftOutRegisters-1; i >= 0; i--) {
    shiftOut(dataPinOut, clockPinOut, MSBFIRST, shiftOutRegisters[i].PreviousValues);
    Serial.println("Shift out "+String(i));
  }

  digitalWrite(latchPinOut, HIGH);
}

void reconnect() {
  // Loop until we're reconnected

  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("arduinoClient")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      //client.publish("outTopic","hello world");
      // ... and resubscribe
      //client.subscribe("#");
      client.subscribe("track/turnout/1001");
      client.subscribe("track/turnout/1002");
      client.subscribe("track/turnout/1003");
      client.subscribe("track/turnout/1004");
      client.subscribe("track/turnout/1005");
      client.subscribe("track/turnout/1006");

      client.subscribe("track/turnout/1009");
      client.subscribe("track/turnout/1010");
      client.subscribe("track/turnout/1011");
      client.subscribe("track/turnout/1012");
      client.subscribe("track/turnout/1013");
      client.subscribe("track/turnout/1014");
      client.subscribe("track/turnout/1015");
      client.subscribe("track/turnout/1016");


      client.subscribe("track/turnout/1041");
      client.subscribe("track/turnout/1042");
      client.subscribe("track/turnout/1043");
      client.subscribe("track/turnout/1044");
      client.subscribe("track/turnout/1045");
      client.subscribe("track/turnout/1046");

      client.subscribe("track/turnout/1049");
      client.subscribe("track/turnout/1050");
      client.subscribe("track/turnout/1051");
      client.subscribe("track/turnout/1052");
      client.subscribe("track/turnout/1053");
      client.subscribe("track/turnout/1054");

    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void InitialiseConfig() {
  //Board 1 - under upper incline junction
  PWMBoards[0].pwm = Adafruit_PWMServoDriver(0x40);
  PWMBoards[0].numberOfTurnouts = 6;
  PWMBoards[0].CMRIIndexModifier = 0;
  PWMBoards[0].turnouts[0] = Turnout("1001", 2000, 1100, 1, 5, 12, false, true); //thrown, closed, feedback pin number, invertfrog
  PWMBoards[0].turnouts[1] = Turnout("1002", 1900, 1150, 1, 5, 14, true, true);
  PWMBoards[0].turnouts[2] = Turnout("1003", 1350, 2150, 1, 5, true); //thrown, closed, invertfrog
  PWMBoards[0].turnouts[3] = Turnout("1004", 1250, 2100, 1, 5, false);
  PWMBoards[0].turnouts[4] = Turnout("1005", 2100, 1250, 1, 5, 11, false, false);
  PWMBoards[0].turnouts[5] = Turnout("1006", 1200, 1900, 1, 5, true);


  //Board 2 - by Arduino, corner 2
  PWMBoards[1].pwm = Adafruit_PWMServoDriver(0x41);
  PWMBoards[1].numberOfTurnouts = 8;
  PWMBoards[1].CMRIIndexModifier = 8; //add this value to the standard identifier to get the correct bit
  PWMBoards[1].turnouts[0] = Turnout("1009", 1000, 2100, 1, 5, 4, true, true);
  PWMBoards[1].turnouts[1] = Turnout("1010", 1200, 1800, 1, 5, 3, true, true);
  PWMBoards[1].turnouts[2] = Turnout("1011", 1900, 1100, 1, 5, 7, true, false);
  PWMBoards[1].turnouts[3] = Turnout("1012", 2000, 1200, 1, 5, 6, false, false);
  PWMBoards[1].turnouts[4] = Turnout("1013", 1250, 2150, 1, 5, 8, true, true);
  PWMBoards[1].turnouts[5] = Turnout("1014", 1250, 2100, 1, 5, 9, true, false);
  PWMBoards[1].turnouts[6] = Turnout("1015", 1400, 2250, 1, 5, 5, false, true);
  PWMBoards[1].turnouts[7] = Turnout("1016", 1200, 1900, 1, 5, 10, true, true);

  PWMBoards[0].pwm.begin();
  PWMBoards[0].pwm.setPWMFreq(50);  // This is the maximum PWM frequency
  PWMBoards[0].pwm.setOscillatorFrequency(25000000);

  PWMBoards[1].pwm.begin();
  PWMBoards[1].pwm.setPWMFreq(50);  // This is the maximum PWM frequency
  PWMBoards[1].pwm.setOscillatorFrequency(25000000);

  

  //Shift in registers
  shiftInRegisters[0].Sensors[0] = Sensor("IR C2AC1IR1", -1, "1041");
  shiftInRegisters[0].Sensors[1] = Sensor("IR C2AC2IR1", -1, "1042");
  shiftInRegisters[0].Sensors[2] = Sensor("IR C2AC3IR1", -1, "1043");
  shiftInRegisters[0].Sensors[3] = Sensor("IR C2AC4IR1", -1, "1044");
  shiftInRegisters[0].Sensors[4] = Sensor("zzIR C2C1IR3 (flashing)", -1, "1045");
  shiftInRegisters[0].Sensors[5] = Sensor("IR CW Yard 5 Entry", -1, "1046");
  shiftInRegisters[0].Sensors[6] = Sensor("IR C2C1IR3b", -1, "1047");
  shiftInRegisters[0].Sensors[7] = Sensor("IR C2C2IR3b", -1, "1048");

  shiftInRegisters[1].Sensors[0] = Sensor("IR C2AC1IR2", -1, "1049");
  shiftInRegisters[1].Sensors[1] = Sensor("IR C2AC3IR2", -1, "1050");
  shiftInRegisters[1].Sensors[2] = Sensor("IR C2AC2R2", -1, "1051");
  shiftInRegisters[1].Sensors[3] = Sensor("IR C2AC4IR2", -1, "1052");
  shiftInRegisters[1].Sensors[4] = Sensor("zzIR C2C2IR3 flashing", -1, "1053");
  shiftInRegisters[1].Sensors[5] = Sensor("IR C2C5IR3", -1, "1054");
  shiftInRegisters[1].Sensors[6] = Sensor("IIR C2C4IR3", -1, "1055");
  shiftInRegisters[1].Sensors[7] = Sensor("IR C2C3IR3", -1, "1056");

  

  //Shift out registers
  shiftOutRegisters[0].LEDs[0] = LED("1041");
  shiftOutRegisters[0].LEDs[1] = LED("1042");
  shiftOutRegisters[0].LEDs[2] = LED("1043");
  shiftOutRegisters[0].LEDs[3] = LED("1044");
  shiftOutRegisters[0].LEDs[4] = LED("1045");
  shiftOutRegisters[0].LEDs[5] = LED("1046");

  shiftOutRegisters[1].LEDs[0] = LED("1049");
  shiftOutRegisters[1].LEDs[1] = LED("1050");
  shiftOutRegisters[1].LEDs[2] = LED("1051");
  shiftOutRegisters[1].LEDs[3] = LED("1052");
  shiftOutRegisters[1].LEDs[4] = LED("1053");
  shiftOutRegisters[1].LEDs[5] = LED("1054");

  for (int sr = 0; sr < NumberOfShiftOutRegisters; sr++) {    
    shiftOutRegisters[sr].PreviousValues = 0;   
  }


  //Sensors - Name, Pin, JMRIId, IsInverted = false, Pinmode = INPUT, Lastknownvalue = 0
    Sensors[0] = Sensor("MS Incline Top 1", 12, "1001", true,INPUT_PULLUP);
    Sensors[1] = Sensor("MS Incline Top 2", 14, "1002", true,INPUT_PULLUP);
    Sensors[2] = Sensor("MS C2 AC Y3",11, "1005", false,INPUT_PULLUP);
    Sensors[3] = Sensor("MS C2 Curved 1", 4, "1009", true,INPUT_PULLUP);
    Sensors[4] = Sensor("MS C2 Curved 2", 3, "1010", true,INPUT_PULLUP);
    Sensors[5] = Sensor("MS C2 Yard Outer 1", 7, "1011", false,INPUT_PULLUP);
    Sensors[6] = Sensor("MS CY4", 6, "1012", true,INPUT_PULLUP);
    Sensors[7] = Sensor("MS CY5", 8, "1013", true,INPUT_PULLUP);
    Sensors[8] = Sensor("MS CY6", 9, "1014", false,INPUT_PULLUP);
    Sensors[9] = Sensor("MS C2 AC Y1", 5, "1015", true,INPUT_PULLUP);
    Sensors[10] = Sensor("MS C2 AC Y2", 10, "1016", true,INPUT_PULLUP);
    Sensors[11] = Sensor("CD Pi end CW", 32, "1017", false,INPUT_PULLUP);
    Sensors[12] = Sensor("CD Pi end AC", 33, "1018", false,INPUT_PULLUP);
    Sensors[13] = Sensor("CD Pi end incline", 28, "1019", false,INPUT_PULLUP);
    Sensors[14] = Sensor("IR CW Yard Entry", 29, "1020", true);
    Sensors[15] = Sensor("IR CW Yard entry lower", 27, "1021", true);
    Sensors[16] = Sensor("IR AC Yard Exit", 26, "1022", true);
    Sensors[17] = Sensor("IR AC Yard Bypass stopping sensor", 36, "1023", true);
    Sensors[18] = Sensor("CD AC Yard bypass Pi End", 24, "1024", false,INPUT_PULLUP);
    Sensors[19] = Sensor("CD CW Yard bypass Pi end", 25, "1025", false,INPUT_PULLUP);
    Sensors[20] = Sensor("CD Incline Top", 22, "1026", false,INPUT_PULLUP);
    Sensors[21] = Sensor("CD corner 2 unused 1", 23, "1027", false,INPUT_PULLUP);



  for (int i = 0; i < NumberOfSensors; i++) {
    Sensors[i].SetPinMode();
    UpdateSensor(i);

    String publishMessage = Sensors[i].State;
    String topic = Sensors[i].GetSensorPublishTopic();
  }

  // Setup 74HC165 connections
  pinMode(load, OUTPUT);
  pinMode(clockEnablePin, OUTPUT);
  pinMode(clockIn, OUTPUT);
  pinMode(dataIn, INPUT);
  pinMode(latchPinOut, OUTPUT);
  pinMode(clockPinOut, OUTPUT);
  pinMode(dataPinOut, OUTPUT);

  Serial.println("Setup complete");
}
