# Vintage80sModelRailway-ArduinoMultiplePWMBoardControl
Controlling multiple PCA9685 PWM driver boards, each with multiple servo controlled point motors and relay based frog polarity switchers, from one Arduino

This repo contains the sketches that run on the 3 Arduinos that run on my model railway.

Node 1 has multiple PWM boards, the others just have one.

They respond to CMRI messages and set turnouts and relays (for electrofrog polarity control), both of which are connected to the PCA9685 board, the first 8 ports

I've adapted them recently to support multiple PCA9685 boards, by adding classes for turnouts. Sinstantiators for the turnout classes are

    Turnout(); - don't use this one, it's useless
    Turnout(int ThrownVal, int ClosedVal); - if you simply just want to move servos
    Turnout(int ThrownVal, int ClosedVal, int FeedbackSensorPin); - feedbackSensorPin is the pin on the arduino that a feedback sensor is attached to. If a feedback sensor pin number is provided, the sketch will send the value from the pin back to CMRI on the same bit value that its corresponding turnout is on.
    
    Turnout(int ThrownVal, int ClosedVal, bool InvertFrog); - if invertFrog value is passed, the sketch will assume that relays are being used for electrofrog polarity and will try to set a relay at PCA9685 board servo pin number + 8.
    Turnout(int ThrownVal, int ClosedVal, int FeedbackSensorPin, bool InvertFrog); - including feedback sensor and relay frog polarity
    
    Turnout(int ThrownVal, int ClosedVal, int StepSize, int DelayTime); - attempts to use slow motion movement. I don't advise using this without testing, it's a long time since I tested it. I don't think it works very well, moving multiple motors at the same time (ie on a route) slows them down considerably and they end up too jerky
    
    Turnout(int ThrownVal, int ClosedVal, int StepSize, int DelayTime, int FeedbackSensorPin);
    
    Turnout(int ThrownVal, int ClosedVal, int StepSize, int DelayTime, bool InvertFrog);
    Turnout(int ThrownVal, int ClosedVal, int StepSize, int DelayTime, int FeedbackSensorPin, bool InvertFrog);
    
    Disclaimer - these sketches are written very much to suit my needs and would be very unlikely to work on another layout. They're just here for reference really, to share how I went about doing it.
