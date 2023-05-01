// ShiftRegister.h
#ifndef ShiftRegister_h
#define ShiftRegister_h

#include <Arduino.h>
#include "Sensor.h"
#include "LED.h"

class ShiftRegister {
  private:    

  public:     
    Sensor Sensors[8];
    LED LEDs[8];
    char PreviousValues;
};


#endif
