// LED.h
#ifndef LED_h
#define LED_h

#include <Arduino.h>

class LED {
  private:
    

  public:     
    LED(String JMRIID = "");  
    String JMRIId;    
};


#endif
