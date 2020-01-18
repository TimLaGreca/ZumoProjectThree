/*
Timer library header file
by Timothy LaGreca and Catherine Neely

A library for Timer objects.
*/

#ifndef Timer_h
#define Timer_h

#include "Arduino.h"
#include <string.h>

class Timer{
  public:
    //constructors
    Timer();
    Timer(long newTimeLimit);
    //getters
    bool getT0();  //T0
    bool getT0_EN();  //T0_EN
    unsigned long getTA0();  //TA0
    String getCurrState();
    long getTimeLimit();
    //setters
    void setT0(bool newT0);
    void setT0_EN(bool newT0_EN);
    void setTA0(unsigned long newTA0);
    void setCurrState(String newState);
    void setTimeLimit(long newTimeLimit);
    //timer functionality
    void runTimer();  //method with while loop that will run the timer (will act like 'loop' in default arduino file)
    void reset();  //method to reset the timer
    
private:
    //values needed for timer
    long timeLimit;  //given as parameter in milliseconds
    unsigned long startTime;  //millis()
    
    String currState;
    bool S1;  //S1 = timer off
    bool S2;  //S2 = timer on
    
    //return values
    bool T0;  //T0
    unsigned long TA0;  //TA0
    bool T0_EN;  //T0_EN
};

#endif
