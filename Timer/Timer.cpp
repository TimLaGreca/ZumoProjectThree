/*
 Timer Library C++ File
 by Timothy LaGreca and Catherine Neely
 
 A library for Timer objects.
 */

#include "Timer.h"

/*
 default constructor
 */
Timer::Timer(){}


/*
 constructor
 does some of Block 1's job
 the rest of Block 1 is in the runTimer() function
 */
Timer::Timer(long newTimeLimit)
{
    timeLimit = newTimeLimit;
    startTime = 0;
    
    currState = "S1";
    S1 = true;
    S2 = false;
    
    T0 = false;
    T0_EN = false;
    TA0 = 0;
}


/*
method to return timeLimitReached (T0)
*/
bool Timer::getT0(){
    return T0;
}

/*
method to return timeEngaged (T0_EN)
*/
bool Timer::getT0_EN(){
    return T0_EN;
}

/*
 method to retrun timeElapsed (TA0)
 */
unsigned long Timer::getTA0(){
    return TA0;
}

/*
 method to retrun state (S1 or S2)
 */
String Timer::getCurrState(){
    return currState;
}

/*
 method to retrun timeLimit
 */
long Timer::getTimeLimit(){
    return timeLimit;
}


/*
method to set T0
*/
void Timer::setT0(bool newT0){
    T0 = newT0;
}

/*
method to set T0_EN
*/
void Timer::setT0_EN(bool newT0_EN){
    T0_EN = newT0_EN;
}

/*
method to set TA0
*/
void Timer::setTA0(unsigned long newTA0){
    TA0 = newTA0;
}

/*
method to set state
*/
void Timer::setCurrState(String newState){
    currState = newState;
}
/*
method to set timeLimit
*/
void Timer::setTimeLimit(long newTimeLimit){
    timeLimit = newTimeLimit;
}

/*
 a method to run the timer
 the while loop acts like 'void loop' in .ino files
 */
void Timer::runTimer(){
    /*
     Block 1
     */
    if(!T0_EN){  //only sets start time if timer is not running
        startTime = millis();
        T0_EN = true;
    }
    
    
    TA0 = millis() - startTime;  //update time ellapsed

//    if(TA0 >= timeLimit){  //if timer passes time limit, should be time limit but just made 5000 for now
//        T0 = true;  //set T0 to true
//        //T0_EN = false;  //disenable timer
//    }

    /*
     Block 2
     */
    bool Ta = S1 && T0_EN;
    bool Tb = S2 && TA0 >= timeLimit;
    bool Tc = S2 && TA0 < timeLimit;
    bool Td = S1 && !T0_EN;

    /*
     Block 3
     */
    S1 = Tb || Td;
    S2 = Ta || Tc;

    /*
     Block 4
     */
    if(S1){  //timer off
        currState = "S1";
        T0 = true;
        reset();
    }
    else if(S2){  //timer on
        currState = "S2";
        T0 = false;
    }

}

/*
method to reset the timer
*/
void Timer::reset(){
    T0_EN = false;
    TA0 = 0;
}
