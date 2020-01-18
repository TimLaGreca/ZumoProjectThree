#include <Wire.h>
#include <Zumo32U4.h>  //documentation url: https://pololu.github.io/zumo-32u4-arduino-library/
#include <Timer.h>

/*
   Block 1
*/

//proxCount Value of Obstruction
float proxMin = 5;


//Buttons
Zumo32U4ButtonA buttonA; //button a
Zumo32U4ButtonB buttonB; //button b
Zumo32U4ButtonC buttonC; //button c

//Alarm
Zumo32U4Buzzer startAlarm;  //moveAlarm
Zumo32U4Buzzer stopAlarm;  //stopAlarm

//Display
Zumo32U4LCD lcd; //display
bool clearDisplay = true;

//Motors
Zumo32U4Motors motors;
float motorSpeed = 400*0.5;
float rMotorSpeed = motorSpeed;
float lMotorSpeed = motorSpeed;

//Encoder
Zumo32U4Encoders encoder;

//Accelerometer
LSM303 compass;
L3G gyro;
char report[120];

//Line Sensor
Zumo32U4LineSensors lineSensors;
#define NUM_SENSORS 3  //the robot has 5 sensors, but we will only use 3
unsigned int lineSensorValues[NUM_SENSORS];  //array of sensor values

//Proxy Sensor
Zumo32U4ProximitySensors proxSensors;
int proxCounts = 0;  //this variable goes from about 2 to about 6. 6 is very close, 2 is far

//Timers
int timer0Time = 3000;
Timer timer0 = new Timer(timer0Time);  //spin for 3 seconds
unsigned long startTime = 0;

//Counters
int turnCount = 0;

//States
String state;
bool S1 = true;  //S1 = stop
bool S2 = false;  //S2 = line folow
bool S3 = false;  //S2 = angle control
bool S4 = false;  //S2 = distance control


//CLOSED LOOP LINE FOLLOW CONTROL
//time
float t = 0;  //microseconds
float dt = 0;  //microseconds
float tPrev = 0;  //microseconds

//error values
float e = 0;  //meters
float de = 0;
float ePrev = 0;

//derivative
float derivative = 0;

//controller output 
float u = 0;  //meters

//gain values
float ksum = 1.0;
float kp = 3.0315e+03;
float kd = 83.992;

float D = 0.0507;  //meters

//CLOSED LOOP ANGLE AND DISTANCE CONTROL
//voltage
float voltage;

//time values
float tDist = 0;  //microseconds
float dtDist = 0;  //microseconds
float tPrevDist = 0;  //microseconds

float tAng = 0;  //microseconds
float dtAng = 0;  //microseconds
float tPrevAng = 0;  //microseconds

//input values
float rAng = 0;  //degrees  //SET FOR TESTING
float rHAng = 0;

float rDist = 0;  //meters  //SET FOR TESTING
float rHDist = 0;

//encoder values
float encoderLDist = 0;  //raw data
float encoderRDist = 0;  //raw data
float distanceDist = 0;  //units of meters

//gyro values
float gyroZ = 0;  //raw data
float angVel = 0;  //units of degees per second

//integral
float integralAng = 0;  //degrees
float integralDist = 0;  //meters

//error values
float eAng = 0;  //degrees
float deAng = 0;
float ePrevAng = 0;

float eDist = 0;  //meters
float deDist = 0;
float ePrevDist = 0;

//derivative
float derivativeAng = 0;
float derivativeDist = 0;

//controller output 
float uAng = 0;  //degrees

float yAng = 0;  //y, units: degrees
float HAng = 1; //H 
float yHAng = 0;

float uDist = 0;  //meters

float yDist = 0;  //y, units of meters
float HDist = 1; //H 
float yHDist = 0;

//angle control gain values
float ksumAng = 1.0;
float kpAng = 0.0492;
float kdAng = 5.7478e-04;
float kiAng =  1.0536;

//distance control gain values
float ksumDist = 1.0;
float kpDist = 72.1246;
float kdDist = 0.9849;
float kiDist =  1.3205e+03;

bool s3Done = false;

//Calibration Function
void calibrateLineSensors()
{
  // To indicate we are in calibration mode, turn on the yellow LED
  // and print "Line cal" on the LCD.
  ledYellow(1);
  lcd.clear();
  lcd.print(F("Line cal"));

  for (uint16_t i = 0; i < 400; i++)
  {
    lcd.gotoXY(0, 1);
    lcd.print(i);
    lineSensors.calibrate();
  }

  ledYellow(0);
  lcd.clear();
}

//prof. Brown's function to read a fat (3.5") line and turn into an "error"
int readLineAAB(unsigned int vals[5]){
  //function returns integer counts representing error from line center
  //values 0, 2, 4 are the actual 3 sensors
  int pos = vals[1]+vals[0]-vals[2]-1000;
  return pos;
}


void resetAngleControl(){  //helper method to reset angle control values between turns
    //reset angle control
    tAng = 0;  //microseconds
    dtAng = 0;  //seconds
    tPrevAng = 0;  //mircoseconds

    //gyro values
    //gyroZ = 0.0;
     angVel = 0;  //units of degees per second

    //error values
    eAng = 0;  //degrees
    deAng = 0;
    ePrevAng = 0;
    
    //derivative
    derivativeAng = 0;

    //controller output 
    uAng = 0;  //degrees
    yAng = 0;  //y, units: degrees
    HAng = 1; //H 
    yHAng = 0;

    //integral
    integralAng = 0;  //degrees
}

void resetDistanceControl(){  //helper method to reset distance control values betwen runs
    //reset distance control
    tDist = 0;  //microseconds
    dtDist = 0;  //seconds
    tPrevDist = 0;  //mircoseconds

    //encoder values
    encoderLDist = 0;  //raw data
    encoderRDist = 0;  //raw data
    distanceDist = 0;  //units of meters

    //error values
    eDist = 0;  //meters
    deDist = 0;
    ePrevDist = 0;

    //derivative
    derivativeDist = 0;

    //controller output 
    uDist = 0;  //meters
    yDist = 0;  //y, units of meters
    HDist = 1; //H 
    yHDist = 0;

    //integral
    integralDist = 0;  //meters
}



void setup() {
  //printing
  Serial.begin(115200);
  Wire.begin();

  //gyro
  if (!gyro.init()) {
    // Failed to detect the gyro.
    ledRed(1);
    while (1) {
      Serial.println(F("Failed to detect gyro."));
      delay(100);
    }
  }
  gyro.enableDefault();


  //initialize 3 line sensors... middle, and two far outside.
  lineSensors.initThreeSensors();
  
  //initialize only the front proximity sensor
  proxSensors.initFrontSensor();

  //initial Timer
  timer0.setTimeLimit(timer0Time);

  // Wait for button A to be pressed and released.
  lcd.clear();
  lcd.print(F("Press A"));
  lcd.gotoXY(0, 1);
  lcd.print(F("to calib"));
  buttonA.waitForButton();

  //this will loop until you move the zumo across the line several times by hand.
  calibrateLineSensors();

  //initial Display
  lcd.clear();
  lcd.print(F("PROJECT"));
  lcd.gotoXY(0, 1);
  lcd.print(F("   3"));
//
}



void loop() {
  //time values
  t = micros();  //microseconds
  dt = (t-tPrev) * (1.0/1000000.0);  //seconds
  tPrev = t;  //mircoseconds

  lineSensors.readCalibrated(lineSensorValues);
  proxSensors.read();
  int error = readLineAAB(lineSensorValues);
  float lineerror_m = error*0.01/700;//this scaling is approximate. YMMV.
  int proxCounts = proxSensors.countsFrontWithRightLeds();

   voltage = readBatteryMillivolts()/1000;
      
  /*
     Block 2
  */
  bool T1 = S1 && buttonC.isPressed();
  bool T2 = S2 && buttonB.isPressed();
  bool T3 = S2 && proxCounts > proxMin; 
  bool T4 = S3 && turnCount >= 4;
  bool T5 = S3 && eAng <= 0.0;
  bool T6 = S4 && eDist <= 0.0;
  bool T7 = S1 && !buttonC.isPressed();
  bool T8 = S2 && proxCounts <= proxMin;
  bool T9 = S3 && eAng > 0.0;
  bool T10 = S4 && eDist > 0.0;

  
  /*
     Block 3
  */
  S1 = T2 || T7;
  S2 = T1 || T8  || T4;
  S3 = T3 || T9  || T6;
  S4 = T5 || T10;

  /*
     Block 4
  */
  if(S1) {  //STOP
      S1 = true;
      S2 = false;
      S3 = false;
      S4 = false;
  
      Serial.print("S1\t" + String(S1));
      Serial.print("\t");
      Serial.print(buttonC.isPressed());
      Serial.print("\t");
      Serial.print(T1);
      Serial.print("\t");
      Serial.println(S2);
      
      motors.setSpeeds(0, 0);
    }
    
  if(S2) {  //LINE FOLLOW CONTROL
      S1 = false;
      S2 = true;
      S3 = false;
      S4 = false;

      resetAngleControl();
      turnCount = 0;
      
      Serial.print("S2\t" + String(S2));
      Serial.print("\t");
      Serial.println(proxCounts);


      //line and proxy values

    
      //error values
      e = lineerror_m;  //updates e value  
      de =  e - ePrev;  //data from function
      ePrev = e;  //data from function
    
      //derivative
      derivative = de/dt;
    
      u = ksum*(kp*e+ kd*(derivative));  //volts

      motorSpeed = 200;

      if(u < -20) {
        motors.setSpeeds(motorSpeed, -motorSpeed);
      } 
      else if (u >= 20) {
        motors.setSpeeds(-motorSpeed, motorSpeed);
      } 
      else {
        motors.setSpeeds(150, 150);
      }
    
//      Serial.print(e);
//      Serial.print("\t");
//      Serial.print(u);
//      Serial.print("\t");
//      Serial.print(motorSpeed);
//      Serial.print("\t");
//      Serial.print(voltage);
      
      //short delay to allow the analog to digital converters to settle.
      delayMicroseconds(1000);

      s3Done = true;
      //motors.setSpeeds(0, 0);

    }
    else if(S3) {  //ANGLE CONTROL
      S1 = false;
      S2 = false;
      S3 = true;
      S4 = false;

      resetDistanceControl();

      if(turnCount == 0 || turnCount == 3){  //if i was going for time, i'd do if statements adjusting motor direction instead
        rAng = 90;
      } else {
        rAng = 270;
      }

      motors.setSpeeds(0, 0);

      Serial.print("S3\t" + String(S3));
      Serial.print("\t");
      Serial.println(eAng);

      //time values
      tAng = micros();  //microseconds
      
      if(s3Done){  //fixes initial dtAng issue at expense of a little accuracy
        s3Done = false;
        dtAng = 0;
      }
      else {
        dtAng = (tAng-tPrevAng) * (1.0/1000000.0);  //seconds
      }

      
      tPrevAng = tAng;  //mircoseconds
  
      //gyro values
      gyro.read();
      gyroZ = gyro.g.z;  //raw data
      angVel = gyroZ*8.99/1000;  //units of degees/second, 8.75/1000;
  
      //integral
      integralAng = integralAng + (angVel*dtAng);  //units of degrees

      //input values
      rHAng = rAng*HAng;
    
      //output values
      yAng = integralAng; //degrees
      yHAng = yAng*HAng;  //degrees
    
      //error values
      eAng = rHAng-yHAng;  //degrees  //updates e value  
      deAng =  abs(eAng - ePrevAng);  //degrees
      ePrevAng = eAng;  //degrees
      
      //derivative
      derivativeAng = deAng/dtAng;
    
      uAng = ksumAng*(kpAng*eAng+ kdAng*(derivativeAng)+ kiAng*integralAng);  //volts
    
      motorSpeed = constrain(uAng*200/voltage, 0, 150);
  
      motors.setSpeeds(-motorSpeed, motorSpeed);
      //Serial.println(String(motorSpeed));
      //Serial.println(String(dtAng) + "\t" + String(angVel) + "\t" + String(integralAng));
      //Serial.println(String(rHAng) + "\t" + String(yHAng) + "\t" + String(eAng) + "\t" + String(deAng) + "\t" + String(dt) + "\t" + String(integralAng) + "\t" + String(derivativeAng) + "\t" + String(uAng) + "\t" + String(motorSpeed));
      encoder.getCountsAndResetLeft();

      if(buttonB.isPressed()){
        eAng = 0;
      }
      
    }
    else if(S4) {  //DISTANCE CONTROL
      S1 = false;
      S2 = false;
      S3 = false;
      S4 = true;

      resetAngleControl();

      if(!s3Done){  //fixes initial dtAng issue at expense of a little accuracy
        s3Done = true;
        turnCount++;
        uDist = 0;
      }

      rDist = 0.15;  //meters

      if(turnCount == 2){  //bc box is not perfect dimensioned 
        rDist = 0.2;
      } else {
        rDist = 0.15;
      }

//      Serial.print("S4\t" + String(S4));
//      Serial.print("\t");
//      Serial.print(eDist);
//      Serial.print("\t");
//      Serial.println(turnCount);



      //time values
      tDist = micros();  //microseconds
      dtDist = (tDist-tPrevDist) * (1.0/1000000.0);  //seconds
      tPrevDist = tDist;  //mircoseconds
        
      //encoder values
      encoderLDist = encoder.getCountsLeft();  //raw data
      distanceDist = encoderLDist/(50*12)*(2*3.14159)* (1.551/2*2.54/100);  //units of meters, (ecnoder data) / (counts per rev) * (2*pi) * rWheel
  
  
      //input values
      rHDist = rDist*HDist;
    
      //integral
      integralDist = integralDist + (distanceDist*dtDist);  //1/s
    
      //output values
      yDist = distanceDist;  //1/s
      yHDist = yDist*HDist;  //1/s
    
      //error values
      eDist = rHDist-yHDist;  //meters  //updates e value  
      deDist =  eDist - ePrevDist;  //meters
      ePrevDist = eDist;  //meters
      
      //derivative
      derivativeDist = deDist/dt;
    
      uDist = ksumDist*(kpDist*eDist+ kdDist*(derivativeDist)+ kiDist*integralDist);  //volts
      uDist = abs(uDist);
    
      motorSpeed = constrain(uDist*200/voltage, 0, 200);
      motors.setSpeeds(motorSpeed, motorSpeed);
  
      //Serial.println(String(encoderLDist));
      //Serial.println(String(motorSpeed));
      //Serial.println(String(rHDist) + "\t" + String(yHDist) + "\t" + String(eDist) + "\t" + String(deDist) + "\t" + String(dtDist) + "\t" + String(integralDist) + "\t" + String(derivativeDist) + "\t" + String(uDist) + "\t" + String(voltage) + "\t" + String(motorSpeed));
      //Serial.println("S5");
      if(eDist < 0){
        delay(100);
      }


//      if(buttonB.isPressed()){
//        eDist = 0;
//        turnCount++;
//        buttonB.waitForRelease();
//      }


    }

            //print the error from the line and the 



}
