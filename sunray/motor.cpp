// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "motor.h"
#include "config.h"
#include "robot.h"
#include "Arduino.h"

volatile uint16_t odoTicksLeft = 0;
volatile uint16_t odoTicksRight = 0;



// odometry signal change interrupt
void OdometryLeftInt(){			
  odoTicksLeft++;
}

void OdometryRightInt(){			
  odoTicksRight++;
}


void Motor::begin() {
  // left wheel motor
  pinMode(pinMotorEnable, OUTPUT);
  digitalWrite(pinMotorEnable, HIGH);
  pinMode(pinMotorLeftPWM, OUTPUT);
  pinMode(pinMotorLeftDir, OUTPUT);
  pinMode(pinMotorLeftSense, INPUT);
  pinMode(pinMotorLeftFault, INPUT);

  // right wheel motor
  pinMode(pinMotorRightPWM, OUTPUT);
  pinMode(pinMotorRightDir, OUTPUT);
  pinMode(pinMotorRightSense, INPUT);
  pinMode(pinMotorRightFault, INPUT);

  // mower motor
  pinMode(pinMotorMowDir, OUTPUT);
  pinMode(pinMotorMowPWM, OUTPUT);
  pinMode(pinMotorMowSense, INPUT);
  pinMode(pinMotorMowRpm, INPUT);
  pinMode(pinMotorMowEnable, OUTPUT);
  digitalWrite(pinMotorMowEnable, HIGH);
  pinMode(pinMotorMowFault, INPUT);

  // odometry
  pinMode(pinOdometryLeft, INPUT_PULLUP);
  pinMode(pinOdometryLeft2, INPUT_PULLUP);
  pinMode(pinOdometryRight, INPUT_PULLUP);
  pinMode(pinOdometryRight2, INPUT_PULLUP);
	
  pinMode(pinMotorMowSense, INPUT);
  pinMode(pinMotorLeftSense, INPUT);
  pinMode(pinMotorRightSense, INPUT);  
 
  // enable interrupts
  attachInterrupt(pinOdometryLeft, OdometryLeftInt, RISING);  
  attachInterrupt(pinOdometryRight, OdometryRightInt, RISING);  
	
	pinMan.setDebounce(pinOdometryLeft, 100);  // reject spikes shorter than usecs on pin
	pinMan.setDebounce(pinOdometryRight, 100);  // reject spikes shorter than usecs on pin	
	
	pwmMax = 255;
  pwmMaxMow = 255;

  //ticksPerRevolution = 1060/2;
  ticksPerRevolution = TICKS_PER_REVOLUTION;
	wheelBaseCm = WHEEL_BASE_CM;    // wheel-to-wheel distance (cm) 36
  wheelDiameter = WHEEL_DIAMETER; // wheel diameter (mm)
  ticksPerCm         = ((float)ticksPerRevolution) / (((float)wheelDiameter)/10.0) / 3.1415;    // computes encoder ticks per cm (do not change)  

  motorLeftPID.Kp       = 2.0;  
  motorLeftPID.Ki       = 0.03; 
  motorLeftPID.Kd       = 0.03; 
  motorRightPID.Kp       = motorLeftPID.Kp;
  motorRightPID.Ki       = motorLeftPID.Ki;
  motorRightPID.Kd       = motorLeftPID.Kd;		 

  motorLeftSwapDir = false;
  motorRightSwapDir = false;
  motorError = false;

  linearSpeedSet = 0;
  angularSpeedSet = 0;
  motorLeftRpmSet = 0;
  motorRightRpmSet = 0;
  motorMowPWMSet = 0;

  lastControlTime = 0;
  motorLeftTicks =0;  
  motorRightTicks =0;
  motorLeftTicksZero=0;
  motorRightTicksZero=0;
  motorLeftPWMCurr =0;  
  motorRightPWMCurr=0; 
  motorMowPWMCurr = 0;
  motorLeftRpmCurr=0;
  motorRightRpmCurr=0;
  setLinearAngularSpeedTimeoutActive = false;  
  setLinearAngularSpeedTimeout = 0;
}


// MC33926 motor driver
// Check http://forum.pololu.com/viewtopic.php?f=15&t=5272#p25031 for explanations.
//(8-bit PWM=255, 10-bit PWM=1023)
// IN1 PinPWM         IN2 PinDir
// PWM                L     Forward
// nPWM               H     Reverse
void Motor::setMC33926(int pinDir, int pinPWM, int speed) {
  //DEBUGLN(speed);
  if (speed < 0) {
    digitalWrite(pinDir, HIGH) ;
    pinMan.analogWrite(pinPWM, 255 - ((byte)abs(speed)));
  } else {
    digitalWrite(pinDir, LOW) ;
    pinMan.analogWrite(pinPWM, ((byte)speed));
  }
}


void Motor::speedPWM ( MotorSelect motor, int speedPWM )
{
  if (motor == MOTOR_MOW) {
    if (speedPWM > pwmMaxMow) speedPWM = pwmMaxMow;
    else if (speedPWM < -pwmMaxMow) speedPWM = -pwmMaxMow;
  } else {
    if (speedPWM > pwmMax) speedPWM = pwmMax;
    else if (speedPWM < -pwmMax) speedPWM = -pwmMax;
  }
  switch (motor) {
    case MOTOR_LEFT:
      if (motorLeftSwapDir) speedPWM *= -1;
      setMC33926(pinMotorLeftDir, pinMotorLeftPWM, speedPWM);
      break;
    case MOTOR_RIGHT:
      if (motorRightSwapDir) speedPWM *= -1;
      setMC33926(pinMotorRightDir, pinMotorRightPWM, speedPWM);
      break;
    case MOTOR_MOW:        
      setMC33926(pinMotorMowDir, pinMotorMowPWM, speedPWM);
      break;
  }
}

// linear: m/s
// angular: rad/s
void Motor::setLinearAngularSpeed(float linear, float angular){
   linearSpeedSet = linear;
   angularSpeedSet = angular;
   setLinearAngularSpeedTimeout = millis() + 1000;
   setLinearAngularSpeedTimeoutActive = true;
   float rspeed = linear + angular * (wheelBaseCm /100.0 /2);          
   float lspeed = linear * 2.0 - rspeed;          
   motorRightRpmSet =  rspeed / (PI*(wheelDiameter/1000.0)) * 60.0;
   motorLeftRpmSet = lspeed / (PI*(wheelDiameter/1000.0)) * 60.0;   
   /*CONSOLE.print("setLinearAngularSpeed ");
   CONSOLE.print(linear);
   CONSOLE.print(",");
   CONSOLE.print(rspeed);
   CONSOLE.print(",");
   CONSOLE.println(motorRightRpmSet);   */
}

void Motor::setMowState(bool switchOn){
  if (switchOn){
    motorMowPWMSet = pwmMaxMow;  
  } else {
    motorMowPWMSet = 0;  
    motorMowPWMCurr = 0;
  }  
}

void Motor::stopImmediately(bool includeMowerMotor){
  //motorRightRpmSet = 0;
  //motorLeftRpmSet = 0;  
  //linearSpeedSet = 0;
  //angularSpeedSet = 0;  
  motorLeftPWMCurr = 0;
  motorRightPWMCurr = 0; 
  speedPWM(MOTOR_LEFT, motorLeftPWMCurr);
  speedPWM(MOTOR_RIGHT, motorRightPWMCurr);  
  if (includeMowerMotor) {
    //motorMowPWMSet = 0;
    motorMowPWMCurr = 0;    
    speedPWM(MOTOR_MOW, motorMowPWMCurr);  
  }
}

void Motor::stopControl(){
  motorRightRpmSet = 0;
  motorLeftRpmSet = 0;
  motorMowPWMSet = 0;
}

void Motor::run() {
  if (millis() < lastControlTime + 50) return;
  if (setLinearAngularSpeedTimeoutActive){
    if (millis() > setLinearAngularSpeedTimeout){
      setLinearAngularSpeedTimeoutActive = false;
      motorLeftRpmSet = 0;
      motorRightRpmSet = 0;
    }
  }
  
  int ticksLeft = odoTicksLeft;
  odoTicksLeft = 0;
  int ticksRight = odoTicksRight;
  odoTicksRight = 0;

  if (motorLeftPWMCurr < 0) ticksLeft *= -1;
  if (motorRightPWMCurr < 0) ticksRight *= -1;
  motorLeftTicks += ticksLeft;
  motorRightTicks += ticksRight;

  unsigned long currTime = millis();
  float deltaControlTimeSec =  ((float)(currTime - lastControlTime)) / 1000.0;
  lastControlTime = currTime;

  // calculate speed via tick count
  // 2000 ticksPerRevolution: @ 30 rpm  => 0.5 rps => 1000 ticksPerSec
  // 20 ticksPerRevolution: @ 30 rpm => 0.5 rps => 10 ticksPerSec
  motorLeftRpmCurr = 60.0 * ( ((float)ticksLeft) / ((float)ticksPerRevolution) ) / deltaControlTimeSec;
  motorRightRpmCurr = 60.0 * ( ((float)ticksRight) / ((float)ticksPerRevolution) ) / deltaControlTimeSec;

  if (ticksLeft == 0) {
    motorLeftTicksZero++;
    if (motorLeftTicksZero > 2) motorLeftRpmCurr = 0;
  } else motorLeftTicksZero = 0;

  if (ticksRight == 0) {
    motorRightTicksZero++;
    if (motorRightTicksZero > 2) motorRightRpmCurr = 0;
  } else motorRightTicksZero = 0;

  
  control();
  checkFault();
  if (motorError){
    resetFault();
  }
}

void Motor::control(){  
  /*CONSOLE.print("rpm set=");
  CONSOLE.print(motorLeftRpmSet);
  CONSOLE.print(",");
  CONSOLE.print(motorRightRpmSet);
  CONSOLE.print("   curr=");
  CONSOLE.print(motorLeftRpmCurr);
  CONSOLE.print(",");
  CONSOLE.println(motorRightRpmCurr);*/
  motorLeftPID.x = motorLeftRpmCurr;
  motorLeftPID.w  = motorLeftRpmSet;
  motorLeftPID.y_min = -pwmMax;
  motorLeftPID.y_max = pwmMax;
  motorLeftPID.max_output = pwmMax;
  motorLeftPID.compute();
  motorLeftPWMCurr = motorLeftPWMCurr + motorLeftPID.y;
  if (motorLeftRpmSet >= 0) motorLeftPWMCurr = min( max(0, (int)motorLeftPWMCurr), pwmMax); // 0.. pwmMax
  if (motorLeftRpmSet < 0) motorLeftPWMCurr = max(-pwmMax, min(0, (int)motorLeftPWMCurr));  // -pwmMax..0
  
  motorRightPID.x = motorRightRpmCurr;
  motorRightPID.w = motorRightRpmSet;
  motorRightPID.y_min = -pwmMax;
  motorRightPID.y_max = pwmMax;
  motorRightPID.max_output = pwmMax;
  motorRightPID.compute();
  motorRightPWMCurr = motorRightPWMCurr + motorRightPID.y;
  if (motorRightRpmSet >= 0) motorRightPWMCurr = min( max(0, (int)motorRightPWMCurr), pwmMax);  // 0.. pwmMax
  if (motorRightRpmSet < 0) motorRightPWMCurr = max(-pwmMax, min(0, (int)motorRightPWMCurr));   // -pwmMax..0  

  if ((abs(motorLeftRpmSet) < 0.01) && (motorLeftPWMCurr < 30)) motorLeftPWMCurr = 0;
  if ((abs(motorRightRpmSet) < 0.01) && (motorRightPWMCurr < 30)) motorRightPWMCurr = 0;
  
  motorMowPWMCurr = 0.99 * motorMowPWMCurr + 0.01 * motorMowPWMSet;
  
  speedPWM(MOTOR_LEFT, motorLeftPWMCurr);
  speedPWM(MOTOR_RIGHT, motorRightPWMCurr);  
  speedPWM(MOTOR_MOW, motorMowPWMCurr);  
}


void Motor::resetFault() {
  motorError = false;
  if (digitalRead(pinMotorLeftFault) == LOW) {
    digitalWrite(pinMotorEnable, LOW);
    digitalWrite(pinMotorEnable, HIGH);
    DEBUGLN(F("Reset motor left fault"));
  }
  if  (digitalRead(pinMotorRightFault) == LOW) {
    digitalWrite(pinMotorEnable, LOW);
    digitalWrite(pinMotorEnable, HIGH);
    DEBUGLN(F("Reset motor right fault"));
  }
  if (digitalRead(pinMotorMowFault) == LOW) {
    digitalWrite(pinMotorMowEnable, LOW);
    digitalWrite(pinMotorMowEnable, HIGH);
    DEBUGLN(F("Reset motor mow fault"));
  }
}


// handle motor errors
void Motor::checkFault() {
  if (motorError) return;
  if (digitalRead(pinMotorLeftFault) == LOW) {
    DEBUGLN(F("Error: motor left fault"));
    stopControl();
    motorError = true;    
  }
  if  (digitalRead(pinMotorRightFault) == LOW) {
    DEBUGLN(F("Error: motor right fault")); 
    stopControl();
    motorError = true;    
  }
  if (digitalRead(pinMotorMowFault) == LOW) {
    DEBUGLN(F("Error: motor mow fault"));
    stopControl();
    motorError = true;        
  }
}

void Motor::test(){
  DEBUGLN(F("motor test - 10 revolutions"));
  odoTicksLeft = 0;  
  odoTicksRight = 0;  
  unsigned long nextInfoTime = 0;
  int seconds = 0;
  speedPWM(MOTOR_LEFT, 100);
  speedPWM(MOTOR_RIGHT, 100);       
  bool slowdown = true;
  uint16_t stopTicks = ticksPerRevolution * 10;
  while (odoTicksLeft < stopTicks){
    if ((slowdown) && (odoTicksLeft + 50 > stopTicks)){
      speedPWM(MOTOR_LEFT, 20);
      speedPWM(MOTOR_RIGHT, 20);       
      slowdown = false;
    }    
    if (millis() > nextInfoTime){
      nextInfoTime = millis() + 1000;      
      CONSOLE.print("t=");
      CONSOLE.print(seconds);
      CONSOLE.print("  ticks=");
      CONSOLE.println(odoTicksLeft);      
      seconds++;
    }    
    delay(1);
  }
  speedPWM(MOTOR_LEFT, 0);
  speedPWM(MOTOR_RIGHT, 0);  
  DEBUGLN(F("motor test done"));
}

