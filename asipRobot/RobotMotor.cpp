/* RobotMotor.cpp
 * Supports motors using Toshiba H-Bridge motor driver ICs.
 * such as Mirto v2 board or Hub-ee 
 * This version has PID speed control
 * Michael Margolis July 2016
 */


#include "RobotMotor.h" 
#include "RobotDescription.h"  // for wheel circumference
#include "utility/debug.h"

RobotMotor::RobotMotor(const byte pins[], Encoder *encoder)
{
   this->pins = pins;  
   this->encoder = encoder;
   PID = new MotorPID(ENCODER_TICKS_PER_WHEEL_REV,MAX_PWM, MAX_PWM_DELTA); 
}

void RobotMotor::begin(int direction)
{
  debug_printf("motor using pins %d,%d,%d\n", pins[0], pins[1], pins[2]) ;  
  pinMode(pins[in1Pin], OUTPUT);
  pinMode(pins[in2Pin], OUTPUT);
  //pinMode(standbyPin, OUTPUT);
  
  motorDirectionMode = direction ; // flag to set rotation when moving forward 
  //motorDirection = DIR_FORWARD;    // indicates if wheel moves robot forward or backward  
  motorBrakeMode = false;  // freewheel
  motorStandbyMode = 0;     
  stopMotor();
}

void RobotMotor::setMotorLabel(const char *labelString) // for debug print only
{
   label = labelString;
   PID->label = label;
}

void RobotMotor::setBrakeMode(boolean brakeMode)
{
  // true shorts motor when stopped, false freewheels
  motorBrakeMode = brakeMode;
}

void RobotMotor::setDirectionMode(int directionMode)
{
  //set relationship between motor direction and forward movement   
   motorDirectionMode = directionMode;    
   debug_printf("set direction mode to %d\n", directionMode);   
}

int RobotMotor::getDirection()
{
  // range +-MAX_PWM, positive is forward
  return pwm;
}


inline int RobotMotor::powerToPWM(int power) 
{
  // maps power from range of +-percent to +-MAX_PWM
  power = constrain(power, -100, 100);
  return map(power, -100, 100, -MAX_PWM, MAX_PWM);
}

void RobotMotor::stopMotor()
{
   setMotorPwm(0);
  //stop the motor using the current braking mode 
  digitalWrite(pins[in1Pin], motorBrakeMode);
  digitalWrite(pins[in2Pin], motorBrakeMode);
  PID->stopPid();
  prevAbsPwm = 0;
  debug_printf("%s stopMotor\n", label);
}


void RobotMotor::setStandbyMode(boolean standbyMode)
{
  //set the standby mode if a standby pin has been assigned.
  //invert the value because LOW activates standby on the IC
  if( standbyPin >= 0)
  {
    digitalWrite(standbyPin, !motorStandbyMode);
  }
}

void RobotMotor::setMotorPower(int power) // 0-100%
{  
  int pwm = powerToPWM(power);  
  setMotorPwm( pwm ); 
}

void RobotMotor::setMotorPwm(int requestedPwm)
{
  int motorDirection = requestedPwm >= 0 ? 1 : -1;    
  pwm = constrain(requestedPwm, -MAX_PWM, MAX_PWM);
  int absPwm = abs(pwm);  

  digitalWrite(pins[in1Pin],  !(motorDirection * motorDirectionMode  == 1) );
  //invert the direction for the second control line
  digitalWrite(pins[in2Pin], (motorDirection * motorDirectionMode == 1) );
  //debug_printf("in setMotor: %s pin %d is %d, %d is %d\n", label, pins[in1Pin], digitalRead(pins[in1Pin]),pins[in2Pin], digitalRead(pins[in2Pin])  );     
  isRampingPwm(absPwm);  // control power ramp  
  //debug_printf("in setMotor: %s pwm=%d, absPwm=%d, dir=%d, dir mode=%d, dir mask=%d\n", label, pwm, absPwm, motorDirection, motorDirectionMode, (motorDirectionMode ==1));     
}


/*
 * Function to limit motor spin-up acceleration 
 * needed to prevent motor from drawing too much current at startup 
 * isRamping returns true if the current motor pwm is at least as much as the target PWM  
 */
boolean RobotMotor::isRampingPwm(int absPwm) // returns true if motor coming up to speed
{
  if( prevAbsPwm >= absPwm){
    //debug_printf("in isRampingPwm, %s writing %d but returning false because prevAbsPwm (%d) is >=  absPwm (%d)\n", label, absPwm,  prevAbsPwm, absPwm);
    analogWrite(pins[PWMPin], absPwm);
    return false;  // motor is getting requested pwm level
  }
#ifndef EXTERNAL_PID_SCHEDULAR  
  if( millis() - prevPwmRampTime >= POWER_RAMP_INTERVAL)  
#endif  
  { 
   // increase power at controlled rate   
   debug_printf("%s PWM ramp from %d ", label, prevAbsPwm);
   prevAbsPwm += MAX_PWM_DELTA;
   if( prevAbsPwm > absPwm){
         prevAbsPwm = absPwm;
   }
   analogWrite(pins[PWMPin], prevAbsPwm);
   debug_printf("to %d\n", prevAbsPwm);
   prevPwmRampTime = millis();
  }   
  return true;
}
   

void RobotMotor::setMotorRPM(int RPM, unsigned long duration) 
{ 
  if( abs(RPM) > 0 ) {
      long targetTicksPerSecond = long(((long)RPM * ENCODER_TICKS_PER_WHEEL_REV) / 60 );      
      PID->startPid(targetTicksPerSecond, duration);
      debug_printf("%s %s, ticksPerSecond set to %d, dur=%d\n",
                   label, RPM >= 0 ? "forward":"reversed",targetTicksPerSecond, duration );
  }       
}


/*
 *  Encoder code *  
 */


long RobotMotor::encoderDelta()
{  
   long pos = encoderPos();
   long delta = pos - prevPos;
   prevPos = pos;	
   return delta;  
}

long RobotMotor::encoderPos()
{	
  //Serial.printf("wheel %s,pos = %d\n", label, encoder->read());
  return encoder->read() * motorDirectionMode;
}

void RobotMotor::encoderResetCume()
{    
    encoder->write(0);
    prevPos =0;
}

