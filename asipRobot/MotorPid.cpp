/* MotorPid.cpp
 * PID support for robot motors.
 * such as Mirto v2 board  
 * Michael Margolis September 2016
 */
 
#include "MotorPid.h"
#include "utility/debug.h"
 
MotorPID::MotorPID(int encoderPPR, int maxPwm, int maxPwmDelta)
{
   this->encoderPPR = encoderPPR; 
   this->maxPwm = maxPwm;
   this->maxPwmDelta = maxPwmDelta;     
}
 
void  MotorPID::initPid(int Kp, int Ki, int Kd, int Ko )
{
  targetTicksPerSecond = 0;  
  prevOutput = prevInput = iTerm = 0;
 
  // set values from eeprom
  this->Kp = Kp; // 20;   5
  this->Kd = Kd; // 12;
  this->Ki = Ki; // 0;
  this->Ko = Ko;  // 20;
  frameInterval = 1000/PID_FRAME_HZ;
  debug_printf("PID vals: %d, %d, %d, %d\n", this->Kp, this->Kd, this->Ki, this->Ko );  
  debug_printf("Motor encoderPPR=%d, maxPwm = %d, maxPwmDelta=%d\n",  encoderPPR, maxPwm, maxPwmDelta );
}

void MotorPID::updatePid( int Kp, int Ki, int Kd, int Ko, unsigned long interval)
{
  this->Kp = Kp;
  this->Ki = Ki;   
  this->Kd = Kd;
  this->Ko = Ko;
  this->frameInterval =  interval;
  debug_printf("PID vals: %d, %d, %d, %d\n", this->Kp, this->Kd, this->Ki, this->Ko ); 
}

void MotorPID::getPid( int &KpOut, int &KiOut, int &KdOut, int &KoOut, unsigned long &interval)
{
  KpOut = this->Kp;
  KiOut = this->Ki;   
  KdOut = this->Kd;
  KoOut = this->Ko;
  interval = frameInterval;
}

void MotorPID::startPid( long targetTicks, long dur)
{   
  isActive = true;
  startTime = prevPulseMillis = prevPidService = millis();
  duration = dur;  
  prevOutput = prevInput = 0;
  targetTicksPerSecond = targetTicks;
  debug_printf("%s pid started, ticks=%d, dur=%d\n",label, targetTicksPerSecond, dur);
}

boolean MotorPID::isPidServiceNeeded()
{
#ifndef  EXTERNAL_PID_SCHEDULAR
  return( isPidActive()  && (millis() - prevPidService >= frameInterval))  ;
#else 
  return isPidActive();
#endif
}


boolean MotorPID::isPidActive()
{
  return isActive;
}

void  MotorPID::stopPid()
{
  this->isActive = false;
}

// return true unless PID is inactive or has timed out.
boolean  MotorPID::servicePid(long encoderCount, motorPwmFunc setMotorPwm)
{
long pError;
long pid;
long input;
int output;
unsigned long timeDelta;
  
  if(!isActive) {         
    return false;
  } 
  unsigned long timeNow = millis();
  if( timeNow - startTime > duration) {       
        debug_printf("%s stopped- duration achieved\n", label); 
        isActive = false;        
        return false;  
  } 
  if( isPidServiceNeeded() ) {     
     timeDelta = timeNow - prevPidService;
     prevPidService = timeNow;  
    input = encoderCount; 
    if( input != 0) {
       prevPulseMillis = timeNow;
    }
    else if( timeNow - prevPulseMillis > AUTO_STOP_INTERVAL) {       
        debug_printf("\nauto stop because no encoder pulse after %d ms\n",AUTO_STOP_INTERVAL );
        isActive = false;
        return false;               
    }  
    long targetTicks = map(timeDelta, 0, 1000, 0, targetTicksPerSecond);
    pError = targetTicks - input;
    pid = (Kp * pError - Kd * (input - prevInput) + iTerm) / Ko;    
    
    // acceleration limit
    if( pid > maxPwmDelta)
       pid = maxPwmDelta;
    else if (pid < -maxPwmDelta)
       pid = -maxPwmDelta;
       
    output = pid + prevOutput;
   
    // Accumulate Integral error *or* Limit output.
    // Stop accumulating when output saturates
    if (output >= maxPwm)
      output = maxPwm;
    else if (output <= -maxPwm)
      output = -maxPwm;
    else
      iTerm += Ki * pError;

    prevOutput = output;
    prevInput = input;
    
    setMotorPwm(output);    
    debug_printf("%s svc after %dms, in=%d, err=%d, pid=%d, out=%d\n", label, timeDelta, input, pError, pid, output);
  } 
  else {
    debug_printf("!ERROR - PID service called too soon, check isPidServiceNeeded() method before calling\n");
    delay(500);
  }
  return true;
}