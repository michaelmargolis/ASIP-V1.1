/*
 * robot.cpp -  Arduino Services Interface Protocol (ASIP)
 * 
 * Copyright (C) 2014 Michael Margolis
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#include "asipRobot.h"
#include "robot_pins.h" 
#include "RobotDescription.h"
#include "RobotMotor.h"

#include "config.h"
 

//declare two encoder objects
static Encoder encoderLeftWheel(encoderLeftPins[0], encoderLeftPins[1]);
static Encoder encoderRightWheel(encoderRightPins[0], encoderRightPins[1]);

static boolean encoderEventsFlag = false; // true enables encoder events

// left and right motor instances
static RobotMotor wheel[NBR_WHEELS] = 
{
  // RobotMotor constructor args:  pins and method to read encoder
  RobotMotor(leftMotorPins, &encoderLeftWheel),
  RobotMotor(rightMotorPins, &encoderRightWheel)
};

const char *motorLabels[NBR_WHEELS] = {"left ", "Right"};  //just for debug messages, 

// static callbacks to enable PID to set motor PWM 
static void leftMotorCallback(int pwm)
{   
   wheel[0].setMotorPwm(pwm);
}  

static void rightMotorCallback(int pwm)
{
    wheel[1].setMotorPwm(pwm); 
} 
  
robotMotorClass::robotMotorClass(const char svcId, const char evtId)
  :asipServiceClass(svcId)
{
  svcName = PSTR("Motors");
}

/*
 * Begin method for motors with encoders
 * each motor uses 3 pins, the array order is: m0In1, m0In2, m0pwm, m1In1, m1In2, m1pwm, ...)
 * pinCount is three times the number of motors  
 */
 void robotMotorClass::begin(byte nbrElements, byte motorPinCount, const pinArray_t pins[], byte encoderPinCount, const pinArray_t encoderPins[])
{
  asipServiceClass::begin(nbrElements,motorPinCount,pins, encoderPinCount, encoderPins);
  if (restoreConfig() == false) // try and retrieve saved PID from eeprom
  {
    debug_printf("Config not found, using default PID values\n");
    saveConfig(); // store default PID values
  }
#ifdef HUBEE_WHEELS
  debug_printf("Initializing robot for HUBee wheels");
  delay(1000);
#endif 
  for(int i=0; i < NBR_WHEELS; i++) {
     wheel[i].setDirectionMode(0); //Direction Mode determines how the wheel responds to positive and negative motor power values 
     wheel[i].setBrakeMode(0);     //Sets the brake mode to zero - freewheeling mode - so wheels are easy to turn by hand
     wheel[i].PID->initPid(configData.Kp, configData.Ki, configData.Kd, configData.Ko );
     wheel[i].encoderResetCume();
     wheel[i].setMotorLabel(motorLabels[i]); // just for debug messages, can be removed
  }
  wheel[0].begin(NORMAL_DIRECTION); 
  wheel[1].begin(NORMAL_DIRECTION);  
  setAutoreport(1000/PID_FRAME_HZ);  
}

/*
 * Begin method for motors without encoders (no PID control)
 */  
 void robotMotorClass::begin(byte nbrElements, byte pinCount, const pinArray_t pins[])
{  
  asipServiceClass::begin(nbrElements,pinCount,pins);
  for(int i=0; i < NBR_WHEELS; i++) {
     wheel[i].setDirectionMode(0); //Direction Mode determines how the wheel responds to positive and negative motor power values 
     wheel[i].setBrakeMode(0);     //Sets the brake mode to zero - freewheeling mode - so wheels are easy to turn by hand
  }
  wheel[0].begin(REVERSED_DIRECTION); // robot moves forward with positive values
  wheel[1].begin(NORMAL_DIRECTION);
  
  wheel[0].setMotorLabel("Left "); // just for debug, can be removed 
  wheel[1].setMotorLabel("Right"); // as above
}

void robotMotorClass::reset()
{
  stopMotors();
}

// reportValues services PID and reports encoder events if encoderEventsFlag is true
void robotMotorClass::reportValues(Stream *stream)
{
   // cache encoder data for sending to PID and reporting event
   delta[0] = wheel[0].encoderDelta();
   count[0] = wheel[0].encoderPos(); 
   delta[1] = wheel[1].encoderDelta();
   count[1] = wheel[1].encoderPos();
   wheel[0].isRampingPwm();  // motor acceleration control 
   wheel[1].isRampingPwm();
   if( wheel[0].PID->isPidServiceNeeded()) {
       if(!wheel[0].PID->servicePid(delta[0], leftMotorCallback))
            wheel[0].stopMotor();
   }   
   if( wheel[1].PID->isPidServiceNeeded()) {
       if(!wheel[1].PID->servicePid(delta[1], rightMotorCallback))
            wheel[1].stopMotor();
   } 
   if(encoderEventsFlag) {
      asipServiceClass::reportValues(stream);        
   }   
}

void robotMotorClass::reportValue(int sequenceId, Stream * stream)  // send the value of the given device
{
   if( sequenceId < nbrElements) {
       stream->print(delta[sequenceId]);
       stream->write(':');   
       stream->print(count[sequenceId]);
    }
}
  
void robotMotorClass::setMotorPower(byte motor, int power)
{
   if(motor < NBR_WHEELS){       
       wheel[motor].setMotorPower(power);
     //  debug_printf("Motor %d set to %d\n", motor, power);
   }
}

void robotMotorClass::setMotorPowers(int power0, int power1)
{
  // TODO this assumes only two motors
   setMotorPower(0, power0);
   setMotorPower(1, power1);
}

void robotMotorClass::setMotorRPM(byte motor, int rpm, long duration)
{
   if(motor < NBR_WHEELS){      
       wheel[motor].setMotorRPM(rpm, duration);
       debug_printf("Motor %d rpm set to %d for %d ms\n", motor, rpm, duration);
   }
}

void robotMotorClass::setMotorsRPM(int rpm0, int rpm1, long duration)
{
  // TODO this assumes only two motors
   setMotorRPM(0, rpm0, duration);
   setMotorRPM(1, rpm1, duration);
}


void robotMotorClass::setRobotSpeedCmPerSec(int cmps, long duration)
{
  int rpm = (int)(60 * cmps) / (WHEEL_CIRCUMFERENCE / 10.0); // circumference is in mm 
  //debug_printf.printf("set rpm to %d for a speed of %d cm per sec, dur=%ld\n", rpm, cmps, duration);
  setMotorsRPM(rpm, rpm, duration);  
}
   
void robotMotorClass::rotateRobot( int dps, int angle)
{  
  long dur = abs((angle * 1000L) / dps);  
  int distance = TRACK_CIRCUMFERENCE * angle / 360.0; // distance in mm
  int speed = distance * 100L / dur; // absolute wheel speed in cm per sec 
  dur =  distance *100L / speed; // recalculated because speed in cm/sec is rounded down. 
  int rpm = (int)(60 * speed) / (WHEEL_CIRCUMFERENCE / 10.0); 
  //debug_printf("\nRotateAngle: dps= %d, angle=%d, dur= %ld, speed=%d cm/sec, distance = %d cm, RPM=%d\n", dps,angle, dur, speed, distance, rpm); 
  setMotorsRPM(rpm, -rpm, dur);    
}
   
void robotMotorClass::stopMotor(byte motor)
{
   if(motor < NBR_WHEELS){
       wheel[motor].stopMotor();
      // debug_printf("Motor %d stopped\n", motor);
   }
}

void robotMotorClass::stopMotors()
{
    for(int i=0; i < NBR_WHEELS; i++ ){
        wheel[i].stopMotor();
    }    
}
   
void robotMotorClass::resetEncoderTotals()
{
    for(int i=0; i < NBR_WHEELS; i++ ){
       wheel[i].encoderResetCume();
    }    
}
   
void robotMotorClass::processRequestMsg(Stream *stream)
{
   int arg0 =-1, arg1 = -1, arg2=-1; 
   int request = stream->read();       
   // parse the correct number of arguments for each method 
   arg0 = stream->parseInt();  
   if(request == tag_AUTOEVENT_REQUEST) {
       // unlike other services, motor autoevents is always on, this request enables or disable the sending of encoder data
        encoderEventsFlag = (arg0 != 0);  
   }
   else{ 
       if( !(request == tag_STOP_MOTORS || request == tag_RESET_ENCODERS) ) { // these have no args                 
          arg1 = stream->parseInt();  
          if( request == tag_SET_MOTOR_RPM || request == tag_SET_MOTORS_RPM ) {
            arg2 = stream->parseInt();  
          }
       }           
       switch(request) {
          case tag_SET_MOTOR:  setMotorPower(arg0,arg1);  break;
          case tag_SET_MOTORS: setMotorPowers(arg0,arg1);break; // TODO this assumes only two motors
          case tag_SET_MOTOR_RPM:  setMotorRPM(arg0,arg1,arg2); break;
          case tag_SET_MOTORS_RPM: setMotorsRPM(arg0,arg1,arg2); break;   
          case tag_SET_ROBOT_SPEED_CM : setRobotSpeedCmPerSec(arg0,arg1); break; 
          case tag_ROTATE_ROBOT_ANGLE: rotateRobot(arg0, arg1); break ;
          case tag_STOP_MOTOR:  stopMotor(arg0); break;
          case tag_STOP_MOTORS: stopMotors(); break; 
          case tag_RESET_ENCODERS: resetEncoderTotals(); break;
          default: reportError(ServiceId, request, ERR_UNKNOWN_REQUEST, stream);
       }
       
   }
}

encoderClass::encoderClass(const char svcId) : asipServiceClass(svcId)
{
  svcName = PSTR("Encoders");; 
}

// each encoder uses 2 pins
void encoderClass::begin(byte nbrElements, byte pinCount, const pinArray_t pins[])
{  
  asipServiceClass::begin(nbrElements,pinCount,pins);
}

void encoderClass::reportValues(Stream *stream) 
{
  //encodersGetData(delta[0], count[0], delta[1], count[1]);
  delta[0] = wheel[0].encoderDelta();
  count[0] = wheel[0].encoderPos();
  delta[1] = wheel[1].encoderDelta();
  count[1] = wheel[1].encoderPos();
  asipServiceClass::reportValues(stream);    
  // delta width removed in this version
}

void encoderClass::reset()
{
  wheel[0].encoderResetCume();
  wheel[1].encoderResetCume();  
}

 void encoderClass::reportValue(int sequenceId, Stream * stream)  // send the value of the given device
{
   if( sequenceId < nbrElements) {
       stream->print(delta[sequenceId]);
       stream->write(':');   
       stream->print(count[sequenceId]);
    }
}

void encoderClass::processRequestMsg(Stream *stream)
{
   int request = stream->read();
   if(request == tag_AUTOEVENT_REQUEST) {
     setAutoreport(stream);
   }
   else {
     reportError(ServiceId, request, ERR_UNKNOWN_REQUEST, stream);
   }
}

bumpSensorClass::bumpSensorClass(const char svcId) : asipServiceClass(svcId)
{
  svcName = PSTR("Bump Sensors");
}

void bumpSensorClass::bumpSensorClass::begin(byte nbrElements, byte pinCount, const pinArray_t pins[])
{ 
  asipServiceClass::begin(nbrElements,pinCount,pins);
  for(int sw=0; sw < nbrElements; sw++) {     
     pinMode(pins[sw], INPUT_PULLUP); 
  }
}

void bumpSensorClass::reset()
{

}

 void bumpSensorClass::reportValue(int sequenceId, Stream * stream)  // send the value of the given device
{
   if( sequenceId < pinCount) {
       //pinMode(pins[sequenceId], INPUT_PULLUP); 
       boolean value = digitalRead(pins[sequenceId]);
       stream->print(value ? "1":"0");
    }
}

void bumpSensorClass::processRequestMsg(Stream *stream)
{
   int request = stream->read();
   if(request == tag_AUTOEVENT_REQUEST) {
      setAutoreport(stream);
   }
   else {
     reportError(ServiceId, request, ERR_UNKNOWN_REQUEST, stream);
   }
}

irLineSensorClass::irLineSensorClass(const char svcId) : asipServiceClass(svcId)
{
   svcName = PSTR("IR Sensors");
}

void irLineSensorClass::begin(byte nbrElements, byte pinCount, const pinArray_t pins[]) 
{
  asipServiceClass::begin(nbrElements,pinCount,pins);
  pinMode(pins[0], OUTPUT);
  digitalWrite(pins[0], LOW);
}

void irLineSensorClass::reportValues(Stream *stream) 
{
   // turn on IR emitters
   digitalWrite(pins[0], HIGH);
   delayMicroseconds(200); // reduced delay time 1 July 2014
   asipServiceClass::reportValues(stream);
    // turn off IR emitters
   digitalWrite(pins[0], LOW);
}

void irLineSensorClass::reset()
{

}

void irLineSensorClass::reportValue(int sequenceId, Stream * stream)  // send the value of the given device
{
   if( sequenceId < nbrElements) {
      int pin = pins[sequenceId+1]; // the first pin is the control pin
      pin = PIN_TO_ANALOG(pin); // convert digital number to analog
      int value = analogRead(pin);   
      stream->print(value);
    }
}

void irLineSensorClass::processRequestMsg(Stream *stream)
{
   int request = stream->read();
   if(request == tag_AUTOEVENT_REQUEST) {
      setAutoreport(stream);
   }
   else {
     reportError(ServiceId, request, ERR_UNKNOWN_REQUEST, stream);
   }
}