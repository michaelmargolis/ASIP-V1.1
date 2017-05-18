/*
 * robot.h -  Arduino Services Interface Protocol (ASIP)
 * 
 * Copyright (C) 2014 Michael Margolis
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#ifndef robot_h
#define robot_h

#include "asip.h"
#include "robot_pins.h"

// Service and method defines
// Service IDs must be unique across all services
// Method and event IDs must be unique within a service

// Motor service
const char id_MOTOR_SERVICE = 'M';
// Motor methods (messages to Arduino)
const char tag_SET_MOTOR            = 'm'; // sets motor power  
const char tag_SET_MOTORS           = 'M';
const char tag_SET_MOTOR_RPM        = 'r'; // wheel rpm
const char tag_SET_MOTORS_RPM       = 'R'; // both wheels rpm 
const char tag_SET_ROBOT_SPEED_CM   = 'c'; // speed in Cm per Sec using PID
const char tag_ROTATE_ROBOT_ANGLE   = 'a'; // Robot rotation using given degrees per second and angle 
const char tag_STOP_MOTOR           = 's';  
const char tag_STOP_MOTORS          = 'S';
const char tag_RESET_ENCODERS       = 'E'; // rest total counts to zero



// Encoder service
const char id_ENCODER_SERVICE = 'E';
// Encoder methods - use system define, tag_AUTOEVENT_REQUEST ('A') to request autoevents
// Encoder events -  events use system tag: tag_SERVICE_EVENT  ('e')


// Bump detect service
const char id_BUMP_SERVICE = 'B';
// Bump sensor methods - use system define, tag_AUTOEVENT_REQUEST ('A') to request autoevents
// Bump Sensor events -  events use system tag: tag_SERVICE_EVENT  ('e')


// IR Line detect service
const char id_IR_REFLECTANCE_SERVICE = 'R';
// IR Line detect methods - use system define, tag_AUTOEVENT_REQUEST ('A') to request autoevents
// IR Line detect events -  events use system tag: tag_SERVICE_EVENT  ('e')


const int NBR_WHEELS = 2;  // defines the number of wheels (and encoders), note not tested with values other than 2

class robotMotorClass : public asipServiceClass
{  
public:

   robotMotorClass(const char svcId, const char evtId);  
   void begin(byte nbrElements, byte pinCount, const pinArray_t pins[]);
   void begin(byte nbrElements, byte motorPinCount, const pinArray_t pins[], byte encoderPinCount, const pinArray_t encoderPins[]);
   void reset();
   void reportValue(int sequenceId, Stream * stream) ; // send the value of the given device
   void reportValues(Stream *stream);   
   void setMotorPower(byte motor, int power);
   void setMotorPowers(int power0, int power1);
   void setMotorRPM(byte motor, int rpm, long duration);
   void setMotorsRPM(int rpm0, int rpm1, long duration);
   void setRobotSpeedCmPerSec(int cmps, long duration);  
   void rotateRobot( int dps, int angle);
   void stopMotor(byte motor);
   void stopMotors();
   void resetEncoderTotals();
   void processRequestMsg(Stream *stream);
 //  void reportName(Stream *stream);
 private:
  long delta[NBR_WHEELS];  // this is now the number of ticks since the last event
  long count[NBR_WHEELS];
 };
   
class encoderClass : public asipServiceClass
{  
public:
   encoderClass(const char svcId);
   void begin(byte nbrElements, byte pinCount, const pinArray_t pins[]);
   void reset();
   void reportValues(Stream * stream);
   void reportValue(int sequenceId, Stream * stream) ; // send the value of the given device   
   void processRequestMsg(Stream *stream);
  // void reportName(Stream *stream);
private:
  long delta[NBR_WHEELS];  // this is now the number of ticks since the last event
  long count[NBR_WHEELS];  
};   
   

class bumpSensorClass : public asipServiceClass
{  
public:
   bumpSensorClass(const char svcId);
   void begin(byte nbrElements, byte pinCount, const pinArray_t pins[]);
   void reset();
   void reportValue(int sequenceId, Stream * stream) ; // send the value of the given device
   void processRequestMsg(Stream *stream);
  // void reportName(Stream *stream);
};

class irLineSensorClass : public asipServiceClass
{  
public:
   irLineSensorClass(const char svcId);
   void begin(byte nbrElements, byte pinCount, const pinArray_t pins[]);
   void reset();
   void reportValues(Stream *stream);
   void reportValue(int sequenceId, Stream * stream) ; // send the value of the given device
   void processRequestMsg(Stream *stream);
  // void reportName(Stream *stream);
};    


#endif
 
   


