/*
 * RobotDescription.h
 * Defines for physical robot
 */

#ifndef RobotDescription_h
#define RobotDescription_h

//#define HUBEE_WHEELS   // uncomment this if using HUBee wheels

// units are millimeters

#ifdef HUBEE_WHEELS
const float WHEEL_DIAMETER =  60;
const float WHEEL_TRACK = 112;
const int   ENCODER_TICKS_PER_WHEEL_REV  = 64;
#else
const float WHEEL_DIAMETER =  64;
const float WHEEL_TRACK = 162;
const int   GEAR_REDUCTION = 34; // 1:34
const int   PPR = 12; // note each pulse triggers 4 interrupts
const int   ENCODER_TICKS_PER_WHEEL_REV = GEAR_REDUCTION * PPR * 4; // org motor was 1248; 
#endif
const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * PI;
const float TRACK_CIRCUMFERENCE = WHEEL_TRACK * PI;
#endif
