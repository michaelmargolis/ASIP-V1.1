/* MotorPid.h
 * PID support for robot motors.
 * such as Mirto v2 board  
 * Michael Margolis July 2016
 */

#ifndef MotorPid_h
#define MotorPid_h
#include "Arduino.h"
//#include <functional> // for Lambda

const int PID_FRAME_HZ        = 30;                       //  frames per second  
const int AUTO_STOP_INTERVAL  = 2000; // turn off PWM if no encoder pulses after this many ms

/*
 * PID code manage frome intervals or relagate this to an extrnal schedular
 * uncomment EXTERNAL_PID_SCHEDULAR if used with ASIP or other scheduling code
 */
#define EXTERNAL_PID_SCHEDULAR // define this frame intervals are controlled externally

typedef void (*motorPwmFunc)(int); // callback to control motor PWM
 
class MotorPID
{
    public:
        MotorPID(int encoderPPR, int maxPwm, int maxPwmDelta );
        void initPid(int Kp, int Ki, int Kd, int Ko ); 
        void startPid( long targetTicksPerSecond, long duration);
        boolean servicePid(long count, motorPwmFunc setMotorPwm);
        void stopPid();
        void updatePid( int Kp, int Ki, int Kd, int Ko, unsigned long interval);
        void getPid( int &Kp, int &Ki, int &Kd, int &Ko,unsigned long &interval);
        boolean isPidServiceNeeded();
        boolean isPidActive();
         
        const char * label; // motor label, for debug only  

    private:      
     
        int encoderPPR;                          // encoder pulses per revolution         
        int maxPwm;                              // max PWM motor value
        int maxPwmDelta;                         // max change in PWM between consecutive frames    
        long targetTicksPerSecond;               // target speed in ticks per second
        long encoder;                            // encoder count               
        int prevInput;                           // last input 
        int iTerm;                               // integrated term
        int prevOutput;                          // last motor setting 

        unsigned long prevPidService;            // time of last PID calculation 
        unsigned long prevPulseMillis;           // time of frame containing most recent encoder pulse (for auto shotoff)
        unsigned long startTime;                 // millis time of setMotorRPM call (for duration timeout)
        unsigned long duration;                  // number of milliseconds to run (-1 runs for 49+ days)
        boolean isActive;                        // true if PID is enabled and motor is moving
        int16_t Kp;
        int16_t Ki;
        int16_t Kd;
        int16_t Ko;                              // scaling divisor enables use of integer PID constants
        unsigned long frameInterval;             // 1000 / PID_FRAME_HZ (milliseconds for each frame) 
        
};

#endif