/*
 * Pin allocations for mirto boards
 * The order of the following pins is service specific, see the service definition for details
 */
#ifndef robot_pins_h
#define robot_pins_h

#define MIRTO2016 

#if defined(__MK20DX256__) // Teensy 3.x
#ifdef MIRTO2016 
    // pin defines for Mirto 2016 PCB
    #define SWAP_MOTORS  // motor 1 is on the right side of robot 
    #ifdef SWAP_MOTORS // connector marked wheel 1 on right side of robot
      const byte motorPins[]   = {33,20,32,27,30,25};  // L-in1,L-in2,L-PWM, R-in1,R-in2,R-PWM
      const byte encoderPins[] = {29,31,26,28}; // Left:A,B  Right:A,B
    #else	
      // todo  - check motor and encoders rotate in correct sense	
      const byte motorPins[]   = {30,27,25, 20,33,32};  // L-in1,L-in2,L-PWM, R-in1,R-in2,R-PWM
      const byte encoderPins[] = {26,28,29,31}; // Left:A,B  Right:A,B

    #endif	
    const byte  switchPin   = 5;
    const byte bumpPins[] = {6,24};
    const byte irReflectancePins[] = {A0,A1,A2,A3}; // first is control, the remainder are used as analog inputs
    const byte servoPins[]    = {3};
    const byte distancePins[] = {4};
    // the following pins use preprocessor defines to enable conditional compile
    #define neoPixelPin 2
    #define tonePin     9
    #define ledPin     13

  #else  // defines for first Teensy/HUBee board
      const byte motorPins[] = {6,20,25,27,30,32};  // L-in1,L-in2,L-PWM, R-in1,R-in2,R-PWM
      const byte encoderPins[] = {26,28,29,31}; // Left:A,B  Right:A,B
      const byte bumpPins[] = {24,33};
      const byte irReflectancePins[] = {A8,A9,A12,A13}; // first is control, the remainder are used as analog inputs
      const byte servoPins[]    = {3};
      const byte distancePins[] = {4};
      const byte  switchPin   = 5;
      // the following pins use preprocessor defines to enable conditional compile
      #define neoPixelPin 2
      #define tonePin     9
      #define ledPin     13
  #endif // end of defines for first Teensy/HUBee board

#else  // defines for standard Mirto 328 V2 board
    const byte motorPins[] = {8,11,5,12,13,6}; 
    const byte encoderPins[] = {3,7,2,4}; // Left:A,B  Right:A,B

    const byte bumpPins[]  = {9,10};
    // note that analog pins are referred to by their digital number (on uno, 15 = analog 1, 16 =analog 2...)
    const byte irReflectancePins[] = {14,15,16,17}; // first is control, the remainder are used as analog inputs
    const byte servoPins[] = {18};     // analog pin 4
    const byte distancePins[] = {19};  // analog pin 5 
#endif
// Seperate arrays are a requirement of the third party encoder library 
#define leftMotorPins    (&motorPins[0])
#define rightMotorPins   (&motorPins[3])
#define encoderLeftPins  (&encoderPins[0])
#define encoderRightPins (&encoderPins[2])

#endif
