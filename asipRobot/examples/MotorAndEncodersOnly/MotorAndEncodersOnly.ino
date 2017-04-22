/*    
 * sketch for motor control with Teensy 3.1 board
 * 
 *  This sketch requires the asipRobot library although only the motor and encoder services are used.
 *  asipPixels is only included to dim the onboard neopixel LED at startup
 */

#define DONT_USE_PIN_IO  // uncomment this to supress ASIP display and control of digital and analog pins

#include <asip.h>                  // the base class definitions
#include <asipIO.h>                // the core I/O class definition


// the following services use seperate libraries 
#include <asipRobot.h>        // definitions for mirtle services(motor, ir, encoder etc) 
#include <asipPixels.h>       // for neopixels 


char const *sketchName = "MotorsOnly";

// only needed if you want to dim the neopixel on Mirto board pin 2
Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, 2, NEO_RGB + NEO_KHZ400);

// create the services
robotMotorClass motors(id_MOTOR_SERVICE, NO_EVENT);
//encoderClass encoders(id_ENCODER_SERVICE);

                 
// make a list of the created services
asipService services[] = { 
#ifndef DONT_USE_PIN_IO
   &asipIO, // the core class for pin level I/O
#endif                                 
   &motors};

void setup()
{  
  dimNeopixel();
  beginAsipComms();   
  // start the services
  motors.begin(2,6,motorPins,4,encoderPins); // two motors,a total of 6 motor pins,4 encoder pins    
  //motors.begin(2,6,motorPins); // two motors that use a total of 6 pins, no encoder (no PID)  
  //encoders.begin(2,4,encoderPins); // two encoders that use a total of 4 pins 
#ifndef DONT_USE_PIN_IO  
#ifdef ledPin  
 asipIO.PinMode(ledPin,OUTPUT_MODE); 
#endif  
  asipIO.begin();  // NEW from  v1.1: core I/O service must follow all other service begin methods
  
  asip.sendPinModes(); // for debug
  asip.sendPortMap(); 
#endif

  for(unsigned int i=0; i < asipServiceCount(services); i++)
  {
    services[i]->reportName(&Serial); 
    Serial.print(" is service ");
    Serial.write(services[i]->getServiceId());
    Serial.println();  
  }
}

void beginAsipComms()
{

  Serial.begin(ASIP_BAUD);   // init both ports even though only one used for ASIP
  Serial3.begin(ASIP_BAUD); 
  delay(200); // wait for USB to init - todo     
  if( bitRead(USB0_OTGSTAT,5))  // todo temp force ignore
  {
    // Pi connected to Serial3 UART
     asip.begin(&Serial3, asipServiceCount(services), services, sketchName);    
  }
  else {
     while(!Serial); // wait for port
     asip.begin(&Serial, asipServiceCount(services), services, sketchName); 
    
  }
  asip.reserve(SERIAL3_RX_PIN);  // reserve pins used for Pi serial comms 
  asip.reserve(SERIAL3_TX_PIN);  // board specific defines are in asip/boards.h
}

void loop() 
{
  asip.service();
}


// set the onboard LED to very dim blue
 void dimNeopixel()
{
   strip.begin();   
   strip.show(); // Initialize all pixels to 'off' 
   strip.setPixelColor(0,strip.Color(0,0,3));
   strip.show(); // Initialize all pixels to 'off'
}