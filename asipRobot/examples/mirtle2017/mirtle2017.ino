/*    
 * Mirtle sketch for robot with Teensy 3.1 board
 * 
 *  This sketch depends on the following libraries in addition to core ASIP:
     (libraries preceeded with * are third party, either included with teensy install or via library manager)
     asipRobot
       * Encoder 
     asipPixels
       * Adafruit_NeoPixel 
     asipLCD
       * U8g2   
          
 */

#include <asip.h>                  // the base class definitions
#include <asipIO.h>                // the core I/O class definition

// the following three services are supported by the standard Arduino distribution
#include <services/asipTone.h>      // square wave tone generator
#include <services/asipDistance.h>  // ultrasonics distance sensor
#include <services/asipServos.h>    // derived definitions for servo

// the following services use seperate libraries 
#include <asipRobot.h>        // definitions for mirtle services(motor, ir, encoder etc) 
#include <asipLCD.h>          // for LCD display
#include <asipPixels.h>       // for neopixels 


char const *sketchName = "Mirtle2017";

//declare servo object(s) 
const byte NBR_SERVOS =1;
Servo myServos[NBR_SERVOS];  // create servo objects
asipCHECK_PINS(servoPins[NBR_SERVOS]);  // compiler will check if the number of pins is same as number of servos

const byte NBR_DISTANCE_SENSORS = 1;
asipCHECK_PINS(distancePins[NBR_DISTANCE_SENSORS]); //this declaration tests for correct nbr of pin initializers

const byte MAX_PIXELS = 9;  // increase this if more pixels are required
Adafruit_NeoPixel strip = Adafruit_NeoPixel(MAX_PIXELS, neoPixelPin);  // instantiate a strip

// create the services
robotMotorClass motors(id_MOTOR_SERVICE, NO_EVENT);
//encoderClass encoders(id_ENCODER_SERVICE);
bumpSensorClass bumpSensors(id_BUMP_SERVICE);
irLineSensorClass irLineSensors(id_IR_REFLECTANCE_SERVICE);
asipToneClass asipTone(id_TONE_SERVICE, NO_EVENT);
asipServoClass asipServos(id_SERVO_SERVICE, NO_EVENT);
asipDistanceClass asipDistance(id_DISTANCE_SERVICE);
asipLCDClass asipLCD(id_LCD_SERVICE, NO_EVENT);
asipPixelsClass asipPixels(id_PIXELS_SERVICE, NO_EVENT);
                 
// make a list of the created services
asipService services[] = { 
                                 &asipIO, // the core class for pin level I/O
                                 &motors,
                                 //&encoders,
                                 &bumpSensors,
                                 &irLineSensors,
                                 &asipTone, 
                                 &asipServos,                                 
                                 &asipDistance, 
                                 &asipLCD,
                                 &asipPixels};

void setup()
{
  asipLCD.begin();
  beginAsipComms();  
  asip.reserve(SDA);  // reserve pins used by I2C 
  asip.reserve(SCL);  // these defines are in pins_arduino.h  
  
  // start the services
  motors.begin(2,6,motorPins,4,encoderPins); // two motors,a total of 6 motor pins,4 encoder pins    
  //motors.begin(2,6,motorPins); // two motors that use a total of 6 pins, no encoder (no PID)  
  //encoders.begin(2,4,encoderPins); // two encoders that use a total of 4 pins 
  bumpSensors.begin(2,2,bumpPins);
  irLineSensors.begin(3,4,irReflectancePins); // 3 sensors plus control pin
  asipDistance.begin(NBR_DISTANCE_SENSORS,distancePins); 
  asipServos.begin(NBR_SERVOS,servoPins,myServos);
#ifdef tonePin  
  asipTone.begin(tonePin);
#endif    
#ifdef neoPixelPin  
  asipPixels.begin(neoPixelPin, &strip);
#endif  
#ifdef ledPin  
 asipIO.PinMode(ledPin,OUTPUT_MODE); 
#endif  

  asipIO.begin();  // NEW from  v1.1: core I/O service must follow all other service begin methods
  
  asip.sendPinModes(); // for debug
  asip.sendPortMap(); 

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
  char buffer[22];
  sprintf(buffer,"ASIP Ver %d.%d",ASIP_MAJOR_VERSION,ASIP_MINOR_VERSION );
  asipLCD.text(buffer,0);
  asipLCD.text(sketchName,1); 
  asipLCD.text(CHIP_NAME,2);

  Serial.begin(ASIP_BAUD);   // init both ports even though only one used for ASIP
  Serial3.begin(ASIP_BAUD); 
  delay(200); // wait for USB to init - todo   
  if( bitRead(USB0_OTGSTAT,5))
  {
    // Pi connected to Serial3 UART
     asip.begin(&Serial3, asipServiceCount(services), services, sketchName); 
     asipLCD.text("Waiting for IP",4);
  }
  else {
     asip.begin(&Serial, asipServiceCount(services), services, sketchName); 
     asipLCD.text("ASIP using USB",4);
  }
  asip.reserve(SERIAL3_RX_PIN);  // reserve pins used for Pi serial comms 
  asip.reserve(SERIAL3_TX_PIN);  // board specific defines are in asip/boards.h
}

void loop() 
{
  asip.service();
}

  