
#include <asip.h>       // the base class definitions
#include <asipIO.h>     // the core I/O class definition
#include <service/asipHeading.h>    // Inertial measurement services 
#include <HMC5883L.h>   // magnetometer

const char * sketchName = "asipHeading";

bool i2cStarted = false; // flag to indcate that i2c started

HeadingClass heading3Axis(id_HEADING_SERVICE);

asipService services[] = { 
    &heading3Axis,
    &asipIO // the core class for pin level I/O
}    
                               

void setup() {
  while(!Serial); // For non-native serial (leonardo & teensy3)
  Serial.begin(57600);
  
  asip.begin(&Serial,  asipServiceCount(services), services, sketchName); 
  asip.reserve(SERIAL_RX_PIN);  // reserve pins used by the serial port 
  asip.reserve(SERIAL_TX_PIN);  // these defines are in asip/boards.h 
  heading3Axis.begin(NBR_MAG_AXIS,startI2C); // 3 raw values, 4th element is the calculated compass heading 
  asipIO.begin(); // start the IO service
  
  for(unsigned int i=0; i< asipServiceCount(services); i++)
  {
    services[i]->reportName(&Serial); 
    Serial.print(" is service ");
    Serial.write(services[i]->getServiceId());
    Serial.println();  
  }
}

void loop() 
{
  asip.service();
}

bool startI2C( char service)
{
  // for now, all services that call this method require I2C
  if( !i2cStarted) {  
     asip.reserve(SDA);  // reserve pins used by I2C 
     asip.reserve(SCL);  // these defines are in pins_arduino.h  
     Wire.begin();
     i2cStarted = true;
  }
  return true; 
}

