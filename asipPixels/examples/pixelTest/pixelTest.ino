/*
   pixel test
*/

#include <asip.h>       // the base class definitions
#include <asipIO.h>     // the core I/O class definition

#include <asipPixels.h>

char const *sketchName = "PixelTest";

#define neoPixelPin  2 
const int MAX_PIXELS = 9;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(MAX_PIXELS, neoPixelPin, NEO_RGB + NEO_KHZ400);

// create the services
asipPixelsClass asipPixels(id_PIXELS_SERVICE, NO_EVENT);

// make a list of the created services
asipService services[] = {
  &asipIO, // the core class for pin level I/O  
  &asipPixels
};

void setup()
{
  beginAsipComms();
  // start the services   
  asipPixels.begin(neoPixelPin, &strip);
  asipIO.begin();  // NEW from  v1.1: core I/O service must follow all other service begin methods

  for (unsigned int i = 0; i < asipServiceCount(services); i++)
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
  delay(200); // wait for USB to init - todo  
  asip.begin(&Serial, asipServiceCount(services), services, sketchName);   
}

void loop()
{
  asip.service();
}