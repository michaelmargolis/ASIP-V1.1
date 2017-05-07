/*
 * asipPixels.cpp -  Arduino Services Interface Protocol (ASIP)
 * 
 * Copyright (C) 2015 Michael Margolis
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *  
 */

#include "asipPixels.h"

const byte DEFAULT_BRIGHTNESS = 64;

 
 asipPixelsClass::asipPixelsClass(const char svcId, const char evtId)
  :asipServiceClass(svcId)
{
  svcName = PSTR("Pixels");
}

void asipPixelsClass::begin(byte pin, Adafruit_NeoPixel* strip)
{
    stripPtr = strip; 
    strip->begin();  // initialize the NeoPixel library.
    strip->setBrightness(DEFAULT_BRIGHTNESS); // 1/4 brightness  
    pinMode(pin, OUTPUT);
    asipServiceClass::begin(pin); // register the pin 
}

void asipPixelsClass::begin(byte nbrElements, const pinArray_t pins[], Adafruit_NeoPixel* strips)
{
  //each strip uses one pin so pinCount equals nbrElements 
  stripPtr = strips; 
  for(int i=0; i < nbrElements; i++) {     
    strips[i].begin();  // initialize the NeoPixel library.
    pinMode(pins[i], OUTPUT);
    strips[i].setBrightness(DEFAULT_BRIGHTNESS); // 1/4 brightness
  }
  asipServiceClass::begin(nbrElements,nbrElements,pins);
}
   
void asipPixelsClass::reset()
{
  
}

void asipPixelsClass::reportValues(Stream * stream)
{
}

void asipPixelsClass::reportValue(int sequenceId, Stream * stream)  // send the value of the given device   
{
   stream->println("Pixels");
}

void asipPixelsClass::processRequestMsg(Stream *stream)
{
   int request = stream->read();  
   //stream->print("processing request "); stream->write(request); stream->println();
   if( request == tag_SET_PIXELS) 
   {       
       //Comma separated pairs of colon separated pixel positions and color values
      // request:  "P,P,strip,count,{pixel:color,...}\n"  // first pixel is 0        
      int stripIndex = stream->parseInt();        // todo - validate stripIndex range
      int count = stream->parseInt();  
      debug_printf("got pixel msg for %d pixels on strip %d\n", count, stripIndex);
      if( stream->read() == ',' && stream->read() == '{') // skip to start of parms
      {           
        // todo - check to ensure  count, data pairs, and closing bracket are consistent
        while(count--)
        {
           int pixel = stream->parseInt();
           unsigned long color = stream->parseInt();
           debug_printf("set pixel %d to %d\n", pixel, color);           
           stripPtr[stripIndex].setPixelColor(pixel, color);
           setPixelColor(pixel, color);
        }
        stripPtr[stripIndex].show();        
      }
    } 
    else if( request == tag_SET_PIXELS_RGB) 
    {
     //Comma separated pairs of colon separated pixel positions and RGB color values
      // request:  "P,p,strip,count,{pixel:R:G:B,...}\n"  // first pixel is 0   
      int stripIndex = stream->parseInt(); 
      int count = stream->parseInt();  
      debug_printf("got pixel msg for %d pixels on strip %d\n", count, stripIndex);
      if( stream->read() == ',' && stream->read() == '{') // skip to start of parms
      {           
        // todo - check to ensure  count, data pairs, and closing bracket are consistent
        while(count--)
        {
           int pixel = stream->parseInt();
           uint32_t color = parseRGB(stream);           
           stripPtr[stripIndex].setPixelColor(pixel, color);          
           // todo check if count > 0 then next char must be comma 
        }
        stripPtr[stripIndex].show();       
      }
    }    
    
    else if(request == tag_SET_PIXEL_SEQUENCE)
    {
    // Comma separated hex color values for pixels starting from position given in the firstPixel field
     // request:  "P,S,strip,count,firstPixel,{color,color,...}\n"  
      int stripIndex = stream->parseInt(); 
      int count = stream->parseInt();  
      int firstPixel = stream->parseInt();  
      debug_printf("pixel sequence msg for %d pixels starting from %d\n", count, firstPixel);
      if(stream->read() == ',' && stream->read() == '{') // skip to start of parms
      {           
        // todo - check to ensure  count, data pairs, and closing bracket are consistent
        while(count--)
        {          
           unsigned long color = stream->parseInt();        
           stripPtr[stripIndex].setPixelColor(firstPixel++, color);
        }
        stripPtr[stripIndex].show();        
      }
      else {
        debug_printf("Pixel Sequence msg missing comma or open bracket\n");
      }      
       
    }
    else if(request == tag_SET_PIXEL_SEQUENCE_RGB)
    {
      int stripIndex = stream->parseInt(); 
      int count = stream->parseInt();  
      int firstPixel = stream->parseInt();  
      debug_printf("pixel sequence msg for %d pixels starting from %d\n", count, firstPixel);
      if(stream->read() == ',' && stream->read() == '{') // skip to start of parms
      {           
        // todo - check to ensure  count, data pairs, and closing bracket are consistent
        while(count--)
        {          
           uint32_t color = parseRGB(stream);    
           stripPtr[stripIndex].setPixelColor(firstPixel++, color);
        }
        stripPtr[stripIndex].show();        
      }
      else {
        debug_printf("Pixel Sequence msg missing comma or open bracket\n");
      }     
    }
    else if(request == tag_SET_BRIGHTNESS)
    {
        int stripIndex = stream->parseInt(); 
        int brightness = stream->parseInt();
        // sets overall strip brightness, 255 is max
        stripPtr[stripIndex].setBrightness(brightness);
        stripPtr[stripIndex].show();  
    }
    else if(request == tag_GET_NUMBER_PIXELS)
    {
       // request:  "P,I\n"  
       // reply:  @P,I,count\n"          
       int stripIndex = stream->parseInt(); 
       int count = stripPtr[stripIndex].numPixels();
       stream->write(EVENT_HEADER);
       stream->write(id_PIXELS_SERVICE);
       stream->write(',');  
       stream->write(tag_GET_NUMBER_PIXELS);
       stream->write(',');
       stream->print(stripIndex);
       stream->write(',');
       stream->print(count); // actual connected pixels may be less
       stream->write(MSG_TERMINATOR);        
    }
    else if(request == tag_CLEAR_ALL_PIXELS)
    {
      int stripIndex = stream->parseInt(); 
      stripPtr[stripIndex].clear(); 
      stripPtr[stripIndex].show();      
    }     
    else
    {
      reportError(ServiceId, request, ERR_UNKNOWN_REQUEST, stream);
    }
    while( stream->available() && stream->read() != '\n') 
         ; // skip to end of line
}

//returns a packed 32 bit color value from 3 colon seperated stream fields
// in this version, any non digit seperator is valid
uint32_t asipPixelsClass::parseRGB(Stream *stream)
{
   uint8_t R = stream->parseInt();   
   uint8_t G = stream->parseInt();
   uint8_t B = stream->parseInt();
   // packed format is always RGB   
   return stripPtr[0].Color(R,G,B); 
}

// enable calling from sketch to control strip 0
void asipPixelsClass::setPixelColor(int pixel, uint32_t color) 
{ 
  debug_printf("in func, set pixel %d to hex %x\n", pixel, color);
  stripPtr[0].setPixelColor(pixel, color);  
  stripPtr[0].show();    
}

void asipPixelsClass::setPixelColor(int pixel, uint8_t r, uint8_t g, uint8_t b) // enable calling from sketch
{
  debug_printf("in func, set pixel %d to %d,%d,%d\n", pixel, r,g,b);
  stripPtr[0].setPixelColor(pixel, r,g,b);  
  stripPtr[0].show();    
}



