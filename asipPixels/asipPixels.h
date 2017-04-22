/*
 * asipPixels.h -  Arduino Services Interface Protocol (ASIP)
 * Service to set one or more RGB  LEDs, such as WS2812 (neopixels)
 * LEDs are identified by a sequence position number, the first pixel is position 0. 
 * Colors are the RGB values expressed as hex digits.
 * 
 * Copyright (C) 2015 Michael Margolis
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */


#ifndef asipPixels_h
#define asipPixels_h

#include "asip.h"
#include <Adafruit_NeoPixel.h>

// Service and method defines
// Service IDs must be unique across all services
// Method and event IDs must be unique within a service

//IDs already used by the core mirtle services:  I,M,E,B,R

// RGB Pixel service
const char id_PIXELS_SERVICE      = 'P';
// methods
const char tag_SET_PIXELS             = 'P';  // 32 bit packed color value
const char tag_SET_PIXELS_RGB         = 'p';  // colon seperated rgb values
const char tag_SET_PIXEL_SEQUENCE     = 'S';
const char tag_SET_PIXEL_SEQUENCE_RGB = 's';
const char tag_SET_BRIGHTNESS         = 'B';
const char tag_GET_NUMBER_PIXELS      = 'I';
const char tag_CLEAR_ALL_PIXELS       = 'C';

// Colors are the RGB  8 bit values packed into a 32 bit integer as follows:
//      Color = r <<16  +  g <<8  +  b
#define RED_PIXEL   16711680 // Red
#define GREEN_PIXEL    65280 // green
#define BLUE_PIXEL       255 // blue 


// friendly defines for default strip configuration
#define defaultStripType (NEO_RGB + NEO_KHZ800)

class asipPixelsClass : public asipServiceClass
{  
public:
   asipPixelsClass(const char svcId, const char evtId);
   void begin(byte pin, Adafruit_NeoPixel* strip);
   void begin(byte nbrElements, const pinArray_t pins[], Adafruit_NeoPixel* strips);
   void reset();
   void reportValues(Stream * stream);
   void reportValue(int sequenceId, Stream * stream) ; // not used in this service  
   void processRequestMsg(Stream *stream);  
   void setPixelColor(int pixel, uint32_t color);                 // enable calling from sketch
   void setPixelColor(int pixel, uint8_t r, uint8_t g, uint8_t b); // enable calling from sketch
private:   
   Adafruit_NeoPixel* stripPtr;
   uint32_t parseRGB(Stream *stream);

};   

//extern asipPixelsClass asipPixels;

#endif
 
   


