/*
 * asipLCD.cpp -  Arduino Services Interface Protocol (ASIP)
 * 
 * Copyright (C) 2015 Michael Margolis
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This version uses version 2 of the U8Gg library: https://github.com/olikraus/u8g2
 */


#include "asipLCD.h" 

// only boards with lots are RAM are supported
#if defined(__AVR_ATmega2560__) ||  \
    defined(__MKL26Z64__) ||defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__) || \
    defined(__SAM3X8E__) ||         \
    defined(ARDUINO_SAMD_MKR1000) || \
    defined(_VARIANT_ARDUINO_ZERO_) || \
    defined(_VARIANT_ARDUINO_101_X_) || \
    defined(ESP8266)                   // ESP8266   

#include <U8g2lib.h> // for LCD
   
  static U8G2_SH1106_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, SCL, SDA, U8X8_PIN_NONE);
  static const byte nbrTextLines = 5;
  static const byte charsPerLine = 21;
  static const byte lcdHeight    = 64;
  static const byte fontHeight   = 11;  // font ascender (cap) height
  static const byte lineSpace    = (lcdHeight- fontHeight) / 4; 
  static const byte txtTop[nbrTextLines]   = {0,lineSpace,lineSpace*2,lineSpace*3,lcdHeight-fontHeight-1};
  static const byte txtBottom[nbrTextLines] = {fontHeight,lineSpace+fontHeight,lineSpace*2+fontHeight,lineSpace*3+fontHeight,lcdHeight-1};
  
  static char textBuffer[nbrTextLines][charsPerLine+1];  
                    
 
 asipLCDClass::asipLCDClass(const char svcId, const char evtId)
  :asipServiceClass(svcId)
{
  svcName = PSTR("LCD");
}

void asipLCDClass::begin()
{
     u8g2.begin();
    // font , color 
    u8g2.setFont( u8g2_font_crox1hb_tr ); 
    clear();
}

void asipLCDClass::reset()
{

}

void asipLCDClass::reportValues(Stream * stream)
{
}

void asipLCDClass::reportValue(int sequenceId, Stream * stream)  // send the value of the given device   
{
   stream->println("LCD");
}

void asipLCDClass::processRequestMsg(Stream *stream)
{
  // request:  "L,W,line,string\n"  // top line is line 0
   int request = stream->read();  
   if( request == tag_WRITE) {
      int line = stream->parseInt();  
      if(line < nbrTextLines) {
         if(stream->find(",")) // skip past comma to get to text
         {
           char lineBuffer[charsPerLine+1];
           memset(&lineBuffer, 0, sizeof(lineBuffer));
           stream->readBytesUntil('\n', (char*)lineBuffer, charsPerLine);                    
           text(lineBuffer, line);
         }
      }
    }
    else if(request == tag_WRITE_RC){
/*
       this command is not yet supported        
        Request: write text starting from  given line and column (0 is first line and column)
          Header		Tag		Line Number		Column		Text            	Terminator
           ‘L ’ 	 ,	‘w’	 ,	digits	    ,	digits	‘	ASCII text string	‘\n’
        Example:  "L,w,0,6, again\n"    write again in 7th column of first line
*/        

      int line = stream->parseInt();  
      if(line < nbrTextLines) {
         if(stream->find(","))// skip past comma to get to text
         {
           int column =  stream->parseInt();
           if(column < charsPerLine && stream->read() == ',' ){    
              char lineBuffer[charsPerLine+1];
              memset(&lineBuffer, 0, sizeof(lineBuffer));              
              stream->readBytesUntil('\n', (char*)lineBuffer, charsPerLine - column);           
              text(lineBuffer, line, column);
           }              
         }
      }
        
    }
    else if(request == tag_GRAPH){
       int count = stream->parseInt(); 
       //Serial.printf("got graph request for %d lines\n", count);
       if(stream->find(",{")){ // skip to data
          for(int i=0; i < count; i++){
              int line = stream->parseInt();
              int val = stream->parseInt();
              hGraph(line,val);  // todo bounds checking
          }
       }       
    }   
    else if(request == tag_CLEAR) {
       clear();
    }    
}

// private methods
void asipLCDClass::text(const char *txt, int row)
{
   clearLine(row);
   text(txt,row, 0);
}

void asipLCDClass::text(const char *txt, int row, int column)
{   
    //u8g2.clearBuffer(); // the pixel buffer 
    strlcpy(&textBuffer[row][column], txt, charsPerLine-column);
    //Serial.printf("text: row = %d, column= %d, txt=%s, buff=%s\n", row, column,txt, &textBuffer[row][column]);    
    show();
}

void asipLCDClass::show()
{     
    for ( int line = 0; line < nbrTextLines; line++) {   
      u8g2.setCursor(0, txtBottom[line]);
      u8g2.print(&textBuffer[line][0]);
      //Serial.printf("show: line = %d, y = %d, %s\n", line, txtBottom[line], &textBuffer[line][0]);    
    }    
    u8g2.sendBuffer();         // transfer internal memory to the display
}

void asipLCDClass::clear()
{      
    memset(textBuffer, 0, sizeof(textBuffer)); // the text buffer
    u8g2.clearBuffer(); // the pixel buffer
    u8g2.sendBuffer();
}

void asipLCDClass::clearLine(int line)
{
    int width =  u8g2.getDisplayWidth() - 1;
    u8g2.setDrawColor(0);
    u8g2.drawBox( 0, txtTop[line], width-1, txtBottom[line]);  
    u8g2.setDrawColor(1);
    u8g2.sendBuffer();
    //Serial.printf("clearLine: x=%d, y=%d, x1=%d, y1=%d\n",0, txtTop[line], width, txtBottom[line] ); 
}

void asipLCDClass::hGraph(int line, int value)
{
   int width =  u8g2.getDisplayWidth() - 1;
   u8g2.setDrawColor(0);
   u8g2.drawBox( 0, txtTop[line], width-1, fontHeight);
   u8g2.setDrawColor(1);
   u8g2.drawBox( 0, txtTop[line]+1, map(value, 0, 100, 0, width), fontHeight-1);
   u8g2.sendBuffer();
}

// the following is not implimented in ASIP
void asipLCDClass::hGraph(char * title, int value1, int value2,int value3,int value4)
{
    int width =  u8g2.getDisplayWidth() - 1;
    u8g2.clearBuffer();
    u8g2.drawStr(0, 0, title);
    u8g2.drawBox( 0, txtTop[1]+1, map(value1, 0, 100, 0, width), fontHeight-1);
    u8g2.drawBox( 0, txtTop[2]+1, map(value2, 0, 100, 0, width), fontHeight-1);
    u8g2.drawBox( 0, txtTop[3]+1, map(value3, 0, 100, 0, width), fontHeight-1);
    u8g2.drawBox( 0, txtTop[4]+1, map(value4, 0, 100, 0, width), fontHeight-1);
    u8g2.sendBuffer();    
}

#else
#error("LCD not supported on this board, see asipLCD.h for supported boards")    
#endif
