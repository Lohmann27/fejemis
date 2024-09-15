/*
 * Oled 0.9" display interface
 #***************************************************************************
 #*   Copyright (C) 2017 by DTU
 #*   jca@elektro.dtu.dk
 #*
 #*
 #* The MIT License (MIT)  https://mit-license.org/
 #*
 #* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 #* and associated documentation files (the “Software”), to deal in the Software without restriction,
 #* including without limitation the rights to use, copy, modify, merge, publish, distribute,
 #* sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
 #* is furnished to do so, subject to the following conditions:
 #*
 #* The above copyright notice and this permission notice shall be included in all copies
 #* or substantial portions of the Software.
 #*
 #* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 #* INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 #* PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
 #* FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 #* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 #* THE SOFTWARE. */

#ifndef UOLED_H
#define UOLED_H

#include <string>
#include "urun.h"
#include "utime.h"

class ArduiPi_OLED;

class UOled : public URun
{
public:
  UOled();
  ~UOled();
 /**
  * Initialize display */
 bool initDisplay();
 /**
  * Display currently allocated IP adresses */
 void displayIPs();
 /**
  * Run  motoring */
 void run();
 /**
  * Display a line on the display 
  * \param lineNumber is nine number (0..5)
  * \param lineText is text to display
  * */
 void printLine(int lineNumber, const char * lineText);
 /**
  * Clear display */
 void clear();
 /*
  * Repaint display string list */
 void redisplay();
 /**
  * display battery voltage and name 
  * \param batteryVoltage display is updated only if battery voltage has changed */
 void displayVoltageAndName(float batteryVoltage, float time, float cpuTemp);
 

public:
  /// seems an oled display to be available (on a Raspberry PI with SPI interface)
  bool displayFound;
  bool displayIP;
  
private:
  ArduiPi_OLED * display = NULL;
//   std::string hostname;
  int lastIpCnt = 0;
  static const int MHL = 100;
  char hostname[MHL];
  static const int MAX_OLED_LINES = 8;
  static const int MAX_OLED_CHARS = 21;
  char oledLines[MAX_OLED_LINES][MAX_OLED_CHARS+1];
  static const int MIPS = 7;
  char * ips[MIPS] = {NULL, NULL, NULL, NULL, NULL, NULL, NULL}; // list of IP strings
public:
  float oldBatteryVoltage = -1.0;
  float oldTemperature = -1;
  UTime oldTempTime;
  float oldRegbotTime = 0;
  float batteryTime = -100;
};

#endif
