/*
 *
 #***************************************************************************
 #*   Copyright (C) 2023 by DTU
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

#ifndef UCONSOLE_H
#define UCONSOLE_H

#include "utime.h"
#include "usource.h"

class UHandler;
class UBridge;
class UServerPort;
class UData;


class UConsole : public USource
{
public:
  void setup();
  /**
   * is console active, */
  bool isActive()
  {
    return not isDaemon;
  }
  void run(bool asDaemon);
protected:
  void printHelp();
  char * getLineFromKeyboard();
  bool decode(const char * key, const char * params, USource * client);

private:
  static const int MLL = 300;
  char lineBuff[MLL];
  char * line = lineBuff;
  UData * data;
  bool isDaemon = true;
};

extern UConsole console;

#endif
