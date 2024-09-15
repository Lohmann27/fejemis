/*
 * Handling messages to and from clients
 * and to and from regbot
 * 
 #***************************************************************************
 #*   Copyright (C) 2017-2023 by DTU
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

#ifndef UHANDLER_H
#define UHANDLER_H

#include <string>
#include <mutex>
#include "udata.h"
#include "userverport.h"

// do not need full class specification at this point
class UTeensy;
class UJoy;
class USource;
class UBridge;
// class UOled;

class UHandler
{
public:
  /**
   * Constructor */
//   UHandler(UData * broker/*, UBridge * brg*/);
  /**
   * destructor */
//   ~UHandler();
  
  void setup();
  /**
   * add all valid message types */
  void addDataItems();
  /**
   * Handle incoming command
   * \param source is the source of the command or message
   * \param msg is a textline with the command or message
   * \param noCRC CRC is not needed for internal commands */
  void handleCommand(USource * source, const char* msg, bool noCRC);
  /**
   * Stop processing */
  void stop()
  {
    stopHandle = true;
  }
  
public:
   UData * items;
//   UBridge * bridge;
protected:
  /**
   * update database only */
  void regularDataUpdate(USource * source, const char* msg);
  
private:
  /**
   * for message handler */
  bool stopHandle = false;
  int concurrentCnt = 0;
};

extern UHandler handler;

#endif
