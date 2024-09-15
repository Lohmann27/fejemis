 /* #***************************************************************************
 #*   Copyright (C) 2022-2023 by DTU
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

 
#ifndef USUBSCRIBE_H
#define USUBSCRIBE_H

#include <string>
#include "utime.h"

class USource;

class USubscribe 
{
public:
  /** client for a subscription
   * or source for a onUpdate action */
  USource * cli;
  /// last time this item was send to this client
  UTime itemSend;
  /**
   * command to send when this item is updated */
  std::string onUpdate;
  /**
   * desired priority (update interval) in seconds */
  float interval = 0;
  /**
   * all means all updates regardless of interval */
  bool all = false;
  
public:
  void sendStatus(USource * client);
  void printStatus();
  /**
   * check subscribers */
  bool tick(const char ** todo);
};

 
#endif
