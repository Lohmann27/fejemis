/*
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

#ifndef UHOST_H
#define UHOST_H

#include <string>
#include "urun.h"
#include "utime.h"
#include "usource.h"


class UHostIp : public URun, public USource
{
public:
  void setup();
 /**
  * Display currently allocated IP adresses */
 bool findIPs();
 /**
  * Run  motoring */
 void run();
 /**
  * Is source connection open */
 bool isActive()
 {
   return true;
 }
 float measureCPUtemp();
 

public:
  /// seems an oled display to be available (on a Raspberry PI with SPI interface)
  bool displayIP;
  int loop = 0;

private:
  
  bool check_wireless(const char* ifname, char* protocol);
  void updateIPlist();
  bool findWifiMACs();
  
  int lastIpCnt = 0;
  static const int MHL = 100;
  char hostname[MHL];
  static const int MIPS = 7;
  char ips[MIPS][MHL] = {{'\0'}}; // list of IP strings
  char macs[MIPS][MHL] = {{'\0'}}; // list of MAC strings
  int ipsCnt = 0;
  int macCnt = 0;
  static const int MHL2 = 150;
  char ip4list[MHL2];
  char ip4listLast[MHL2];
  char maclist[MHL2];
  // debug
  int loopBridge = 0;
  int loopJoy = 0;
};

extern UHostIp hostip;

#endif
