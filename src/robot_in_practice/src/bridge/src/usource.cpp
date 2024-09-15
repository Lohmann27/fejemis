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


#include <iostream>
#include "usource.h"
#include "uhandler.h"
#include "ubridge.h"

USource::USource()
{
//   bridge.addSource(this);
  terr.now();
  errCnt = 0;
//   printf("# added source\n");
}


void USource::setSourceID(const char* id)
{
  strncpy(sourceID, id, MAX_ID_LENGTH);
  bridge.addSource(this);
}

void USource::sendString(const char* cmd, int /*msTimeout*/)
{ // this is the default, all is send to the console
  std::cout << cmd;
}

bool USource::decode(const char* /*key*/, const char* /*params*/, USource* /*client*/)
{
//   std::cout << "# this source has no decode function " << sourceID << "\n";
  return false;
}

void USource::sendStr(const char* cmd)
{
  sendString(cmd, timeoutMs);
}

void USource::sendDeviceDetails(USource* toClient)
{
  const int MSL = 100;
  char s[MSL];
  snprintf(s, MSL, "source %d %s:dev %d has no data yet\n", sourceNum, sourceID, isActive());
  toClient->sendStr(s);
}
