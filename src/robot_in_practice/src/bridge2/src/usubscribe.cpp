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

 
#include "usubscribe.h"
#include "usource.h"
#include "utime.h"
#include "stdio.h"

void USubscribe::sendStatus(USource * client)
{
  const int MSL = 200;
  char s[MSL];
  snprintf(s, MSL, "#                   subscriber %d %s interval %.3f, all %d\r\n", cli->sourceNum, cli->sourceID, interval, all);
  client->sendLock.lock();
  client->sendStr(s);
  client->sendLock.unlock();
}

void USubscribe::printStatus()
{
  printf("                                  subscriber %d %s interval %.3f, all %d\r\n", cli->sourceNum, cli->sourceID, interval, all);
}

bool USubscribe::tick(const char ** todo)
{
  bool isTime;
  if (all)
    isTime = true;
  else
  {
    float dt = itemSend.getTimePassed();
  //   printf("# USubscribe::tick\n");
    isTime = dt > interval;
  }
  if (isTime)
  { // send or action taken time
    itemSend.now();
  }
  if (onUpdate.size() > 0)
  {
    printf("# USubscribe::tick: an onUpdate tick: '%s'\n", onUpdate.c_str());
    *todo = onUpdate.c_str();
  }
  return isTime;
}
