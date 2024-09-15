/*
 * bridge respond functions to data changes
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

#include <string.h>
#include <unistd.h>

#include "udataitem.h"
#include "ubridge.h"
#include "uoled.h"
#include "uteensy.h"
// #include "udata.h"
#include "userverport.h"
#include "uhandler.h"
#include "usource.h"
#include "uhost.h"
#include "ujoy.h"

bool quitBridge = false;
bool restartBridge = false;

UBridge bridge;

void UBridge::setup() //: USource(bridge)
{
  serv = &server;
  lastHbt.now();
  setSourceID("bridge");
  sourceNum = -3;
  ledIndex = 1;
  start();
}

UBridge::~UBridge()
{
  stop();
}



void UBridge::list(USource * client)
{
  handler.items->sendStatus(client, verbose);
  serv->sendString("# Bridge data sources:\n", client);
  for (int i = 0; i < (int)sourceList.size(); i++)
  {
    const int MSL = 200;
    char s[MSL];
    snprintf(s, MSL, "#   %d   %-10s (is active = %d)\n", i, sourceList[i]->sourceID, sourceList[i]->isActive());
    serv->sendString(s, client);
  }
}


bool UBridge::decode(const char * key, const char * params, USource * client)
{
  bool used = strcmp(key, sourceID) == 0;
//   printf("# UBridge::decode, got key=%s, params=%s, used=%d\n", key, params, used);
//   if (used)
//     used = not UDataItem::reservedKeyword(params);
  if (used)
  {
    const char * p1 = params;
    if (strncmp(p1, "list", 4) == 0)
    { // list data items and connections to client
      list(client);
    }
    else if (strncmp(p1, "items", 3) == 0)
    { // list data items
      handler.items->sendAll(client);
    }
    else if (strncmp(p1, "sources", 4) == 0)
    {
      client->sendStr("%% source list: is active, source:dev, [params]* \r\n");
      for (int i = 0; i < (int)sourceList.size(); i++)
      {
        sourceList[i]->sendDeviceDetails(client);
      }
    }
    else if (strncmp(p1, "restart", 4) == 0)
    {
      restartBridge = true;
      quitBridge = true;
    }
    else if (strncmp(p1, "quit", 4) == 0)
    {
      quitBridge = true;
    }
    else if (strncmp(p1, "batlow ", 7) == 0)
    {
      p1 += 7;
      while (*p1 <= ' ' and *p1 > '\0')
        p1++;
      if (strlen(p1) > 0)
        shutDownVoltage = strtof32(p1, nullptr);
    }
    else if (strncmp(p1, "batlow ", 6) == 0)
    { // print batlow
      const int MSL = 100;
      char s[MSL];
      snprintf(s, MSL, "# shut down at battery voltage %f V\n", shutDownVoltage);
      serv->sendString(s, client);
    }
    else if (strncmp(p1, "teensy ", 7) == 0)
    { // start or stop teensy connections
      p1 += 7;
      int teensyDev = strtol(p1, (char**)&p1, 10);
      int teensyOn = strtol(p1, (char**)&p1, 10);
      if (teensyDev == 0)
      {
        teensy1.activate(teensyOn);
        printf("# UBridge::decode: setting Teensy1 to active=%d\n", teensyOn);
      }
      else if (teensyDev == 1)
      {
        teensy2.activate(teensyOn);
        printf("# UBridge::decode: setting Teensy2 to active=%d\n", teensyOn);
      }
    }
    else if (strncmp(p1, "help", 4) == 0)
    { // bridge help
      serv->sendString("# Bridge has these sub-command options:\r\n", client);
      serv->sendString("#     list     Gives a list of static information on all data items as comment lines\r\n", client);
      serv->sendString("#     items    Gives a list of static information bridge:items \r\n", client);
      serv->sendString("#     sources  Gives a list of bridge devices as comments\r\n", client);
      serv->sendString("#             format: id':dev' active device rxCnt errCnt sendCnt logRx logTx\n", client);
      serv->sendString("#     teensy D R Stop or start Teensy device D (0 or 1), R=1 start, R=0 stop\r\n", client);
      serv->sendString("#     restart Restart the bridge (implementing newly compiled version)\r\n", client);
      serv->sendString("#     quit    stop bridge\r\n", client);
      serv->sendString("#     help    This help text\r\n", client);
    }
    else
      used = false;
  }
  return used;
}

/**
 * This is a thread that handles all publish data items as needed */
void UBridge::run()
{
  const int MSL = 100;
  char s[MSL];
  while (isRunning())
  {   // update status every 2 seconds
    USource * ds = findSource("front");
    if (ds != nullptr)
    { // found front processor
      if (ds->isActive())
      { // front is active, so send status.
        for (int i = 0; i < (int)sourceList.size(); i++)
        {
          USource * ac; // = nullptr;
          ac = sourceList[i];
          int n = ac->ledIndex;
          if (n >= 0)
          {
            if (ac->isActive())
            { // make corresponding LED green
              snprintf(s, MSL, "front led %d 0 200 0\n", n);
            }
            else
            { // device is not active
              snprintf(s, MSL, "front led %d 200 0 0\n", n);
            }
            handler.handleCommand(this, s, true);
          }
        }
      }
    }
    // 
    // monitor battery voltage
    UDataItem * hbt = dataa.findData("drive", "bat");
    if (hbt != nullptr)
    {
//       printf("# bridge:run found drive:bat (battery)\n");
      //
      const char * p1 = hbt->itemParams.c_str();
      float bv = strtof(p1, nullptr);
//           printf("# bridge:run 5, battery voltage is %f V\n", bv);
      if (bv > 6.0 and bv < shutDownVoltage)
      { // battery voltage low - shut down
        handler.handleCommand(this, "drive off 10\n", true);
        printf("# bridge:run: commanding a shut down - low battery %gV\n", bv);
      }
      if (bv < 6.0)
        handler.handleCommand(this, "front led 0 0 0 120\n", true);
      else if (bv < shutDownVoltage + 0.2)
        handler.handleCommand(this, "front led 0 200 0 0\n", true);
      else if (bv < shutDownVoltage + 1.2)
        handler.handleCommand(this, "front led 0 120 120 0\n", true);
      else
        handler.handleCommand(this, "front led 0 0 120 0\n", true);
      //
      // at the same tume update the manual override ledIndex
      if (joy.manOverride)
      { // manual is green
        handler.handleCommand(this, "front led 13 0 200 0\n", true);
        handler.handleCommand(this, "front led 14 0 200 0\n", true);
      }
      else
      { // is white'ish
        handler.handleCommand(this, "front led 13 90 90 90\n", true);
        handler.handleCommand(this, "front led 14 90 90 90\n", true);
      }
    }
    //
    for (int i = 0; i < 4; i++)
    {
      usleep(500000);
      loop++;
    }
  }      
}


void UBridge::addSource(USource* source)
{
  sourceList.push_back(source);
}


bool UBridge::decodeSourceCmds(const char * key, const char * params, USource * client)
{
  bool used =  false;
//   printf("# UBridge::decodeSourceCmds 1 key='%s', params='%s', source-list size = %d\n", key, params, (int) sourceList.size());
  if (true or strcmp(key, "socket") != 0)
  {
    for (int i = 0; i < (int)sourceList.size(); i++)
    {
      used = sourceList[i]->decode(key, params, client);
      if (used)
      {
        break;
      }
    }
  }
  return used;
}

USource * UBridge::findSource(const char* id)
{
  USource * src = nullptr;
  for (int i = 0; i < (int)sourceList.size(); i++)
  { // id may end with a '\n'
    if (strncmp(sourceList[i]->sourceID, id, strlen(sourceList[i]->sourceID)) == 0)
    {
      src = sourceList[i];
      break;
    }
  }
  return src;
}

void UBridge::sendDeviceDetails(USource* toClient)
{
  const int MSL = 200;
  char s[MSL];
  snprintf(s, MSL, "source %d %s:dev %d %s:%d %d %d %g %d %d\n", sourceNum, sourceID, isActive(), serv->getHostName(), serv->getPort(), serv->getActiveClientCnt(), (int)sourceList.size(), hostip.measureCPUtemp(), 0, 0);
  toClient->sendLock.lock();
  toClient->sendStr(s);
  toClient->sendLock.unlock();
}


const char * UBridge::getLogPath()
{
  return dataa.logPath;
}

void UBridge::listSources()
{
  printf("# source list size %u\n", (int)sourceList.size());
  for (int i = 0; i < (int)sourceList.size(); i++)
  {
    printf("# source list %d, id = %s\n", i, sourceList[i]->sourceID);
  }
}

