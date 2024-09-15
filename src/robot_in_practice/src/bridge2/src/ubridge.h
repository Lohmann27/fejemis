/*
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

#ifndef UBRIDGE_H
#define UBRIDGE_H

#include "vector"
#include "urun.h"
#include "utime.h"
#include "usource.h"

class UDataItem;
class UTeensy;
// class UOled;
class UData;
class UServerPort;
class UHandler;

// shutdown and restart flags
extern bool quitBridge;
extern bool restartBridge;

class UBridge : public URun, public USource
{
public:
  float shutDownVoltage = 21.0; // when battery gets below - power off is send to drive
  /**
   * constructor */
//   UBridge(UBridge * bridge);
  void setup();
  /** destructor */
  ~UBridge();
  /** get responce number 
   * searches if this data item needs some special treatment,
   * such as send to oled display or send something to robot
   * \param key is data item key,
   * \param client is the message source client number (robot is -1, joystick=-2)
   * \param sequence is to return to data item, if this datatype
   *                 is a sequence (e.g. logfile or program list
   * \returns number of response process, or -1 if no special
   *          process is needed. */
//    USource * getDestination(const char * key, USource * client, bool * sequence);
  /**
   * Response function with pointer to data item 
   * \param respondseClass is allocated response function (allocated by the getResponseNumber() function
   * \param dataItem is pointer to updated data item 
   * \param client is the source number for the data update */
  void response(USource * responseClass, UDataItem * dataItem, USource * client);
  /**
   * bridge thread loop */
  void run();
  /**
   * time for last hbt */
  UTime lastHbt;
  /**
   * Add a source possibility */
  void addSource(USource * source);
  
  bool decodeSourceCmds(const char * key, const char * params, USource * client);
  
  void setPortServer(UServerPort * portServer)
  {
    serv = portServer;
  }
  USource * findSource(const char * id);
  /**
   * Bridge is always active */
  bool isActive()
  {
    return true;
  }
  /**
   * List data iotems and data sources with status
   * all send as comments 
   * \param client is the destination for the list */
  void list(USource * client);
  
  /**
   * more printout */
  bool verbose = false;
  int loop = 0;
  /**
   * Set link to data */
//   void setData(UData * dataLink)
//   {
//     data = dataLink;
//   }
  
  const char * getLogPath();
  
  void listSources();
  
private:
  /**
   * information from socket server */
  void responceRobotID(UDataItem* dataItem, int client);
  /** measure CPU temperature
   * */
//   float measureCPUtemp();
  /**
   * is the message for this device */
  bool decode(const char * key, const char * params, USource * client);
  /**
   * den status */
  void sendDeviceDetails(USource* toClient);
  
private:
  // link to other parts of bridge
//   UData * data;
  UServerPort * serv = nullptr;
  //
  std::vector<USource *> sourceList;
  /*
   * CPU temperature in degrees C */
  float cputemp;
};

extern UBridge bridge;

#endif
