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

#ifndef USOURCE_H
#define USOURCE_H

#include "mutex"
#include "utime.h"

class UHandler;
class UBridge;


class USource
{
public:
  USource(/*UBridge * bridge*/);
  /**
   * source ID used to explain to message receiver from this client */
  static const int MAX_ID_LENGTH = 32;
  char sourceID[MAX_ID_LENGTH];
  /**
   * source ID used as index to client name as source */
  int sourceNum = -1;
  int ledIndex = -1;
  /**
   * error count */
  int errCnt;
  UTime terr;
  std::mutex sendLock;
  /**
   * send error count
   * - stop subscriptions if too high */
  int sendErrCnt = 0;
  const int sendErrCntMax = 10;
  /**
   * Set source name */
  void setSourceID(const char * id);
  /**
   * is the message for this device */
  virtual bool decode(const char * key, const char * params, USource * client);
  /**
   * send device details */
  virtual void sendDeviceDetails(USource * toClient);
  /**
   * is data source active (is device open) */
  virtual bool isActive()
  {
    return false;
  }
  /**
   * Send this string to the associated connection
   * \param cmd is string to send
   * \param msTimeout is timeout, if string can not be send */
  void sendStr(const char * cmd);
  /**
   * Send this string to the associated connection
   * \param key is keyword for the message
   * \param params is the parameters to send after the keyword
   * \param msTimeout is timeout, if string can not be send */
//   void sendMsg(const char * key, const char * params);

protected:
  /**
   * pointer to handler of all messages */
//   UHandler* handler = nullptr;
  /**
   * timeout in ms for tx */
  int timeoutMs = 50;

private:
  /**
   * Send this string to the associated connection
   * \param cmd is string to send
   * \param msTimeout is timeout, if string can not be send */
  virtual void sendString(const char * cmd, int msTimeout);
};
#endif
