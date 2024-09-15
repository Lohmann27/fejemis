/* #***************************************************************************
 #*   Copyright (C) 2017-2023 by DTU
 #*   jcan@dtu.dk
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


#ifndef UREGBOT_H
#define UREGBOT_H

#include <sys/time.h>
#include <mutex>
#include <queue>

#include "urun.h"
#include "utime.h"
#include "ulogfile.h"
#include "tcpCase.h"
#include "usource.h"

class UHandler;


using namespace std;


class UOutQueue
{
public:
  static const int MML = 400;
  char msg[MML];
  int len;
  bool isSend = false;
  UTime queuedAt;
  UTime sendAt;
  int resendCnt;
  /**
   * Constructor */
  UOutQueue(const char * crc, const char * msg)
  {
    setMessage(crc, msg);
    queuedAt.now();
    isSend = false;
    resendCnt = 0;
  }
  /**
   * set new message */
  bool setMessage(const char * crc, const char * message)
  {
    strncpy(msg, crc, 3);
    len = strnlen(message, MML);
    bool isOK = len + 4 < MML;
    if (isOK)
    {
      strncpy(&msg[3], message, len);
      len += 3;
      if (msg[len-1] != '\n')
      { // add a \n if it is not there
        msg[len] = '\n';
        msg[++len] = '\0';
      }
    }
    else
      printf("# UTeensy::UOutQueue::setMessage: messages longer than %d chars are not allowed! '%s'\n", MML, message);
    return isOK;
  }
  /**
   * Confirm a match */
  bool compare(const char * got)
  { // ignore potential \n
    int n = strlen(got) - 1;
    bool equal = strncmp(&msg[3], got, n) == 0;
    return equal;
  }
};


/**
 * The robot class handles the 
 * port to the REGBOT part of the robot,
 * REGBOT handles the most real time issues
 * and this class is the interface to that. */
class UTeensy : public URun, public USource
{ // REGBOT interface
public:
  /// Is port opened successfully
  bool teensyConnectionOpen;
  // mission state from hbt 
  int missionState = 0;
  
  
private:
  // serial port handle
  int usbport;
  // serial port (USB)
  int usbdeviceNum = 0;
  // simulator hostname
  const char * simHost;
  // simulator port
  int simPort = 0;
  // mutex to ensure commands to regbot is not mixed
//   mutex txLock;
//   mutex logMtx;
  mutex eventUpdate;
  // receive buffer
  static const int MAX_RX_CNT = 1000;
  char rx[MAX_RX_CNT];
  // number of characters in rx buffer
  int rxCnt;
  //
  UTime lastTxTime;
  UTime justConnectedTime;
  // socket to simulator
  tcpCase socket;
  /**
   * communication count */
  int gotCnt = 0;
  int sendCnt = 0;
  /** interface just opened */
  bool justConnected = false;
  bool confirmSend = false;
  
public:
  /** constructor */
//     UTeensy(/*UBridge * bridge*/);
  /** destructor */
    ~UTeensy();
  /**
   * Set device */
  void setup(int usbDevNum, int simport, char * simhost, const char * id);
  
  /**
   * send device details to client */
  virtual void sendDeviceDetails(USource * toClient);
  /**
   * send a string to the serial port 
   * But wait no longer than timeout - the unsend part is skipped 
   * \param message is c_string to send,
   * \param timeoutMs is number of ms to wait at maximum */
  void sendString(const char * message, int timeoutMs);
  /**
   * runs the receive thread 
   * This run() function is called in a thread after a start() call.
   * This function will not return until the thread is stopped. */
  void run();
  /**
   * Init data types to and from robot */
  void initMessageTypes();
  /**
   * open log with communication with robot */
  void openCommLog(const char * path);
  /** close logfile */    
  void closeCommLog();
  /**
   * decode commands potentially for this device */
  bool decode(const char * key, const char * params, USource * client);
  /**
   * is data source active (is device open) */
  virtual bool isActive()
  {
    return (usbport >= 0 or socket.connected) and gotActivityRecently and not justConnected and shouldBeActive;
  }
  
  void activate(bool toActive);
  void activate()
  {
    activate(shouldBeActive);
  };
  
private:
  /**
   * Open the connection.
   * \returns true if successful */
  bool openToTeensy();
  
  const char * getTeensyDeviceName(int devNum);
  /**
   * generate new host rename script */
  void saveRobotName(const char * newName);
  /**
   * A confirm message is received,
   * Check, and
   * release the next in the queue */
  void messageConfirmed(const char * confirm);
  void closeUSB();
  /**
   * Logvile */
  ULogFile * botComTx;
  ULogFile * botComRx;
  //   mutex logMtx;
  int connectErrCnt = 0;
  ///
  bool gotActivityRecently = true;
  UTime lastRxTime;
  static const int MAX_USB_DEVS = 5;
  static int usbDevIsOpen[MAX_USB_DEVS];
  static const int MAX_DEV_NAME_LENGTH = 20;
  static mutex usbDevOpenList;
  char usbDevName[MAX_DEV_NAME_LENGTH];
  bool shouldBeActive = true;
  bool initialized = false;
  /**
   * uotgoing message queue */
  std::queue<UOutQueue> outQueue;
  float confirmTimeout = 0.05;
  int confirmMismatchCnt = 0;
  int confirmRetryCnt = 0;
  int confirmRetryDump = 0;
};

extern UTeensy teensy1;
extern UTeensy teensy2;

#endif
