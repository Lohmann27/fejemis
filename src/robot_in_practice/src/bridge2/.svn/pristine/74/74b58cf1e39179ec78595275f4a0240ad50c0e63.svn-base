/*
 * Data storage (message storage)
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

#ifndef UDATA_H
#define UDATA_H

#include <string>
#include <mutex>

#include "urun.h"
#include "userverport.h"
#include "userverclient.h"
#include "ulogfile.h"

//#define MAX_PRIORITY 5

// maximum allowed message number
#define MAX_MESSAGE_COUNT 500

//#define MAX_ITEM_KEY_LENGTH 5

class UDataItem;
class UBridge;

class UData : public URun
{
public:
  /**
   * constructor */
//   UData(UServerPort * port, char filepath[]);
//   /**
//    * destructor */
  ~UData();
  
  void setup(char filepath[]);
  /**
   * new message for a data item from this client (source of this message)
   * \param key ID for this message,
   * \param params string with remaining parameters
   * \param source is the source for the message
   * */
  void updateItem(const char * key, const char * params, USource * source, USource * explicitSource, bool anySource);
//   void regularData(USource* source, const char* skey, const char* msg, USource* dataSrc);
  
  void setItem(int index, std::string itemName, const char * itemKey);
  /**
   * print data status to console */
  void printStatus();
  /**
   * send same status to a client */
  void sendStatus(USource * client, bool verbose);
  /**
   * send all data items to a client */
  void sendAll(USource * client);
  /**
   * get number of items in database */
  inline int getItemCnt()
  {
    return itemsCnt;
  }
  /**
   * set pointer to responce handler */
//   void setHandler(UHandler * messageHandler, UBridge * responsHandler)
//   {
//     handler = messageHandler;
//     bridge = responsHandler;
//   }
  /**
   * stop subscription from this client */
  void stopSubscription(int client);

  char logPath[MAX_FILENAME_SIZE];
  /**
   * Find a data item (see also findDataItem() that returns the data index)
   * \param dataSource if not nullptr, then only for this source
   * \param item the item keyword to look for.
   * \returns nullptr if not found */
  UDataItem * findData(const char * dataSource, const char * item);
  
private:
  /**
   * pointer to socket server - to get client connection */
  UServerPort* portServer;
  
  UDataItem * items[MAX_MESSAGE_COUNT];
  int itemsCnt = 0;
  std::mutex findDataItemLock;
  /**
   * Find a data item in the item database (see also findData() that returns a pointer)
   * \param item is the item key
   * \param requestedDataSource is the source for the data item we are looking for, NULL is any source.
   * \param firstItem is the index to start data item (used if any source)
   * \returns index to the item, or -1 if no match */
  int findDataItem(const char * item, USource * requestedDataSource, int firstItem);
  UBridge * bridge;
  UHandler * handler;
  UTime tStart;
};

extern UData dataa;

#endif
