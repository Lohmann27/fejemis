/*
 *
 #***************************************************************************
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

#include <string>
//#include <time.h>
#include <unistd.h>
#include <stdio.h>
#include <readline/readline.h>
#include <readline/history.h>
#include <atomic>
#include <iostream>
#include <poll.h>
#include "uconsole.h"
#include "ubridge.h"
#include "uhandler.h"
#include "userverport.h"
#include "udata.h"
#include "udataitem.h"


UConsole console;

/**
  * constructor */

void UConsole::printHelp()
{ // show help
  printf("Bridge %s commands:\n", sourceID);
  printf("   q \t: quit\n");
  printf("   s \t: status of data items etc\n");
  printf("   h \t: this help\n");
  printf("   console daemon 0/1 \t: turns off keyboard if 1 (running as daemon)\n");
  printf("   bridge help \t: help for the bridge\n");
  printf("Lines with more than one character are sent to the handler as a message.\n\n");
}


void UConsole::setup()
{
//   handler = messageHandler;
  data = &dataa;
//   server = socketServer;
  sourceNum = -5;
  setSourceID("console");
  ledIndex = 5;
  // subscribe to warnings etc
  handler.handleCommand(this, (char*)"# Console loaded", true);
  handler.handleCommand(this, (char*)":# subscribe -1", true);
//   handler.handleCommand(this, (char*)"front:# subscribe -1", true);
//   handler.handleCommand(this, (char*)"bridge:# subscribe -1", true);
//   handler.handleCommand(this, (char*)"joy:# subscribe -1", true);
//   handler.handleCommand(this, (char*)"console:# subscribe -1", true);
//   handler.handleCommand(this, (char*)"ini:# subscribe -1", true);
}

void UConsole::run(bool asDaemon)
{
  // wait for console printout to finish
  isDaemon = asDaemon; // may be changed by ini-file
  sleep(1);
  if (isDaemon)
    printf("Bridge::Main:: Running as daemon - no keyboard input\n");
  else
    printf("Main:: press enter for prompt, h for help and q\\n for quit\n");
  while (not quitBridge)
  {
    int loop;
    if (not isDaemon)
    { // wait for a 'q' from keyboard
      std::string s;
      s = getLineFromKeyboard();
      if (s.size() == 2)
      { // one character command + newline
        switch (s[0])
        {
          case 'q':
            quitBridge = true;
            break;
          case 'v':
            bridge.verbose = true;
            break;
          case 's':
            bridge.list(this);
//             data->printStatus();
            server.printStatus();
            printf("\r\n");
            break;
          case 'h':
            printHelp();
            break;
          default:
            break;
        }
      }
      else if (s.size() > 2)
      {
        if (strncmp(s.c_str(), "help", 4) == 0)
          printHelp();
        else
          handler.handleCommand(this, (char*)s.c_str(), true);
      }
    }
    usleep(100000);
    loop++;
  } // while loop
}

char * UConsole::getLineFromKeyboard()
{ // readline allocates a string and returns a pointer to it
  line = readline(">>");
  if (line == nullptr)
  { // signal interrupt -> quit
    strncpy(lineBuff,"q\n", 5);
    return lineBuff;
  }
  if (strlen(line) > 1)
  { // add to history if more than 1 character only
    add_history(line);
  }
  // add a new-line at the end and move to line buffer
  snprintf(lineBuff, MLL, "%s\n", line);
  // line need to be released
  free(line);
  // let line point to just received line
  line=lineBuff;
  return line;
}


bool UConsole::decode(const char* key, const char* params, USource* client)
{
  bool used = strcmp(key, sourceID) == 0;
//   if (used)
//     used = not UDataItem::reservedKeyword(params);
  if (used)
  {
    const char * p1 = params;
    if (strncmp(p1, "daemon", 6) == 0)
    {
      p1 += 6;
      bool d = strtol(p1, nullptr, 10);
      if (d != isDaemon)
      {
        isDaemon = not isDaemon;
        client->sendLock.lock();
        if (isDaemon)
          client->sendStr("Bridge is now running without keyboard\n");
        else
          client->sendStr("Bridge is now running with active keyboard\n");
        client->sendLock.unlock();
      }
    }
    else if (strncmp(p1, "help", 4) == 0)
    {
      client->sendLock.lock();
      client->sendStr("# Console has these command options:\r\n");
      client->sendStr("#     daemon 1/0 \tRun app with no keyboard (1 = daemon)\r\n");
      client->sendStr("#     help    \tThis help text\r\n");
      client->sendLock.unlock();
    }
  }
  return used;
}
