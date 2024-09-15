/*
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

#include <iostream>
#include <sys/time.h>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <thread>
#include <mutex>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <readline/readline.h>
#include <readline/history.h>
// local files
#include "userverport.h"
#include "ujoy.h"
#include "uhandler.h"
// #include "uoled.h"
#include "ubridge.h"
#include "uconsole.h"
#include "uini.h"
#include "uhost.h"
#include "uros2.h"

using namespace std;

char ** argvs;
int argcs;



////////////////////////////////////////////////////////////////////



void restart()
{
  printf("# Going to reboot ...\r\n\n\n\n");
  execve(argvs[0], argvs, NULL);
  printf("# Never here - this process should be overwritten by now.\n");
}



bool readCommandLineParameters(int argc, char ** argv, 
                               bool * deamon, 
                               int * port,
                               int * usbdev, int usbdevSize,
                               char jsdev[], int jsdevSize,
                               char simhost[], int simhostSize,
                               int * simport,
                               int * usbdevFront,
                               char filepath[], int filepathsize
)
{
  // are there mission parameters
  bool isHelp = false;
  *deamon = false;
  // default values
  *usbdev = 0;
  *usbdevFront = 1;
  strncpy(jsdev, "/dev/input/js0", usbdevSize);
  strncpy(simhost, "localhost", simhostSize);
//   for (int i = 0; i < argc; i++)
//   {
//     fprintf(stderr,"# %d %s\n", i, argv[i]);
//   }
  for (int i = 1; i < argc; i++)
  {
    if (isHelp)
      break;
    char * c = argv[i];
    if (c[0] == '-')
    {
      if (c[1] == '-')
        c++;
      switch (c[1])
      {
        case 'h':
          isHelp = true;
          break;
        case 'd':
          *deamon = true;
          break;
        case 'p':
          *port = strtol(argv[++i], NULL, 10);
          break;
        case 'i':
          *simport = strtol(argv[++i], NULL, 10);
          break;
        case 'u':
          *usbdev = strtol(argv[++i], nullptr, 10);
          break;
        case 'f':
          *usbdevFront = strtol(argv[++i], nullptr, 10);
          break;
        case 'l':
          strncpy(filepath, argv[++i], filepathsize);
          break;
        case 's':
          strncpy(simhost, argv[++i], simhostSize);
          break;
        case 'j':
          strncpy(jsdev, argv[++i], jsdevSize);
          break;
        default:
          printf("# Unused parameter '%s'\n", argv[i]);
          break;
      }
    }
    else
      printf("Parameter option should start with a '-', not '%s'\n", c);
  }
  // print config
  if (not isHelp)
  { // mostly debug
    printf("\n# Using the following parameters:\n");
    printf("# usbdev1 = /dev/ttyACM%d\n", *usbdev);
    printf("# usbdev2 = /dev/ttyACM%d\n", *usbdevFront);
    printf("# port    = %d\n", *port);
    printf("# gamepad = %s\n", jsdev);
    printf("# simhost = %s\n", simhost);
    printf("# simport = %d\n", *simport);
    printf("# daemon  = %d\n", *deamon);
    printf("# log+ini = %s\n", filepath);
    printf("# ------------------------\n");
  }
  return isHelp;
}


int main ( int argc,char **argv ) 
{ // save application name for restart
  argvs = argv;
  argcs = argc;
  bool asDaemon = false;
  bool help = false;
  const int MPL = 100;
  int usbDev;
  int usbDevFront;
  char simHost[MPL];
  char jsDev[MPL];
  int port = 24001;
  int simport = 0;
  const int MFPL = 200;
  char filepath[MFPL];
  snprintf(filepath, MFPL, "%s/src/bridge/launch", std::getenv("PWD"));
  help = readCommandLineParameters(argc, argv, &asDaemon, &port, &usbDev, MPL, jsDev, MPL, simHost, MPL, &simport, &usbDevFront, filepath, MFPL);
  if (port == simport)
  {
    printf("#### Port can't be the same for simulator input %d and output %d!\n", simport, port);
    help = true;
  }
  if (help)
  { // command line help
    printf("#  \n");
    printf("# Bridge from a REGBOT to a socket connection\n");
    printf("# parameters:\n");
    printf("#  -h     : this help\n");
    printf("#  -d     : run as daemon - without listening to keyboard, default is %d\n", asDaemon);
    printf("#  -p N   : set output port number to N (default is %d)\n", port);
    printf("#  -i N   : set input  port number for simulator REGBOT, default is %d (use USB)\n", simport);
    printf("#  -s str : set input host name/IP for simulated REGBOT, default is '%s'\n", simHost);
    printf("#  -u str : set USB device to drive Teensy, default is '/dev/ttyACM%d'\n", usbDev);
    printf("#  -f str : set USB device to front Teensy, default is '/dev/ttyACM%d'\n", usbDevFront);
    printf("#  -j str : set joystick device name, default is '%s'\n", jsDev);
    printf("#  -l str : set set path for ini-file and logfiles, is '%s'\n", filepath);
    printf("#  \n");
  }
  else
  { // not help 
    handler.setup();
    ini.setup(filepath);
    dataa.setup(filepath);
    server.setup(port);
    bridge.setup();
    console.setup();
    joy.setup(jsDev);
    rosif.setup();
    hostip.setup();
    teensy1.setup(usbDev, simport, simHost, "ttyACM0");
    teensy2.setup(usbDev, simport, simHost, "ttyACM1");
    //
    // give names to some common data items
    handler.handleCommand(&console, (char*)"ini:# name Text message from ini-file loader", true);
//     handler.handleCommand(&console, (char*)"drive:# name Text message from drive interface", true);
//     handler.handleCommand(&console, (char*)"front:# name Text message from front interface", true);
    handler.handleCommand(&console, (char*)"bridge:# name Text message from bridge interface", true);
    handler.handleCommand(&console, (char*)"gamepad:# name Text message from gamepad interface", true);
    handler.handleCommand(&console, (char*)"console:# name Text message from console interface", true);
    //
    // load commands from ini-file
    ini.loadSystemIni();
    if (rosif.isActive())
      rosif.start();
    // Open connection if set to active
    teensy1.activate();
    teensy2.activate();
    // main loop
    sleep(2);
    console.run(asDaemon);
    //
    printf("Stopping ...\n");
    hostip.stop();  // stop checking for IP change
    joy.stop();     // delete joystick interface
    handler.stop(); // stop handling messages
    server.stop();  // delete socket server
    teensy1.stop();   // delete Teensy interface
    teensy2.stop();   // delete Teensy interface
    bridge.stop();  // stop the rest (database, items and subscriptions)
  }
  printf("Finished\n");
  if (restartBridge)
  {
    printf("Will restart in a moment\n");
    restart();
  }
}

