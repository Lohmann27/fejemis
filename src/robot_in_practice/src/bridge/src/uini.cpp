/*
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

#include <string>
//#include <time.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
// #include <readline/readline.h>
// #include <readline/history.h>
#include "uini.h"
#include "ubridge.h"
#include "uhandler.h"
// #include "userverport.h"
// #include "udata.h"
#include "udataitem.h"

UIni ini;

/**
  * constructor */
// UIni::UIni(/*UBridge * bridge*/)
// //: USource(bridge)
// {
// }

void UIni::printHelp()
{ // show help
//   printf("\nBridge between a Regbot robot and joystick and socket clients\n");
//   printf("Runs on Linux PC and raspberry (preferably in rc.local - runs here an oled display too if available).\n");
  printf("Ini commands:\n");
  printf("   load [filename] \t: load commands from ini-file\n");
  printf("   reload [interface] \t: reload commands to this interface, default is all\n");
}


void UIni::setup(const char * iniPath)
{
  snprintf(systemIniFile, SIFL, "%s/%s.ini", iniPath, "bridge");
  sourceNum = -4;
  ledIndex = 6;
  setSourceID("ini");
  handler.handleCommand(this, (char*)"# Ini-file loader loaded", true);
}



bool UIni::decode(const char* key, const char* params, USource* client)
{
  bool used = strcmp(key, sourceID) == 0;
  if (used)
  {
    const char * p1 = params;
    if (strncmp(p1, "load", 4) == 0)
    {
      p1 += 4;
      while (isspace(*p1))
        p1++;
      if (*p1 > ' ')
      { // assumed to be a filename
        printf("# Loading ini-file: '%s'\n", p1);
        loadIniFile(p1, nullptr);
      }
      else
      {
        printf("# Loading default ini-file '%s'\n", systemIniFile);
        loadIniFile(systemIniFile, nullptr);
      }
    }
    if (strncmp(p1, "reload", 6) == 0)
    { // parameter is now interface
      p1 += 6;
      while (isspace(*p1))
        p1++;
      if (*p1 > ' ')
      {
        const char * p2 = p1;
        while (*p2 > ' ')
          p2++;
        int n = p2 - p1;
        const int MNL = 32;
        char s[MNL];
        if (n < MNL)
        {
          strncpy(s, p1, n);
          s[n] = '\0';
//           printf("# UIni::decode: reloading for interface '%s'\n", s);
          loadIniFile(systemIniFile, s);
        }
        else
        {
          client->sendLock.lock();
          client->sendStr("# reload has bad interface name\n");
          client->sendLock.unlock();
        }
      }
      else
        // load all again
        loadIniFile(systemIniFile, nullptr);
    }
    else if (strncmp(p1, "help", 4) == 0)
    {
      client->sendLock.lock();
      client->sendStr("# Inifile commands\n");
      client->sendStr("#  load \tLoads from default ini-file\n");
      client->sendStr("#  load filename \tLoads from specified file name\n");
      client->sendLock.unlock();
    }
    else
      used = false;
  }
  return used;
}


void UIni::loadIniFile(const char* iniName, const char * interface)
{
  struct stat attrib;                   // create a file attribute structure
  int err = stat(iniName, &attrib);      // get the attributes of old logfile
  const int MSL = 200;
  char s[MSL];
  iniExist = (err == 0);
  if (not iniExist)
  {
    snprintf(s, MSL, "# load ini: no such file '%s'\n", iniName);
    handler.handleCommand(this, s, true);
    fprintf(stderr, "# UIni::loadIniFile %s", s);
  }
  else
  {
    FILE * ini = fopen(iniName, "r");
    if (ini == nullptr)
    {
      snprintf(s, MSL, "# load ini: failed to open '%s'\n", iniName);
      handler.handleCommand(this, s, true);
      printf("# UIni::loadIniFile %s", s);
    }
    else
    {
      char * g = fgets(s, MSL, ini);
      while (g != nullptr)
      { // ignore comments
        if (strchr(";%/", g[0]) == nullptr)
        { // this line is not a comment
          if (interface == nullptr)
            handler.handleCommand(this, g, true);
          else
          { // one interface only - must be in command string
            if (strstr(g, interface) != nullptr)
            {
              handler.handleCommand(this, g, true);
              printf("# UIni::loadIniFile for interface %s: %s", interface, g);
            }
          }
        }
        // get next line
        g = fgets(s, MSL, ini);
      }
    }
    fclose(ini);
  }
}
