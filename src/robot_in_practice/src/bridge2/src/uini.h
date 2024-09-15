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

#ifndef UINI_H
#define UINI_H

#include "utime.h"
#include "usource.h"

class UHandler;
class UBridge;


class UIni : public USource
{
public:
  /**
   * initialize and set path for ini-file and logfiles
   * \param iniPath is the path where bridge.ini is found, and where logfiles are stored
   * */
void setup(const char * iniPath);
  /**
   * is console active, */
  bool isActive()
  {
    return iniExist;
  }
  /**
   * Load the ini-file with this name
   * \param name is the filename */
  void loadSystemIni()
  {
    loadIniFile(systemIniFile, nullptr);
  }
  /**
   * Reload the ini-file for this interface
   * \param is is the interface name */
  void reloadIni(const char * id)
  {
    loadIniFile(systemIniFile, id);
  }
protected:
  void printHelp();
  /**
   * Decode command
   * \param key is the first keyword of the command
   * \param params is the rest of the line
   * \returns true if used */
  bool decode(const char * key, const char * params, USource * client);
  /**
   * Load commands from an ini-file
   * \param iniName is the filename to open.
   * \param interface if NULL, then all lines are used, else interfaces with this name only
   * */
  void loadIniFile(const char * iniName, const char * interface);
private:
  bool iniExist = false;
  static const int SIFL = 300;
  char systemIniFile[SIFL];
};

extern UIni ini;

#endif
