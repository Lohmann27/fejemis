/***************************************************************************
 #***************************************************************************
 #*   Copyright (C) 2016-2023 by DTU
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
#include "urun.h"

/**
 * calculatin difference between two timestamps as a float - in seconds
 * \param newest is a timestamp created with gettimeofday(&timeval) of type timeval (defined in \<sys/time.h> ))
 * \param oldest is a timestamp created with gettimeofday(&timeval) of type timeval (defined in \<sys/time.h> ))
 * \returns difference in decimal seconds
 * */
// float getTimeDiff(timeval newest, timeval oldest)
// { // whole seconds
//   int32_t ds = newest.tv_sec - oldest.tv_sec;
//   int32_t du = newest.tv_usec - oldest.tv_usec; // microseconds
//   return float(ds) + float(du)/1000000.0;
// }


/** start a thread in the provided object */
// void runObj(URun * obj)
// {
//   obj->run();
// }

