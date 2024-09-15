 /*
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


#ifndef URUN_H
#define URUN_H
 
#include <stdio.h>
#include <thread>
 
/**
 * calculatin difference between two timestamps as a float - in seconds
 * \param newest is a timestamp created with gettimeofday(&timeval) of type timeval (defined in \<sys/time.h> ))
 * \param oldest is a timestamp created with gettimeofday(&timeval) of type timeval (defined in \<sys/time.h> ))
 * \returns difference in decimal seconds
 * */
// float getTimeDiff(timeval newest, timeval oldest);

inline int mini(int a, int b)
{
  if (a < b)
    return a;
  else
    return b;
}

inline int maxi(int a, int b)
{
  if (a > b)
    return a;
  else
    return b;
}


class URun;

/** start a thread in the provided object */
void runObj(URun * obj);

/**
 * Base class, that makes it easier to starta thread
 * from the method runObj
 * The run method should be overwritten in the real class */
class URun
{
public:
  URun()
  {
    th1 = nullptr;
    th1stop = true;
  }
  
  ////////////////////////////////////////////////
  
  virtual ~URun()
  {
//     printf("instance of run stopping\n");
    stop();
  }
  
  ////////////////////////////////////////////////

  static void runObj(URun * obj)
  { // called, when thread is started
    // transfer to the class run() function.
    obj->run();
    // call stop to ensure that start can be called again
    // in case the run loop has stopped by user code
    if (obj->isRunning())
      obj->stop();
  }
  
  ////////////////////////////////////////////////

  virtual void run()
  {
    printf("Should not show\n");
  }
  
  ////////////////////////////////////////////////
  /**
   * Start the thread and run until thStop is set */
  
  bool start()
  {
    th1stop = false;
    if (th1 == nullptr)
      th1 = new std::thread(runObj, this);
    return th1 != nullptr;
  }
  
  virtual void stop()
  {
    th1stop = true;
    if (th1 != nullptr)
    {
      if (th1->joinable())
        th1->join();
      delete th1;
      th1 = nullptr;
    }
  }
  
  bool isRunning()
  {
    return not th1stop;
  };
  
protected:
  // read thread handle
  std::thread * th1;
  // set true to stop thread
  bool th1stop;
};


#endif
