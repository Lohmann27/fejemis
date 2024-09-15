/*
 * 
 ***************************************************************************
 *   Copyright (C) 2022 by DTU (Christian Andersen)                        *
 *   jca@elektro.dtu.dk                                                    *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Lesser General Public License as        *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Lesser General Public License for more details.                   *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this program; if not, write to the                 *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/


#ifndef UROS_H
#define UROS_H

#include "urun.h"
#include "utime.h"
#include "ulogfile.h"
#include "usource.h"
#include <memory>

//#define USE_ROS2
#ifdef USE_ROS2
#include "rclcpp/rclcpp.hpp"
#endif

class URosPubSub;

class URos : public URun, public USource
{
public:
  /**
   * Initialize interface and connect to bridge */
  void setup();
  /**
   * ROS thread to handle ros messages - like spinOnce */
  void run() override;
  void stopRos();
  /**
   * Is interface active */
  bool isActive() override;

  static void onRosShutdown();

  std::shared_ptr<URosPubSub> pubsub;

protected:
  /**
   * This is the entry point for messages to ros */
  void sendString(const char * message, int msTimeout) override;
  
  /**
   * publish pose */
  void publishPose(const char * msg);
  /**
   * publish IR */
  void publishIr(const char * msg);

  int n = 0;
  double x, x2, y, y2, h, h2;
  bool poseNew = true;
  double dt;
  double t1, t2;
  bool rosOK;
public:
  bool rosValid = false;
  std::mutex waitForRos;

};

extern URos rosif;

#endif
