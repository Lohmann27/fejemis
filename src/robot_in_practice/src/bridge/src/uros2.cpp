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

#include <string.h>
#include <unistd.h>
#include <csignal>
#include "uros2.h"
#include "main.h"
#include "ubridge.h"

URos rosif;
#define USE_ROS2

#ifdef USE_ROS2

// #include "rclcpp/rclcpp.hpp" // in the header file
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
// #include "teensy_bridge_messages/msg/ir_dist.hpp"
/**
 * ROS2 node class
 * - only included if the USE_ROS2 is defined. */
class URosPubSub : public rclcpp::Node
{
public:
  // constructor
  URosPubSub()
    : Node("bridge")
  {
    hash_data_ = create_publisher<std_msgs::msg::String>("hash_data", 4);
//     ir_range_ = create_publisher<teensy_bridge_messages::msg::IrDist>("ir_range", 4);
    pose_ = create_publisher<geometry_msgs::msg::Pose>("teensy_pose", 4);
    //
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "Base parameter interface to Teensy";
    this->declare_parameter("param_line", "key + params", param_desc);
  }
  /** publish function for IR data
   * \param msg is the parameters to the ir message
   * the values are IR distance for IR1 and ir 2 */
  void pub_hash_msg(const char* msg)
  {
    auto hhm = std_msgs::msg::String();
    // start at first parameter
    hhm.data = msg;
    hash_data_->publish(hhm);
  }
  /** publish function for IR data
   * \param msg is the parameters to the ir message
   * the values are IR distance for IR1 and ir 2 */
  void pub_ir(const char* msg)
  {
    (void) msg;
//     auto ir = teensy_bridge_messages::msg::IrDist();
//     // start at first parameter
//     const char * p1= msg;
//     ir.distance[0] = strtof(p1, (char**)&p1); // in meters
//     ir.distance[1] = strtof(p1, (char**)&p1); // in meters
//     ir.ad_value[0] = strtol(p1, (char**)&p1, 10);
//     ir.ad_value[1] = strtol(p1, (char**)&p1, 10);
//     ir.calibrate_13cm[0] = strtol(p1, (char**)&p1, 10);
//     ir.calibrate_50cm[0] = strtol(p1, (char**)&p1, 10);
//     ir.calibrate_13cm[1] = strtol(p1, (char**)&p1, 10);
//     ir.calibrate_50cm[1] = strtol(p1, (char**)&p1, 10);
//     ir.enabled = strtol(p1, (char**)&p1, 10);
//     ir_range_->publish(ir);
  }
  /** publish function for IR data
   * \param msg is the parameters to the ir message
   * the values are IR distance for IR1 and ir 2 */
  void pub_pose(const char* msg)
  {
    auto ps = geometry_msgs::msg::Pose();
    // start at first parameter
    const char * p1= msg;
    (void) /*float t =*/ strtof(p1, (char**)&p1); // teensy time
    ps.position.x = strtof(p1, (char**)&p1); // in meters
    ps.position.y = strtof(p1, (char**)&p1); // in meters
    ps.position.z = strtol(p1, (char**)&p1, 10);
    float hdg = strtof(p1, (char**)&p1);
    float tilt = strtof(p1, (char**)&p1);
    tf2::Quaternion q;
    q.setRPY(0, tilt, hdg);
    ps.orientation.x = q.getX();
    ps.orientation.y = q.getY();
    ps.orientation.z = q.getZ();
    ps.orientation.w = q.getW();
//     printf("# ros2::pub_pose x=%.3f, y=%.3f h=%g, tilt=%g, q:%g %g %g %g\n",
//            ps.position.x, ps.position.y,
//            hdg, tilt, q.getX(), q.getY(), q.getZ(), q.getW());
    pose_->publish(ps);
  }

private:
   rclcpp::Publisher<std_msgs::msg::String>::SharedPtr hash_data_;
//    rclcpp::Publisher<teensy_bridge_messages::msg::IrDist>::SharedPtr ir_range_;
   rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_;
};
#endif



void URos::setup()
{ // bridge source name
  setSourceID("ros");
  start();
  ledIndex = 7;
}

// static varaible - ros is OK, unless thread is stopped.
// bool URos::rosOK = true;

void URos::run()
{ // Initialize ROS2
#ifdef USE_ROS2
  rclcpp::init(argcs, argvs);
  pubsub = std::make_shared<URosPubSub>();
  rosOK = true;
  rosValid = true; // ros is implemented
  rclcpp::on_shutdown(onRosShutdown);
  std::chrono::duration<int64_t, std::milli> t(10);
  while (not th1stop)
    rclcpp::spin(pubsub);
#endif
  rosOK = false;
  th1stop = true;
  printf("# URos::run() has terminated\n");
}

void URos::stopRos()
{
  // none of this works
  // there will be errors at shutdown with q\n
  // - require ctrl-c to shutdown after a q\n
  // - ctrl-c gives a core-dump
  stop();
// tested code elements
//   raise(SIGTERM);
// #ifdef USE_ROS2
//   if (rosOK)
//     rclcpp::shutdown();
// #endif
//   for (int i = 0; i < 20; i++)
//   {
//     if  (not rosOK)
//       break;
//     usleep(5000);
//     printf("# waiting for ROS %d\n", i);
//   }
//   printf("# - ended ROS2 stop\n");
}

void URos::onRosShutdown()
{
  rosif.rosOK = false;
  quitBridge = true;
  // tell main loop before main terminate
  rosif.waitForRos.lock();
}

bool URos::isActive()
{
  return rosOK;
}


void URos::sendString(const char* message, int msTimeout)
{
  (void) msTimeout;
#ifdef USE_ROS2
  if (rosOK)
  {
    const char * p1 = strchr(message, ':');
    if (strncmp(p1, ":#", 2) == 0)
    { // publishes the full message string
  //     printf("# URosSendMessage: sending (timeout=%dms) %s",msTimeout, message);
      pubsub->pub_hash_msg(message);
      RCLCPP_DEBUG(pubsub->get_logger(), "pub message %s", message);
    }
    else if (strncmp(p1, ":ir ", 4) == 0)
    {
      p1 += 4;
      pubsub->pub_ir(p1);
    }
    else if (strncmp(p1, ":pose ", 6) == 0)
    {
      p1 += 6;
      pubsub->pub_pose(p1);
    }
  }
#endif
}
