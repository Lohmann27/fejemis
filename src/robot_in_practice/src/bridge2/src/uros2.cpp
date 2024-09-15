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
#include "uhandler.h" 


URos rosif;


#ifdef USE_ROS2

#include "rclcpp/rclcpp.hpp" // ROS 2 main header
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp" // Include the Twist message

class URosPubSub : public rclcpp::Node
{
public:
    // Constructor
    URosPubSub()
        : Node("bridge")
    {
        hash_data_ = create_publisher<std_msgs::msg::String>("hash_data", 4);
        pose_ = create_publisher<geometry_msgs::msg::Pose>("teensy_pose", 4);

        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "Base parameter interface to Teensy";
        this->declare_parameter("param_line", "key + params", param_desc);

        // Subscriber for Twist messages
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel_joy", 10, std::bind(&URosPubSub::handleTwistCommand, this, std::placeholders::_1));
    }

    // Function to handle Twist commands
    void handleTwistCommand(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        float linear_velocity = msg->linear.x;
        float angular_velocity = msg->angular.z;

        // Convert Twist to motor commands
        setMotorSpeeds(linear_velocity, angular_velocity);

        // Logging for debugging
        RCLCPP_INFO(this->get_logger(), "Handling Twist: linear.x=%f, angular.z=%f", linear_velocity, angular_velocity);
    }

    void setMotorSpeeds(float linear, float angular) {
        // ROS typically provides angular velocity in radians per second
        // Convert angular velocity from radians to degrees
        float angular_degrees = angular * (180.0 / M_PI);

        // Limit the linear velocity to max 0.5
        if (linear > 0.5) linear = 0.5;
        if (linear < -0.5) linear = -0.5;

        // Format the command string as "rc 2 <linear> <angular>"
        char command[100];
        snprintf(command, sizeof(command), "rc 2 %.2f %.2f\n", linear, angular_degrees);

        // Send the command via the UHandler instance
        sendMotorCommand(command);
    }

    // Function to send motor commands using UHandler
    void sendMotorCommand(const char* command) {
      handler.handleCommand(nullptr, command, true); // Pass the command to UHandler

    }

    // Publish function for hash data
    void pub_hash_msg(const char* msg)
    {
        auto hhm = std_msgs::msg::String();
        hhm.data = msg;
        hash_data_->publish(hhm);
    }

    // Publish function for pose data
    void pub_pose(const char* msg)
    {
        auto ps = geometry_msgs::msg::Pose();
        const char * p1 = msg;
        strtof(p1, (char**)&p1); // teensy time
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
        pose_->publish(ps);
    }

    // Function to publish IR data
    void pub_ir(const char* msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received IR data: %s", msg);
        // Add actual IR data handling and publishing logic here
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr hash_data_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_;
};

#endif // USE_ROS2



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

