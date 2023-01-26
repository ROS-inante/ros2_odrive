/*
* Copyright (C) 2021 Alexander Junk <dev@junk.technology>
* 
* This code is based on the ros_odrive repository by Ioannis Kokkoris
* (https://github.com/johnkok/ros_odrive), which is distributed under the
* MIT license.
* 
* This program is free software: you can redistribute it and/or modify it 
* under the terms of the GNU Lesser General Public License as published 
* by the Free Software Foundation, either version 3 of the License, or 
* (at your option) any later version.
* 
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; 
* without even the implied warranty of MERCHANTABILITY or FITNESS
* FOR A PARTICULAR PURPOSE. 
* See the GNU Lesser General Public License for more details.
* 
* You should have received a copy of the GNU Lesser General Public License 
* along with this program. If not, see <https://www.gnu.org/licenses/>. 
*
*/

#ifndef ODRIVE_H
#define ODRIVE_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <jsoncpp/json/json.h>

#include "rclcpp/rclcpp.hpp"

#include "ros2_odrive/odrive_enums.hpp"
#include "ros2_odrive/odrive_utils.hpp"
#include "ros2_odrive/odrive_endpoint.hpp"

#include "ros2_odrive_interfaces/msg/status.hpp"
#include "ros2_odrive_interfaces/msg/ctrl.hpp"
#include "ros2_odrive_interfaces/msg/velocity.hpp"
#include "ros2_odrive_interfaces/msg/position.hpp"
#include "ros2_odrive_interfaces/msg/error.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;


// Listener commands
enum ODRIVE_CMD {
    AXIS_RESET,
    AXIS_IDLE,
    AXIS_CLOSED_LOOP,
    AXIS_SET_VELOCITY,
    AXIS_SET_VELOCITY_DUAL,
    REBOOT
};



class ODrive : public rclcpp::Node
{
public:
  
  ODrive();

  ~ODrive();

private:
  void timer_callback()
  {
    if( publish_odrive_status()){
      RCLCPP_ERROR(this->get_logger(), "Failed at publishing state!");
    }

    if( publish_error<0>()){
      RCLCPP_ERROR(this->get_logger(), "Failed at publishing error state state!");
    }
    
    if( publish_error<1>()){
      RCLCPP_ERROR(this->get_logger(), "Failed at publishing error state state!");
    }
    
    if( publish_vel_estimate<0>()){
      RCLCPP_ERROR(this->get_logger(), "Failed at publishing velocity estimate state!");
    }

    if( publish_vel_estimate<1>()){
      RCLCPP_ERROR(this->get_logger(), "Failed at publishing velocity estimate state!");
    }

    return;
  }

  int publish_odrive_status() const;

  void ctrl_topic_callback(const ros2_odrive_interfaces::msg::Ctrl::SharedPtr) const;

  template<unsigned int AXIS>
    void cmd_vel_topic_callback(const ros2_odrive_interfaces::msg::Velocity::SharedPtr ) const;
  
  template<unsigned int AXIS>
    void cmd_pos_topic_callback(const ros2_odrive_interfaces::msg::Position::SharedPtr) const;

  template<unsigned int AXIS>
    int publish_error(void) const;
  
  template<unsigned int AXIS>
    int publish_vel_estimate(void) const;

  template<unsigned int AXIS>
    int cmd_set_vel(float vel) const;

  template<unsigned int AXIS>
    int cmd_set_pos(float pos) const;

  template<unsigned int AXIS>
    int cmd_set_state(AXIS_STATE state) const;


  std::string parameter_sn_;
  std::string parameter_cfg_;

  odrive_endpoint* endpoint_;
  Json::Value json_;
  std::string cfg_;

  rclcpp::Publisher<ros2_odrive_interfaces::msg::Status>::SharedPtr status_publisher_;
  rclcpp::Subscription<ros2_odrive_interfaces::msg::Ctrl>::SharedPtr ctrl_subscriber_;

  std::vector<rclcpp::Subscription<ros2_odrive_interfaces::msg::Velocity>::SharedPtr> velocity_subscribers_;
  std::vector<rclcpp::Subscription<ros2_odrive_interfaces::msg::Position>::SharedPtr> position_subscribers_;
  std::vector<rclcpp::Publisher<ros2_odrive_interfaces::msg::Error>::SharedPtr> error_publishers_;
  std::vector<rclcpp::Publisher<ros2_odrive_interfaces::msg::Velocity>::SharedPtr> velocity_publishers_;

  rclcpp::TimerBase::SharedPtr timer_;
};

#endif //ODRIVE_H
