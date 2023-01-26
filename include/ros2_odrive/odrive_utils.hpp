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

#ifndef ODRIVE_UTILS_HPP_
#define ODRIVE_UTILS_HPP_

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <iostream>
#include <string>
#include <vector>
#include <jsoncpp/json/json.h>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "ros2_odrive/odrive_endpoint.hpp"
#include "ros2_odrive/odrive_enums.hpp"

#include "ros2_odrive/to_string.hpp"

int setChannelConfig(odrive_endpoint *endpoint, const Json::Value& odrive_json, Json::Value config_json,
        bool save_config);


template<unsigned int axis> 
int cmd_set_vel(odrive_endpoint* endpoint, const float vel)
{
    const std::string cmd = std::string("axis") + std::string(to_string<axis>) + std::string(".controller.input_vel");
    return endpoint->writeOdriveData(cmd, vel);
}

template<unsigned int axis>
int cmd_set_pos(odrive_endpoint* endpoint, float pos)
{
  const std::string cmd = std::string("axis") + std::string(to_string<axis>) + std::string(".controller.input_pos");
  return endpoint->writeOdriveData(cmd, pos);
}

template<unsigned int axis>
int cmd_set_state(odrive_endpoint* endpoint, AXIS_STATE state)
{
  const std::string cmd = std::string("axis") + std::string(to_string<axis>) + std::string(".requested_state");
  return endpoint->writeOdriveData(cmd, utils::underlying_value(state));
}

#endif

