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

#include "ros2_odrive/odrive_utils.hpp"

#define ROS_INFO(...) RCLCPP_INFO(rclcpp::get_logger("odrive_utils"), __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("odrive_utils"), __VA_ARGS__)

// /**
//  *
//  *  Set odrive config
//  *  Configure parameter on odrive hardware
//  *  @param endpoint odrive enumarated endpoint
//  *  @param odrive_json target json
//  *  @param config_json json including configuration parameters
//  *  @return ODRIVE_OK on success
//  *
//  */
// int setChannelConfig(odrive_endpoint *endpoint, Json::Value& odrive_json, Json::Value config_json,
//                      bool save_config = 0)
// {
//     int ret = ODRIVE_OK;

//     for (unsigned int i = 0; i < config_json.size(); i++)
//     {
//         std::string name = config_json[i]["name"].asString();
//         std::string type = config_json[i]["type"].asString();

//         ROS_INFO("Setting %s config value", name.c_str());

//         if (!type.compare("float"))
//         {
//             float val = config_json[i]["value"].asFloat();
//             writeOdriveData(endpoint, odrive_json, name, val);
//         }
//         else if (!type.compare("uint8"))
//         {
//             uint8_t val = config_json[i]["value"].asUInt();
//             writeOdriveData(endpoint, odrive_json, name, val);
//         }
//         else if (!type.compare("uint16"))
//         {
//             uint16_t val = config_json[i]["value"].asUInt();
//             writeOdriveData(endpoint, odrive_json, name, val);
//         }
//         else if (!type.compare("uint32"))
//         {
//             uint32_t val = config_json[i]["value"].asUInt();
//             writeOdriveData(endpoint, odrive_json, name, val);
//         }
//         else if (!type.compare("uint64"))
//         {
//             uint64_t val = config_json[i]["value"].asUInt64();
//             writeOdriveData(endpoint, odrive_json, name, val);
//         }
//         else if (!type.compare("int32"))
//         {
//             int val = config_json[i]["value"].asInt();
//             writeOdriveData(endpoint, odrive_json, name, val);
//         }
//         else if (!type.compare("int16"))
//         {
//             short val = config_json[i]["value"].asInt();
//             writeOdriveData(endpoint, odrive_json, name, val);
//         }
//         else
//         {
//             ROS_ERROR("* Error: invalid type for %s", name.c_str());
//             return ODRIVE_ERROR;
//         }
//     }

//     // Save configuration
//     if (save_config)
//     {
//         ret = execOdriveFunc(endpoint, odrive_json, "save_configuration");
//     }

//     return ret;
// }


#undef ROS_INFO
#undef ROS_ERROR