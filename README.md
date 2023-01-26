This repository contains a ROS2 node for controlling ODrive-Controller-Boards.
It is based on the libusb-based ROS1 ODrive node by Ioannis Kokkoris (https://github.com/johnkok/ros_odrive).

The USB endpoint code was only modified slightly to eliminate compiler warnings and simplify it for only handling a single device at a time.

The ODrive node handles one board at a time. All configuration is done through ROS parameters. Axes are controllable via individual topics. All run-time logic is contained within the node.

A serial number has to be passed as the "serial_number" parameter upon node creation.

# ros2_odrive

ROS2 node and USB endpoint driver for controlling ODrive motor driver boards.

## Overview
This repository provides a USB driver and ROS2 node for ODrive boards.

The provided odrive_endpoint uses the [ODrive Communication Protocol][def2]. 
API descriptors are queried once upon initialisation and can be written to the filesystem.
Subsequently stored API descriptors can be used to speed up initialization time.

The ROS2 node exposes basic commands and feedback as topics.

## ROS2 Parameters

| Parameter |  Type  | Description |
|:-----|:--------:|:---|
| `serial_number` | hex | Hexadecimal serial number of target ODrive board (no leading '0x'). |

## Attributions

The code for the [compile time integer-to-string conversion](include/ros2_odrive/to_string.hpp) was written by Clyne Sullivan [source][def].

The code for the [odrive endpoint](include/ros2_odrive/odrive_endpoint.hpp) and [odrive ros node](include/ros2_odrive/odrive.hpp) is based on the [ros_odrive repository by Ioannis Kokkoris][def3].


[def2]: https://docs.odriverobotics.com/v/latest/protocol.html
[def]: https://github.com/tcsullivan/constexpr-to-string
[def3]: https://github.com/johnkok/ros_odrive