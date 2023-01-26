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

#ifndef ODRIVE_ENDPOINT_HPP_
#define ODRIVE_ENDPOINT_HPP_

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <iostream>
#include <string>
#include <vector>
#include <endian.h>
#include <mutex>
#include <functional>
#include <utility>

#define ODRIVE_OK 0
#define ODRIVE_ERROR 1

#include "rclcpp/rclcpp.hpp"

#include <jsoncpp/json/json.h>

#include <libusb-1.0/libusb.h>

// ODrive Device Info
#define ODRIVE_USB_VENDORID     0x1209
#define ODRIVE_USB_PRODUCTID    0x0D32

// ODrive USB Protool
#define ODRIVE_TIMEOUT 100
#define ODRIVE_MAX_BYTES_TO_RECEIVE 64
#define ODRIVE_MAX_RESULT_LENGTH 100
//#define ODRIVE_DEFAULT_CRC_VALUE 0x7411
constexpr uint16_t ODRIVE_DEFAULT_CRC_VALUE = 0xd271;
constexpr uint16_t ODRIVE_PROTOCOL_VERSION  = 1;

// ODrive Comm
#define ODRIVE_COMM_SUCCESS 0
#define ODRIVE_COMM_ERROR   1

// Endpoints (from target)
#define CDC_IN_EP                                   0x81  /* EP1 for data IN (target) */
#define CDC_OUT_EP                                  0x01  /* EP1 for data OUT (target) */
#define CDC_CMD_EP                                  0x82  /* EP2 for CDC commands */
#define ODRIVE_IN_EP                                0x83  /* EP3 IN: ODrive device TX endpoint */
#define ODRIVE_OUT_EP                               0x03  /* EP3 OUT: ODrive device RX endpoint */

// CDC Endpoints parameters
#define CDC_DATA_HS_MAX_PACKET_SIZE                 0x40  /* Endpoint IN & OUT Packet size */
#define CDC_DATA_FS_MAX_PACKET_SIZE                 0x40  /* Endpoint IN & OUT Packet size */
#define CDC_CMD_PACKET_SIZE                         0x08  /* Control Endpoint Packet size */

#define USB_CDC_CONFIG_DESC_SIZ                     (67 + 39)
#define CDC_DATA_HS_IN_PACKET_SIZE                  CDC_DATA_HS_MAX_PACKET_SIZE
#define CDC_DATA_HS_OUT_PACKET_SIZE                 CDC_DATA_HS_MAX_PACKET_SIZE

#define CDC_DATA_FS_IN_PACKET_SIZE                  CDC_DATA_FS_MAX_PACKET_SIZE
#define CDC_DATA_FS_OUT_PACKET_SIZE                 CDC_DATA_FS_MAX_PACKET_SIZE

#define CDC_SEND_ENCAPSULATED_COMMAND               0x00
#define CDC_GET_ENCAPSULATED_RESPONSE               0x01
#define CDC_SET_COMM_FEATURE                        0x02
#define CDC_GET_COMM_FEATURE                        0x03
#define CDC_CLEAR_COMM_FEATURE                      0x04
#define CDC_SET_LINE_CODING                         0x20
#define CDC_GET_LINE_CODING                         0x21
#define CDC_SET_CONTROL_LINE_STATE                  0x22
#define CDC_SEND_BREAK                              0x23

void printlibusberror(int error);

typedef struct _odrive_object {
    std::string name;
    int id;
    std::string type;
    std::string access;
} odrive_object;

typedef std::vector<uint8_t> commBuffer;

class odrive_endpoint {
    public:
        odrive_endpoint();
        ~odrive_endpoint();

        int init(std::string serialNumber, bool readConfigFromFile = false, bool writeConfigToFile = false);
        void remove(void);

        int getDescriptor();

        uint64_t serialNumberAsUInt();

        template<typename T>
            int readOdriveData(const odrive_object & odo, T &value);
        template<typename T>
            int readOdriveData(std::string object, T &value);
        
        template<typename T>
            int writeOdriveData(const odrive_object & odo, const T value);
        template<typename T>
            int writeOdriveData(std::string object, const T value);

        int execOdriveFunc(const odrive_object& object);
        int execOdriveFunc(std::string object);

        int endpointRequest(uint16_t endpoint_id, commBuffer& received_payload,
            int& received_length, commBuffer payload, bool ack = false,
            int length = 0, bool read = false, int address = 0);

        enum STATE : uint32_t {
            NOT_INITIALIZED     = 0,
            CONNECTED           = 1,
            OPENED              = 2,
            CONFIGURED          = 4,
            RUNNING             = 8,
            ODRV_ERROR          = 16,
            USB_ERROR           = 32
        };

        STATE state() const;
        bool isRunning()    const;
        bool hasError()     const;
        bool isConfigured() const;
        bool isConnected()  const;
        bool hasOdrvError() const;
        bool hasUsbError()  const;

        void setFlag(STATE state);
        void clearFlag(STATE state);

        int getObjectByName(std::string name, odrive_object *odo) const;

    private:
        libusb_context* libusb_context_;
        short outbound_seq_no_{0};
        libusb_device *_odrive_device{NULL};
        libusb_device_handle *odrive_handle_ = NULL;

        std::mutex ep_lock;

        std::string serial_number_{""};
        std::string configuration_path_{""};
        bool read_config_from_file{false};
        bool write_config_to_file{false};

        std::unique_ptr<Json::Value> odrive_json_;

        STATE _state{NOT_INITIALIZED};

        template<unsigned int SIZE, typename T>
            int appendToCommBuffer(commBuffer& buf, const T value);

        commBuffer decodeODrivePacket(commBuffer& received_packet, uint16_t& seq_no);
        commBuffer createODrivePacket(uint16_t seq_no, uint16_t endpoint_id, uint16_t response_size,
            bool read, int address, const commBuffer& input);

        libusb_device* find_device();
        int open_device(libusb_device* device = nullptr);
        void close_device(libusb_device_handle* device_handle = nullptr);

        template<typename T>
            int getData(int id, T& value);
        template<typename TT>
            int setData(int id, const TT value);

        int execFunc(int id);


};

#endif // ODRIVE_ENDPOINT_HPP_

