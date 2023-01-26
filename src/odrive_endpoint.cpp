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

#include "ros2_odrive/odrive_endpoint.hpp"

#include <iostream>
#include <fstream>

#define ROS_INFO(...)  RCLCPP_INFO(rclcpp::get_logger("odrive_endpoint"), __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("odrive_endpoint"), __VA_ARGS__)


using namespace std;

void printlibusberror(int error)
{
    switch (error){
        case LIBUSB_ERROR_TIMEOUT:
            ROS_INFO("LibUSB ERROR: Transfer timed out!");
            break;
        case LIBUSB_ERROR_PIPE:
            ROS_INFO("LibUSB ERROR: Endpoint halted!");
            break;
        case LIBUSB_ERROR_OVERFLOW:
            ROS_INFO("LibUSB ERROR: Device offered more data!");
            break;
        case LIBUSB_ERROR_NO_DEVICE:
            ROS_INFO("LibUSB ERROR: Device disconnected!");
            break;
        default:
            if(error != LIBUSB_SUCCESS) ROS_INFO("LibUSB ERROR: Generic error!");
            break;
    }


}

/**
 *
 * Odrive endpoint constructor
 * initialize USB library and local variables
 *
 */
odrive_endpoint::odrive_endpoint()
{
    if (libusb_init(&libusb_context_) != LIBUSB_SUCCESS) {
        ROS_ERROR("* Error initializing USB!");
    }
}

/**
 *
 * Odrive endpoint destructor
 * release USB library
 *
 */
odrive_endpoint::~odrive_endpoint()
{
    this->remove();
    
    if (libusb_context_ != NULL) {
        libusb_exit(libusb_context_);
        libusb_context_ = NULL;
    }
}

uint64_t odrive_endpoint::serialNumberAsUInt(){
            return std::stoull(serial_number_.c_str(), 0, 16);
}

/**
 *
 * Append data to buffer
 * @param buf data buffer
 * @param value data to append
 *
 */
template<unsigned int SIZE, typename T> int odrive_endpoint::appendToCommBuffer(commBuffer& buf, const T value)
{
    static_assert(SIZE <= sizeof(T), "Error: Tried to extract too many bytes from too small of a type.");
    
    for(unsigned int i = 0; i < SIZE; i++){
        buf.push_back((value >> i*8) & 0xFF);
    }

    return 0;
}


/**
 *
 *  Decode odrive packet
 *  @param buf data buffer
 *  @param seq_no packet sequence number
 *  @param received_packet received buffer
 *  @return data buffer
 *
 */
commBuffer odrive_endpoint::decodeODrivePacket(commBuffer& received_packet,
    	uint16_t& seq_no)
{
    commBuffer payload;

    memcpy(&seq_no, received_packet.data(), sizeof(short));
    seq_no &= 0x7fff;
    for (commBuffer::size_type i = 2; i < received_packet.size(); ++i) {
        payload.push_back(received_packet[i]);
    }
    return payload;
}

/**
 *
 * Read data buffer from Odrive harware
 * @param seq_no next sequence number
 * @param endpoint_id USB endpoint ID
 * @param response_size maximum data length to be read
 * @param read append request address
 * @param address desctination address
 * @param input data buffer to send
 * @return data buffer read
 *
 */
//ToDo: Preallocate buffer and change appending to directly write to memory block (caveat endianness).
commBuffer odrive_endpoint::createODrivePacket(uint16_t seq_no, uint16_t endpoint_id,
                uint16_t response_size, bool read, int address, const commBuffer& input)
{
    //ToDo: Calculate CRC correctly, seems to not be checked by odrive for now.
    commBuffer packet;
    uint16_t crc = 0;

    appendToCommBuffer<2>(packet, seq_no);
    //appendShortToCommBuffer(packet, seq_no);

    appendToCommBuffer<2>(packet, endpoint_id);
    appendToCommBuffer<2>(packet, response_size);
    
    if (read) {
        appendToCommBuffer<4>(packet, address);
    }

    for (uint8_t b : input) {
        packet.push_back(b);
    }

    crc = ((endpoint_id & 0x7fff) == 0) ? ODRIVE_PROTOCOL_VERSION : ODRIVE_DEFAULT_CRC_VALUE;


    appendToCommBuffer<2>(packet, crc);

    return packet;
}

/**
 *
 *  Read value from ODrive
 *  @param id odrive ID
 *  @param value Data read
 *  @return ODRIVE_OK on success
 *
 */
template<typename T>
int odrive_endpoint::getData(int id, T& value)
{
    commBuffer tx;
    commBuffer rx;
    int rx_size;

    int result = endpointRequest(id, rx,
                    rx_size, tx, 1 /* ACK */, sizeof(value));
    
    if (result != LIBUSB_SUCCESS) {
        return ODRIVE_ERROR;
    }


    //ToDo: Deserialize properly
    //This only works if the host system running the node is little endian
    value = reinterpret_cast<T*>(rx.data())[0];

    return ODRIVE_OK;
}

/**
 *
 *  Write value to Odrive
 *  @param id odrive ID
 *  @param value Data to be written
 *  @return ODRIVE_OK on success
 *
 */
template<typename TT>
int odrive_endpoint::setData(int endpoint_id, const TT value)
{
    commBuffer tx;
    commBuffer rx;
    int rx_length;

    for(unsigned int i = 0; i < sizeof(value); i++){
       tx.push_back(((uint8_t*) &value)[i]);
    }
    
    int ret = endpointRequest(endpoint_id, rx, rx_length, tx, 1, 0);

    return (ret == LIBUSB_SUCCESS) ? ODRIVE_OK : ODRIVE_ERROR; 
}


/**
 *
 *  Request function to ODrive
 *  @param id odrive ID
 *  @return ODRIVE_OK on success
 *
 */
int odrive_endpoint::execFunc(int endpoint_id)
{
    commBuffer tx;
    commBuffer rx;
    int rx_length;
    int status;

    status = endpointRequest(endpoint_id, rx, rx_length, tx, 1, 0);
    if (status != LIBUSB_SUCCESS) {
        ROS_ERROR(std::string("* execFunc: Error in endpoint request " + std::to_string(endpoint_id) ).c_str());
        return ODRIVE_ERROR;
    }

    return ODRIVE_OK;
}

/**
 *
 * Request endpoint
 * @param handle USB device handler
 * @param endpoint_id odrive ID
 * @param received_payload receive buffer
 * @param received_length receive length
 * @param payload data read
 * @param ack request acknowledge
 * @param length data length
 * @param read send read address
 * @param address read address
 * @return LIBUSB_SUCCESS on success
 *
 */
int odrive_endpoint::endpointRequest(uint16_t endpoint_id, commBuffer& received_payload,
    	int& received_length, commBuffer payload,
    	bool ack, int length, bool read, int address)
{
    commBuffer send_buffer;
    commBuffer receive_buffer;
    unsigned char receive_bytes[ODRIVE_MAX_RESULT_LENGTH] = { 0 };
    int sent_bytes = 0;
    int received_bytes = 0;
    uint16_t received_seq_no = 0;

    if(!this->isConnected() && (this->init(this->serial_number_) != ODRIVE_OK)){
        //ROS_INFO("ODrive disconnected.");
        return LIBUSB_ERROR_NO_DEVICE;
    }

    if(!ep_lock.try_lock()){
        ROS_INFO("Could not send data on time, operation still in progress.");
        return LIBUSB_ERROR_BUSY;
    }

    // Prepare sequence number
    if (ack) {
        endpoint_id |= 0x8000;
    }
    outbound_seq_no_ = (outbound_seq_no_ + 1) & 0x7fff;
    outbound_seq_no_ |= LIBUSB_ENDPOINT_IN;
    uint16_t seq_no = outbound_seq_no_;

    // Create request packet
    commBuffer packet = createODrivePacket(seq_no, endpoint_id, length, read, address, payload);

    // Transfer paket to target
    int result = libusb_bulk_transfer(odrive_handle_, ODRIVE_OUT_EP,
    	    packet.data(), packet.size(), &sent_bytes, ODRIVE_TIMEOUT);

    //printlibusberror(result);

    if(result == LIBUSB_ERROR_NO_DEVICE){
        this->remove();
        ep_lock.unlock();
        return LIBUSB_ERROR_NO_DEVICE;
    }

    if(result == LIBUSB_ERROR_TIMEOUT)
    {
        ROS_INFO("Timed out and sent %d of %ld bytes to ODrive [0x%8.8lX]!", sent_bytes, packet.size(), serialNumberAsUInt());
        ep_lock.unlock();
        return result;
    }else if (result != LIBUSB_SUCCESS) {
        ROS_ERROR("LibUSB request: Error in transfering data to ODrive [0x%8.8lX]!", serialNumberAsUInt());
        ep_lock.unlock();
        return result;
    } else if (packet.size() != static_cast<unsigned int>(sent_bytes)) {
        ROS_ERROR("LibUSB: Error in transfering data to USB, not all data transferred to ODrive [0x%8.8lX]!", serialNumberAsUInt());
    }

    // Get responce
    if (ack) {
        result = libusb_bulk_transfer(odrive_handle_, ODRIVE_IN_EP,
    		receive_bytes, ODRIVE_MAX_BYTES_TO_RECEIVE,
    		&received_bytes, ODRIVE_TIMEOUT);

        if(result == LIBUSB_ERROR_NO_DEVICE){
            this->remove();
            ep_lock.unlock();
            return LIBUSB_ERROR_NO_DEVICE;
        }

        if (result != LIBUSB_SUCCESS) {
            ROS_ERROR("LibUSB request: Error in reading data from ODrive [0x%8.8lX]!", serialNumberAsUInt());
            ep_lock.unlock();
            return result;
        }

        // Push recevived data to buffer
        for (int i = 0; i < received_bytes; i++) {
            receive_buffer.push_back(receive_bytes[i]);
        }

        received_payload = decodeODrivePacket(receive_buffer, received_seq_no);
        if (received_seq_no != seq_no) {
            ROS_ERROR("LibUSB request: Error Received data out of orderfrom ODrive [0x%8.8lX]!", serialNumberAsUInt());
        }
        received_length = received_payload.size();
    }

    ep_lock.unlock();

    return LIBUSB_SUCCESS;
}

/**
 *
 * open usb device
 * @param device LIBUSB_DEVICE
 * @return Valid libusb_device pointer on success, otherwise nullptr
 *
 */
libusb_device* odrive_endpoint::find_device()
{
    libusb_device *device = nullptr;
    libusb_device ** usb_device_list;

    ssize_t device_count = libusb_get_device_list(libusb_context_, &usb_device_list);
    if (device_count <= 0) {
        return nullptr;
    }

    for (size_t i = 0; i < static_cast<size_t>(device_count); ++i) {
        device = usb_device_list[i];

        if(open_device(device) == ODRIVE_OK)
            break;
        else
            device = nullptr;
    }
    libusb_free_device_list(usb_device_list, 1);

    return device;
}

/**
 *
 * open usb device
 * @param device LIBUSB_DEVICE
 * @return ODRIVE_OK on success
 *
 */
int odrive_endpoint::open_device(libusb_device* device)
{
    libusb_device_descriptor desc = {};

    if(!device && !this->_odrive_device){
        return ODRIVE_ERROR;
    }else if(!device && this->_odrive_device){
        device = _odrive_device;
    }

    int result = libusb_get_device_descriptor(device, &desc);
    if (result != LIBUSB_SUCCESS) {
        ROS_ERROR("LibUSB error getting device descriptor");
        return ODRIVE_ERROR;
    }
    /* Check USB devicei ID */
    if (desc.idVendor == ODRIVE_USB_VENDORID && desc.idProduct == ODRIVE_USB_PRODUCTID) {

        libusb_device_handle *device_handle;
        if (libusb_open(device, &device_handle) != LIBUSB_SUCCESS) {
            //ROS_ERROR("Error opening USB device");
            return ODRIVE_ERROR;
            }

        struct libusb_config_descriptor *config;
        result = libusb_get_config_descriptor(device, 0, &config);
        int ifNumber = 2; //config->bNumInterfaces;

        if ((libusb_kernel_driver_active(device_handle, ifNumber) != LIBUSB_SUCCESS) &&
                (libusb_detach_kernel_driver(device_handle, ifNumber) != LIBUSB_SUCCESS)) {
            ROS_ERROR("*LibUSB kernel driver error");
            libusb_close(device_handle);
            return ODRIVE_ERROR;
        }

        if ((result = libusb_claim_interface(device_handle, ifNumber)) !=  LIBUSB_SUCCESS) {
            //ROS_INFO("Error claiming USB device");
            libusb_close(device_handle);
            return ODRIVE_ERROR;
        } else {
            bool attached_to_handle = false;
            unsigned char buf[128];

            result = libusb_get_string_descriptor_ascii(device_handle, desc.iSerialNumber, buf, 127);
            if (result <= 0) {
                ROS_ERROR("Could not get serial number from USB device.");
                result = libusb_release_interface(device_handle, ifNumber);
                libusb_close(device_handle);
                return ODRIVE_ERROR;
            } else {

                //ToDo: Conversion redundant not. Remove.
                std::stringstream stream;
                stream << uppercase << std::hex << serialNumberAsUInt();
                std::string sn(stream.str());

                if (sn.compare(0, strlen((const char*)buf), (const char*)buf) == 0) {
                    ROS_INFO("Device 0x%8.8lX Found", serialNumberAsUInt());
                    ROS_INFO("FOUND DEVICE");
                    libusb_ref_device(device);
                    _odrive_device = device;
                    odrive_handle_ = device_handle;
                    attached_to_handle = true;
                    this->setFlag(odrive_endpoint::STATE::OPENED);
                    this->setFlag(odrive_endpoint::STATE::CONNECTED);

                    return ODRIVE_OK;
                }    	
            }
            if (!attached_to_handle) {
                result = libusb_release_interface(device_handle, ifNumber);
                libusb_close(device_handle);
            }
        }
    }

    return ODRIVE_ERROR;
}

/**
 *
 * Odrive endpoint close
 * close ODrive handle
 *
 */
void odrive_endpoint::close_device(libusb_device_handle* device_handle)
{
    if(!device_handle && !this->odrive_handle_){
        ROS_INFO("Trying to close nonexistent usb device.");
        return;
    }else if(!device_handle && this->odrive_handle_){
        device_handle = odrive_handle_;
    }

    if(device_handle != nullptr) {
        libusb_release_interface(device_handle, 2);
        libusb_close(device_handle);
        odrive_handle_ = nullptr;
    }

    this->clearFlag(odrive_endpoint::STATE::OPENED);
}

/**
 *
 * Odrive endpoint init
 * enumerate ODrive hardware
 * @param serialNumber odrive serial number
 * @return ODRIVE_OK on success
 *
 */
int odrive_endpoint::init(std::string serialNumber, bool readConfigFromFile, bool writeConfigToFile)    	
{
    int ret = ODRIVE_ERROR;

    serial_number_ = serialNumber;
    read_config_from_file = readConfigFromFile;
    write_config_to_file = writeConfigToFile;


    libusb_device *device = find_device();

    if(device){
        ret =  ODRIVE_OK;
        //ROS_INFO("Endpoint initialized. [ONLINE]");
    }else{
        //ROS_INFO("Endpoint not initialized. [OFFLINE]");
    }

    return ret;
}


/**
 *
 *  Read JSON file from target
 *  @return 0 on success
 *
 */
int odrive_endpoint::getDescriptor()
{

    commBuffer rx;
    commBuffer tx;
    int len;
    int address = 0;
    std::string json;

    if(read_config_from_file){
        ROS_INFO("Reading JSON endpoint description from file.");

        odrive_json_.reset(new Json::Value());
        bool ok = false;

        ifstream in_file(serial_number_);
        
        if(in_file.is_open()){
            std::string errors;
            Json::CharReaderBuilder builder;
            builder["collectComments"] = false;
            ok = parseFromStream(builder, in_file, odrive_json_.get(), &errors);
        }

        if(!ok){
            ROS_ERROR("Could not open JSON-Descriptor from file. Trying to get from board.");
            odrive_json_.reset();
        }else{
            setFlag(odrive_endpoint::STATE::CONFIGURED);
            return 0;
        }
    }


    ROS_INFO("Acquiring JSON endpoint description. (This could take a while).");
    do
    {
        int ret = 0;
        unsigned retry = 10;
        do
        {
            ret = endpointRequest(0, rx, len, tx, true, 64, true, address);
            retry--;
        }while(ret != LIBUSB_SUCCESS && retry > 0);

        if(ret != LIBUSB_SUCCESS)
        {
            ROS_ERROR("Failed to get JSON-description block from ODrive.");
            return 1;
        }

        address = address + len;
        json.append((const char *)&rx[0], (size_t)len);
    } while (len > 0);

    odrive_json_.reset(new Json::Value());

    ROS_INFO("Parsing JSON endpoint description.");
    Json::Reader reader;
    bool res = reader.parse(json, *odrive_json_);
    if (!res)
    {
        ROS_ERROR("ODrive-Endpoint could not parse JSON.");
        odrive_json_.reset();
        return 1;
    }

    if(write_config_to_file){
        ROS_INFO("Writing JSON endpoint description to file.");
        Json::StreamWriterBuilder builder;
        builder["indentation"] = "";  // assume default for comments is None

        std::ofstream out_file(this->serial_number_);
        
        if(out_file.is_open() && write_config_to_file){
            out_file << Json::writeString(builder, *odrive_json_);
            out_file.close();
        }else{
            ROS_ERROR("Could not open file for dumping JSON-Descriptor!");
        }
    }

    setFlag(odrive_endpoint::STATE::CONFIGURED);
    return 0;
}


/**
 *
 * Odrive endpoint remove
 * close ODrive dvice
 *
 */
void odrive_endpoint::remove(void)
{
    this->close_device();

    libusb_unref_device(_odrive_device);
    this->_odrive_device = nullptr;
    
    this->clearFlag(odrive_endpoint::STATE::CONNECTED);
    
}

odrive_endpoint::STATE odrive_endpoint::state() const
{
    return this->_state;
}

bool odrive_endpoint::isRunning()    const
{
    return this->_state & odrive_endpoint::STATE::RUNNING;
}

bool odrive_endpoint::hasError()     const
{
    return this->_state & (odrive_endpoint::STATE::ODRV_ERROR | odrive_endpoint::STATE::USB_ERROR);
}

bool odrive_endpoint::isConfigured() const
{
    return this->_state & odrive_endpoint::STATE::CONFIGURED;
}

bool odrive_endpoint::isConnected()  const
{
    return this->_state & odrive_endpoint::STATE::CONNECTED;
}

bool odrive_endpoint::hasOdrvError() const
{
    return this->_state & odrive_endpoint::STATE::ODRV_ERROR;
}

bool odrive_endpoint::hasUsbError()  const
{
    return this->_state & odrive_endpoint::STATE::USB_ERROR;
}

void odrive_endpoint::setFlag(odrive_endpoint::STATE state)
{
    this->_state = static_cast<odrive_endpoint::STATE>(
                    static_cast<std::underlying_type<odrive_endpoint::STATE>::type>(this->_state) 
                    | static_cast<std::underlying_type<odrive_endpoint::STATE>::type>(state));
}

void odrive_endpoint::clearFlag(odrive_endpoint::STATE state)
{
    this->_state = static_cast<odrive_endpoint::STATE>(
                    static_cast<std::underlying_type<odrive_endpoint::STATE>::type>(this->_state) 
                    & ~static_cast<std::underlying_type<odrive_endpoint::STATE>::type>(state));

    ROS_INFO("%u", static_cast<std::underlying_type<odrive_endpoint::STATE>::type>(this->_state));
}


template int odrive_endpoint::getData(int, bool&);
template int odrive_endpoint::getData(int, short&);
template int odrive_endpoint::getData(int, int&);
template int odrive_endpoint::getData(int, float&);
template int odrive_endpoint::getData(int, uint8_t&);
template int odrive_endpoint::getData(int, uint16_t&);
template int odrive_endpoint::getData(int, uint32_t&);
template int odrive_endpoint::getData(int, uint64_t&);

template int odrive_endpoint::setData(int, const short);
template int odrive_endpoint::setData(int, const int);
template int odrive_endpoint::setData(int, const float);
template int odrive_endpoint::setData(int, const uint8_t);
template int odrive_endpoint::setData(int, const uint16_t);
template int odrive_endpoint::setData(int, const uint32_t);
template int odrive_endpoint::setData(int, const uint64_t);



/**
 *
 *  Scan for object name in target JSON
 *  @param odrive_json target json
 *  @param name object name to be found
 *  @param odo odrive object pointer including object parameters
 *  @return ODRIVE_OK on success
 *
 */
int odrive_endpoint::getObjectByName(std::string name, odrive_object *odo) const
{
    int ret = -1;
    size_t pos;
    std::string token;
    Json::Value js;
    Json::Value js2 = *odrive_json_;

    while ((pos = name.find(".")) != std::string::npos)
    {
        js = js2;
        token = name.substr(0, pos);
        for (unsigned i = 0; i < js.size(); i++)
        {
            if (!token.compare(js[i]["name"].asString()))
            {
                if (!std::string("object").compare(js[i]["type"].asString()))
                {
                    js2 = js[i]["members"];
                }
                else
                {
                    js2 = js[i];
                }
                break;
            }
        }
        name.erase(0, pos + 1);
    }

    for (unsigned i = 0; i < js2.size(); i++)
    {
        if (!name.compare(js2[i]["name"].asString()))
        {
            odo->name = js2[i]["name"].asString();
            odo->id = js2[i]["id"].asInt();
            odo->type = js2[i]["type"].asString();
            odo->access = js2[i]["access"].asString();
            ret = 0;
            break;
        }
    }

    if (ret)
    {
        ROS_ERROR("* %s not found!", name.c_str());
    }
    return ret;
}

/**
 *
 *  Read single value from target
 *  @param endpoint odrive enumarated endpoint
 *  @param odrive_object target descriptor object
 *  @param value return value
 *  @return ODRIVE_OK on success
 *
 */
template <typename T>
int odrive_endpoint::readOdriveData(const odrive_object& odo, T &value)
{
 
    if (odo.access.find("r") == std::string::npos)
    {
        ROS_ERROR("* Error: invalid read access for %s", odo.name.c_str());
        return ODRIVE_ERROR;
    }

    if (!odo.type.compare("float"))
    {
        if (sizeof(value) != sizeof(float))
        {
            ROS_ERROR("* Error value for %s is not float", odo.name.c_str());
            return ODRIVE_ERROR;
        }
    }
    else if (!odo.type.compare("uint8"))
    {
        if (sizeof(value) < sizeof(uint8_t))
        {
            ROS_ERROR("* Error value for %s [%s] is not uint8_t",odo.type.c_str(), odo.name.c_str());
            return ODRIVE_ERROR;
        }
    }
    else if (!odo.type.compare("uint16"))
    {
        if (sizeof(value) != sizeof(uint16_t))
        {
            ROS_ERROR("* Error value for %s is not uint16_t", odo.name.c_str());
            return ODRIVE_ERROR;
        }
    }
    else if (!odo.type.compare("uint32"))
    {
        if (sizeof(value) != sizeof(uint32_t))
        {
            ROS_ERROR("* Error value for %s is not uint32_t", odo.name.c_str());
            return ODRIVE_ERROR;
        }
    }
    else if (!odo.type.compare("uint64"))
    {
        if (sizeof(value) != sizeof(uint64_t))
        {
            ROS_ERROR("* Error value for %s is not uint64_t", odo.name.c_str());
            return ODRIVE_ERROR;
        }
    }
    else if (!odo.type.compare("int32"))
    {
        if (sizeof(value) != sizeof(int))
        {
            ROS_ERROR("* Error value for %s is not int", odo.name.c_str());
            return ODRIVE_ERROR;
        }
    }
    else if (!odo.type.compare("int16"))
    {
        if (sizeof(value) != sizeof(short))
        {
            ROS_ERROR("* Error value for %s is not short", odo.name.c_str());
            return ODRIVE_ERROR;
        }
    }
    else if (!odo.type.compare("bool"))
    {
        if (sizeof(value) != sizeof(bool))
        {
            ROS_ERROR("* Error value for %s is not bool", odo.name.c_str());
            return ODRIVE_ERROR;
        }
    }
    else
    {
        ROS_ERROR("* Error: invalid type for %s", odo.name.c_str());
        return ODRIVE_ERROR;
    }

    int ret = getData(odo.id, value);

    return ret;
}

/**
 *
 *  Read single value from target
 *  @param endpoint odrive enumarated endpoint
 *  @param odrive_json target json
 *  @param value return value
 *  @return ODRIVE_OK on success
 *
 */
template <typename T>
int odrive_endpoint::readOdriveData(std::string object, T &value)
{
    int ret;
    odrive_object odo;

    ret = getObjectByName(object, &odo);

    if (ret)
    {
        ROS_ERROR("* Error getting ID for %s", object.c_str());
        return ret;
    }

    return readOdriveData(odo, value);
}

/**
 *
 *  Write single value to target
 *  @param value value to be written
 *  @return ODRIVE_OK on success
 *
 */
template <typename T>
int odrive_endpoint::writeOdriveData(const odrive_object& odo, const T value)
{
    if (odo.access.find("w") == std::string::npos)
    {
        ROS_ERROR("* Error: invalid write access for %s", odo.name.c_str());
        return ODRIVE_ERROR;
    }

    if (!odo.type.compare("float"))
    {
        if (sizeof(value) != sizeof(float))
        {
            ROS_ERROR("* Error value for %s is not float", odo.name.c_str());
            return ODRIVE_ERROR;
        }
    }
    else if (!odo.type.compare("uint8"))
    {
        //ToDo: CHANGE THE COMPARISON BACK
        if (sizeof(value) < sizeof(uint8_t))
        {
            ROS_ERROR("* Error value for %s is not uint8_t", odo.name.c_str());
            return ODRIVE_ERROR;
        }
    }
    else if (!odo.type.compare("uint16"))
    {
        if (sizeof(value) != sizeof(uint16_t))
        {
            ROS_ERROR("* Error value for %s is not uint16_t", odo.name.c_str());
            return ODRIVE_ERROR;
        }
    }
    else if (!odo.type.compare("uint32"))
    {
        if (sizeof(value) != sizeof(uint32_t))
        {
            ROS_ERROR("* Error value for %s is not uint32_t", odo.name.c_str());
            return ODRIVE_ERROR;
        }
    }
    else if (!odo.type.compare("uint64"))
    {
        if (sizeof(value) != sizeof(uint64_t))
        {
            ROS_ERROR("* Error value for %s is not uint64_t", odo.name.c_str());
            return ODRIVE_ERROR;
        }
    }
    else if (!odo.type.compare("int32"))
    {
        if (sizeof(value) != sizeof(int))
        {
            ROS_ERROR("* Error value for %s is not int", odo.name.c_str());
            return ODRIVE_ERROR;
        }
    }
    else if (!odo.type.compare("int16"))
    {
        if (sizeof(value) != sizeof(short))
        {
            ROS_ERROR("* Error value for %s is not short", odo.name.c_str());
            return ODRIVE_ERROR;
        }
    }
    else if (!odo.type.compare("bool"))
    {
        if (sizeof(value) != sizeof(bool))
        {
            ROS_ERROR("* Error value for %s is not bool", odo.name.c_str());
            return ODRIVE_ERROR;
        }
    }
    else
    {
        ROS_ERROR("* Error: invalid type for %s", odo.name.c_str());
        return ODRIVE_ERROR;
    }

    return setData(odo.id, value);
}

/**
 *
 *  Write single value to target
 *  @param value value to be written
 *  @return ODRIVE_OK on success
 *
 */
template <typename T>
int odrive_endpoint::writeOdriveData(std::string object, const T value)
{
    int ret;
    odrive_object odo;

    ret = getObjectByName(object, &odo);
    if (ret)
    {
        ROS_ERROR("* Error: getting ID for %s", object.c_str());
        return ODRIVE_ERROR;
    }

    return writeOdriveData(odo, value);
}

template int odrive_endpoint::writeOdriveData(std::string, const uint8_t);
template int odrive_endpoint::writeOdriveData(std::string, const uint16_t);
template int odrive_endpoint::writeOdriveData(std::string, const uint32_t);
template int odrive_endpoint::writeOdriveData(std::string, const uint64_t);
template int odrive_endpoint::writeOdriveData(std::string, const int);
template int odrive_endpoint::writeOdriveData(std::string, const short);
template int odrive_endpoint::writeOdriveData(std::string, const float);

template int odrive_endpoint::readOdriveData(std::string, uint8_t &);
template int odrive_endpoint::readOdriveData(std::string, uint16_t &);
template int odrive_endpoint::readOdriveData(std::string, uint32_t &);
template int odrive_endpoint::readOdriveData(std::string, uint64_t &);
template int odrive_endpoint::readOdriveData(std::string, int &);
template int odrive_endpoint::readOdriveData(std::string, short &);
template int odrive_endpoint::readOdriveData(std::string, float &);


/**
 *
 *  Execute function on ODrive
 *  @param object Endpoint descriptor for function
 *  @return ODRIVE_OK on success
 *
 */
int odrive_endpoint::execOdriveFunc(const odrive_object& object)
{
    if (object.type.compare("function"))
    {
        ROS_ERROR("* Error invalid type");
        return ODRIVE_ERROR;
    }

    int ret = execFunc(object.id);
    if (ret != LIBUSB_SUCCESS)
    {
        ROS_ERROR("* Error executing %s function", object.name.c_str());
        return ODRIVE_ERROR;
    }

    return ODRIVE_OK;
}

/**
 *
 *  Execute function on ODrive
 *  @param object Endpoint descriptor name for function
 *  @return ODRIVE_OK on success
 *
 */
int odrive_endpoint::execOdriveFunc(std::string object)
{
    int ret;
    odrive_object odo;

    ret = getObjectByName(object, &odo);
    if (ret)
    {
        ROS_ERROR("* Error getting ID");
        return ret;
    }

    return execOdriveFunc(odo);
}



#undef ROS_INFO
#undef ROS_ERROR