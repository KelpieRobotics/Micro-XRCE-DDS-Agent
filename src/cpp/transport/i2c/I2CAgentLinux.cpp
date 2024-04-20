// Copyright 2018 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// TODO: Implementation Stages
// 1. [ x ] Send Raw as is
// 2. [ x ] Framing
// 3. [ ] SMBUS?

// TODO: General TODOs
// 1. [ ] Logging
// 2. [ ] transport_rc TODO: current
// 3. [ ] Verify attributes

#include <uxr/agent/transport/i2c/I2CAgentLinux.hpp>
#include <uxr/agent/transport/stream_framing/StreamFramingProtocol.hpp>
#include <uxr/agent/utils/Conversion.hpp>
#include <uxr/agent/logger/Logger.hpp>

#include <unistd.h>
#include <fcntl.h>
#include <chrono>

#include <sys/ioctl.h>

#include <linux/i2c-dev.h>
#include <linux/i2c.h>

namespace eprosima {
namespace uxr {

I2CAgent::I2CAgent(
        char const * dev,
        std::vector<uint8_t> addrs,
        Middleware::Kind middleware_kind)
    : Server<I2CEndPoint>{middleware_kind}
    , dev_{dev}
    , framing_ios_{}
    , fd_(-1)
    , buffer_{0}
    , timeout_(-1)
{
    for (uint8_t addr : addrs)
    {
        FramingIO temp_framing_io( // TODO: better names
            0,
            std::bind(&I2CAgent::write_data, this, addr, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
            std::bind(&I2CAgent::read_data, this, addr, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4)
        );

        framing_ios_.insert(std::pair<uint8_t, FramingIO>(addr, temp_framing_io));
    }
}

I2CAgent::~I2CAgent() = default;

bool I2CAgent::init() {
     bool rv = false;
    
    fd_ = open(dev_.c_str(), O_RDWR);

    if(fd_ >= 0) {
        UXR_AGENT_LOG_INFO(
            UXR_DECORATE_GREEN("running..."),
            "device: {}, fd: {}",
            dev_, fd_
        );
    }
    else {
        UXR_AGENT_LOG_ERROR(
            UXR_DECORATE_RED("Cannot open I2C device file."),
            "device: {}, errno: {}",
            dev_, errno
        );
    }

    return rv;
}

bool I2CAgent::fini() {
    if(fd_ < 0) return true;

    bool rv = false;

    if(close(fd_) == 0) {
        UXR_AGENT_LOG_INFO(
            UXR_DECORATE_GREEN("server stopped"),
            "fd: {}, device: {}",
            fd_, dev_
        );
        
        rv = true;
    }
    else {
        UXR_AGENT_LOG_ERROR(
            UXR_DECORATE_RED("close server error"),
            "fd: {}, device: {}, errno: {}",
            fd_, dev_, errno
        );
    }
    fd_ = -1;

    return rv;
}

bool I2CAgent::handle_error(TransportRc transport_rc) {
    // TODO: Remove clients from addrs_ when X multiple attempts? Useful for autodetection?
    
    return fini() && init();
}

enum I2CCommands : uint8_t {
    NOP         = 0x00,
    WRITE       = 0x01,
    READ_LEN    = 0x02,
    READ_DATA   = 0x03
};

bool I2CAgent::set_timeout(int timeout) {
    if(timeout > 0) {
        if(timeout != timeout_) {
            if(ioctl(fd_, I2C_TIMEOUT, timeout/10) == 0) {
                timeout_ = timeout;

                return true;
            }
            else {
                timeout_ = -1;

                // TODO: Logging
            }
        }
        else return true;
    } 

    return false;
}

ssize_t I2CAgent::read_data(
        uint8_t addr, 
        uint8_t* buf, 
        size_t len,
        int timeout, 
        TransportRc transport_rc)
{
    ssize_t bytes_read = 0;

    uint8_t avail_len[2], buf_len[2] = {static_cast<uint8_t>((len & 0xFF00) >> 8), static_cast<uint8_t>(len & 0x00FF)}; // TODO: better names

    struct i2c_msg len_request_msgs[2], data_request_msgs[2];
    struct i2c_rdwr_ioctl_data len_request_msgset, data_request_msgset;

    // Prepare messages
    //// Len request messages
    len_request_msgs[0].addr = addr;
    len_request_msgs[0].flags = 0;
    len_request_msgs[0].len = 1;
    len_request_msgs[0].buf = new uint8_t[1]{I2CCommands::READ_LEN};
    
    len_request_msgs[1].addr = addr;
    len_request_msgs[1].flags = I2C_M_RD | I2C_M_NOSTART;
    len_request_msgs[1].len = 2;
    len_request_msgs[1].buf = avail_len;

    len_request_msgset.msgs = len_request_msgs;
    len_request_msgset.nmsgs = 2;

    //// Data request messages
    data_request_msgs[0].addr = addr;
    data_request_msgs[0].flags = 0;
    data_request_msgs[0].len = 3;
    data_request_msgs[0].buf = new uint8_t[3]{I2CCommands::READ_DATA};

    data_request_msgs[1].addr = addr;
    data_request_msgs[1].flags = I2C_M_RD | I2C_M_NOSTART;
    data_request_msgs[1].buf = buf;

    data_request_msgset.msgs = data_request_msgs;
    data_request_msgset.nmsgs = 2;
    
    // Set timeout
    set_timeout(timeout);

    // Try reading len from a slave
    if(ioctl(fd_, I2C_RDWR, &len_request_msgset) == 0) {
        // Slave responded

        if(avail_len[0] != 0 && avail_len[1] != 0) {
            // Only receive up to len bytes
            if(avail_len[0] > buf_len[0] || (avail_len[0] == buf_len[0] && avail_len[1] > buf_len[1])) {
                avail_len[0] = buf_len[0];
                avail_len[1] = buf_len[1];
            }

            // Reconstruct len to read
            len = (static_cast<size_t>(avail_len[0]) << 8) + (static_cast<size_t>(avail_len[1]));

            data_request_msgs[0].buf[1] = avail_len[0];
            data_request_msgs[0].buf[2] = avail_len[1];
            data_request_msgs[1].len = len;
            if(ioctl(fd_, I2C_RDWR, &data_request_msgset) == 0) {
                // Data retrieval sucessful
                bytes_read = static_cast<ssize_t>(len);
                transport_rc = TransportRc::ok;
            }
            else {
                // Reading data failed.
                // TODO: Logging
                // TODO: transport_rc
                switch(errno) {
                    // TODO: handle errors
                    default:
                        // Unknown error
                        break;
                }
            }
        }
        else {
            // No data to read
            // TODO: Logging
            // TODO: transport_rc
        }
    }
    else {
        // Slave is unresponsive. Potentially offline
        // TODO: Logging
        // TODO: transport_rc
        switch(errno) {
            // TODO: handle errors
            default:
                // Unknown error
                break;
        }
    }
    
    return bytes_read;
}

ssize_t I2CAgent::write_data(
        uint8_t addr, 
        uint8_t* buf, 
        size_t len, 
        TransportRc transport_rc) 
{
    ssize_t bytes_written = 0;

    struct i2c_msg msg;
    struct i2c_rdwr_ioctl_data msgset;

    msg.addr = addr;
    msg.flags = 0;
    msg.len = len;
    msg.buf = buf;

    msgset.msgs = &msg;
    msgset.nmsgs = 1;

    if(ioctl(fd_, I2C_RDWR, &msgset) == 0) {
        bytes_written = static_cast<ssize_t>(len);
        transport_rc = TransportRc::ok;
    }
    else {
        // Writing failed
        // TODO: Logging
        // TODO: transport_rc
        switch(errno) {
            // TODO: handle errors (like early NACK)
            default:
                // Unknown error
                break;
        }
    }

    return bytes_written; 
}

bool I2CAgent::recv_message(
        std::vector<InputPacket<I2CEndPoint>>& input_packet,
        int timeout,
        TransportRc& transport_rc)
{
    bool rv = false;

    for(auto it = framing_ios_.begin(); it != framing_ios_.end(); it++) {
        uint8_t remote_addr = 0x00;
        ssize_t bytes_read = 0;

        do {
            bytes_read = it->second.read_framed_msg(
                buffer_, sizeof (buffer_), remote_addr, timeout, transport_rc);
        } while (bytes_read <= 0 && timeout > 0);

        if(bytes_read > 0) {
            struct InputPacket<I2CEndPoint> temp_packet{};
            temp_packet.message.reset(new InputMessage(buffer_, static_cast<size_t>(bytes_read)));
            temp_packet.source = I2CEndPoint(it->first);
            rv = true;
            transport_rc = TransportRc::ok;

            uint32_t raw_client_key;
            if (Server<I2CEndPoint>::get_client_key(temp_packet.source, raw_client_key))
            {
                UXR_AGENT_LOG_MESSAGE(
                    UXR_DECORATE_YELLOW("[==>> I2C <<==]"),
                    raw_client_key,
                    temp_packet.message->get_buf(),
                    temp_packet.message->get_len());
            }

            input_packet.push_back(std::move(temp_packet));
        }
    }

    return rv;
}

bool I2CAgent::send_message(
        OutputPacket<I2CEndPoint> output_packet,
        TransportRc& transport_rc)
{
    bool rv = false;
    uint8_t remote_addr = 0x00;
    std::map<uint8_t, FramingIO>::iterator it = framing_ios_.find(output_packet.destination.get_addr());

    if(it == framing_ios_.end()) {
        transport_rc = TransportRc::server_error;
        
        return rv;
    }

    ssize_t bytes_written =
            it->second.write_framed_msg(
                output_packet.message->get_buf(),
                output_packet.message->get_len(),
                remote_addr,
                transport_rc);

    if(bytes_written > 0 && static_cast<size_t>(bytes_written) == output_packet.message->get_len()) {
        rv = true;
        transport_rc = TransportRc::ok; // FIXME: here or in the transport callback?

        uint32_t raw_client_key;
        if (Server<I2CEndPoint>::get_client_key(output_packet.destination, raw_client_key))
        {
            UXR_MULTIAGENT_LOG_MESSAGE(
                UXR_DECORATE_YELLOW("[** <<I2C>> **]"),
                raw_client_key,
                output_packet.destination.get_addr(),
                output_packet.message->get_buf(),
                output_packet.message->get_len());
        }
    }

    return rv;
}

} // namespace uxr
} // namespace eprosima
