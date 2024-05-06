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
// 1. [ X ] Logging
// 2. [ X ] transport_rc
//    - Gets set to ok by default, only change it when error
// 3. [ ] Verify attributes
// 4. [ ] Cleanup comments

// TODO: Optimization
// 1. [ x ] Reduce READ_LENs
//        - Cache the result of READ_LEN - what we actually read
//        - If in the next read cycle we try to read less or equal to what we know is available, skip READ_LEN
// 2. [ ] Remove READ_LEN
//        - Master sends READ_DATA command with number of bytes it's trying to read
//        - Master sends read request
//        - Slave responds with numbers of bytes it actually is going to send (2B) followed by the data (len B)
//        - If slave has less bytes to send than master requires, it pads it with 0s (this is potentially inefficient, however if block transports are supported no padding will be needed)

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
#include <errno.h>

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

        framing_ios_.insert(std::pair<uint8_t, FramingIO>(addr, std::move(temp_framing_io)));
        read_len_available_.insert(std::pair<uint8_t, uint16_t>(addr, 0));
    }
}

I2CAgent::~I2CAgent() = default;

bool I2CAgent::init() {
     bool rv = false;
    
    fd_ = open(dev_.c_str(), O_RDWR);

    if(fd_ >= 0) {
        rv = true;
        UXR_AGENT_LOG_INFO(
            UXR_DECORATE_GREEN("running..."),
            "device: {}, fd: {}",
            dev_, fd_
        );
    }
    else {
        switch(errno) {
            default:
                // Unknown error
                UXR_AGENT_LOG_ERROR(
                    UXR_DECORATE_RED("Unknown error while opening file descriptor"),
                    "device: {}, errno: {}",
                    dev_, errno
                );
                break;
        }
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
        switch(errno) {
            default:
                // Unknown error
                UXR_AGENT_LOG_ERROR(
                    UXR_DECORATE_RED("Unknown error while closing file descriptor"),
                    "device: {}, errno: {}",
                    dev_, errno
                );
                break;
        }
    }
    fd_ = -1;

    return rv;
}

bool I2CAgent::handle_error(TransportRc transport_rc) {
    // TODO: Remove clients from addrs_ when X multiple attempts? Useful for autodetection?

    UXR_AGENT_LOG_INFO(
        "Resetting connection", // TODO: Better log
        "device: {}, transport_rc: {}",
        dev_, transport_rc
    );

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

                UXR_AGENT_LOG_WARN(
                    UXR_DECORATE_YELLOW("Unable to set timeout"),
                    "device: {}, timeout: {}",
                    dev_, timeout
                );
            }
        }
        else return true;
    } 

    return false;
}

ssize_t I2CAgent::read_len(
        uint8_t addr,
        TransportRc& transport_rc)
{
    i2c_msg msgs[2];
    i2c_rdwr_ioctl_data msgset;

    msgs[0].addr = addr;
    msgs[0].flags = 0;
    msgs[0].len = 1;
    msgs[0].buf = new uint8_t[1]{I2CCommands::READ_LEN};
    
    msgs[1].addr = addr;
    msgs[1].flags = I2C_M_RD | I2C_M_NOSTART;
    msgs[1].len = 2;
    msgs[1].buf = new uint8_t[2];

    msgset.msgs = msgs;
    msgset.nmsgs = 2;

    if(ioctl(fd_, I2C_RDWR, &msgset) < 0) {
        switch(errno) {
            case ETIMEDOUT: 
                UXR_AGENT_LOG_ERROR(
                    UXR_DECORATE_RED("Device timed out while reading bytes available"),
                    "address: {}",
                    addr
                );
                transport_rc = TransportRc::timeout_error;
                break;
            default:
                UXR_AGENT_LOG_ERROR(
                    UXR_DECORATE_RED("Unknown error while reading bytes available"),
                    "address: {}, errno: {}",
                    addr, errno
                );
                transport_rc = TransportRc::connection_error;
                break;
        }

        return -1;
    }

    return static_cast<ssize_t>((static_cast<size_t>(msgs[1].buf[0]) << 8) + (static_cast<size_t>(msgs[1].buf[1])));
}

ssize_t I2CAgent::read_data(
        uint8_t addr, 
        uint8_t* buf, 
        size_t len,
        int timeout, 
        TransportRc& transport_rc)
{
    ssize_t bytes_read = 0;
    uint16_t& read_len_available = read_len_available_.at(addr); // TODO: Exception handeling

    struct i2c_msg msgs[2];
    struct i2c_rdwr_ioctl_data msgset;


    set_timeout(timeout);

    if(len > read_len_available) {
        ssize_t len_read = read_len(addr, transport_rc); // TODO: better name

        if(len_read >= 0) {
            size_t ulen_read = static_cast<size_t>(len_read);
            if(ulen_read < len) {
                if(ulen_read >= read_len_available) len = ulen_read;
                else {
                    UXR_AGENT_LOG_WARN(
                        UXR_DECORATE_YELLOW("Client lost some data"),
                        "address: {}, cached size: {}, actual size: {}",
                        addr, read_len_available, ulen_read
                    );
                }
            }

            read_len_available = ulen_read;
        }
        else {
            UXR_AGENT_LOG_ERROR(
                UXR_DECORATE_RED("Unable to communicate with client. Skipping..."),
                "address: {}",
                addr
            );

            return bytes_read;
        }
    }

    msgs[0].addr = addr;
    msgs[0].flags = 0;
    msgs[0].len = 3;
    msgs[0].buf = new uint8_t[3]{I2CCommands::READ_DATA, static_cast<uint8_t>((len & 0xFF00) >> 8), static_cast<uint8_t>(len & 0x00FF)};

    msgs[1].addr = addr;
    msgs[1].flags = I2C_M_RD | I2C_M_NOSTART;
    msgs[1].len = len;
    msgs[1].buf = buf;

    msgset.msgs = msgs;
    msgset.nmsgs = 2;

    if(ioctl(fd_, I2C_RDWR, &msgset) >= 0) {
        // Data retrieval sucessful
        bytes_read = static_cast<ssize_t>(len);
    }
    else {
        // Reading data failed.
        switch(errno) {
            case ETIMEDOUT:
                UXR_AGENT_LOG_ERROR(
                    UXR_DECORATE_RED("Device timed out while reading data"),
                    "address: {}",
                    addr
                );

                transport_rc = TransportRc::timeout_error;
                break;
            default:
                UXR_AGENT_LOG_ERROR(
                    UXR_DECORATE_RED("Unknown error while reading data"),
                    "address: {}, errno: {}",
                    addr, errno
                );
                transport_rc = TransportRc::connection_error;
                break;
        }
    }
    
    return bytes_read;
}

ssize_t I2CAgent::write_data(
        uint8_t addr, 
        uint8_t* buf, 
        size_t len, 
        TransportRc& transport_rc) 
{
    ssize_t bytes_written = 0;

    struct i2c_msg msg;
    struct i2c_rdwr_ioctl_data msgset;

    msg.addr = addr;
    msg.flags = 0;
    msg.len = len+1;
    msg.buf = new uint8_t[len+1]{I2CCommands::WRITE};
    memcpy(msg.buf+1, buf, len);

    msgset.msgs = &msg;
    msgset.nmsgs = 1;

    if(ioctl(fd_, I2C_RDWR, &msgset) >= 0) {
        bytes_written = static_cast<ssize_t>(len);
    }
    else {
        // Writing failed
        switch(errno) {
            case ETIMEDOUT:
                UXR_AGENT_LOG_ERROR(
                    UXR_DECORATE_RED("Device timed out while writing data"),
                    "address: {}",
                    addr
                );

                transport_rc = TransportRc::timeout_error;
                break;
            default:
                UXR_AGENT_LOG_ERROR(
                    UXR_DECORATE_RED("Unknown error while writing data"),
                    "address: {}, errno: {}",
                    addr, errno
                );
                transport_rc = TransportRc::connection_error;
                break;
        }
    }

    return bytes_written; 
}

bool I2CAgent::recv_message(
        std::vector<InputPacket<I2CEndPoint>>& input_packets,
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
            struct InputPacket<I2CEndPoint> temp_packet{}; // TODO: better name
            temp_packet.message.reset(new InputMessage(buffer_, static_cast<size_t>(bytes_read)));
            temp_packet.source = I2CEndPoint(it->first);
            rv = true;

            uint32_t raw_client_key;
            if (Server<I2CEndPoint>::get_client_key(temp_packet.source, raw_client_key))
            {
                UXR_AGENT_LOG_MESSAGE(
                    UXR_DECORATE_YELLOW("[==>> I2C <<==]"),
                    raw_client_key,
                    temp_packet.message->get_buf(),
                    temp_packet.message->get_len());
            }

            input_packets.push_back(std::move(temp_packet));
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
