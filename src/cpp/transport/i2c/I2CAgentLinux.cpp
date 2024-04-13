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
// 2. [ ] Queue optimazations (if device available but had nothing to send, prioritize it in the next read cycle)
// 3. [ ] Add CRC
// 4. [ ] Add possibility for framing
// 5. [ ] SMBUS?

// TODO: General TODOs
// 1. [ ] Logging
// 2. [ ] transport_rc
// 3. [ ] Verify attributes

#include <uxr/agent/transport/i2c/I2CAgentLinux.hpp>
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

const uint8_t I2CAgent::buffer_len_[2] = {static_cast<uint8_t>((SERVER_BUFFER_SIZE & 0xFF00) >> 8), static_cast<uint8_t>(SERVER_BUFFER_SIZE & 0x00FF)};

I2CAgent::I2CAgent(
        char const * dev,
        std::vector<uint8_t> addrs,
        Middleware::Kind middleware_kind)
    : Server<I2CEndPoint>{middleware_kind}
    , dev_{dev}
    , addrs_{addrs}
    , last_addr_(0)
    , fd_(-1)
    , buffer_{0}
{
    for (uint8_t element : addrs)
    {
        addrs_.push_back(element);
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
    REAT_DATA   = 0x03
};

bool I2CAgent::recv_message(
        InputPacket<I2CEndPoint>& input_packet,
        int timeout,
        TransportRc& transport_rc)
{
    bool rv = false, timeout_en = timeout > 0;
    uint8_t remote_addr, bytes_read;
    uint8_t cmnd_buf[3], len_buf[2];
    uint16_t len;
    std::chrono::time_point<std::chrono::system_clock> timer_start, timer_end = std::chrono::system_clock::now();

    struct i2c_msg msgs[4];
    struct i2c_rdwr_ioctl_data msgset;

    // Set command to len req
    msgs[0].flags = 0;
    msgs[0].buf = cmnd_buf;
    msgs[0].len = 1;

    // Get len
    msgs[1].flags = I2C_M_RD | I2C_M_NOSTART;
    msgs[1].buf = len_buf;
    msgs[1].len = 2;

    // Set command to data req
    msgs[2].flags = 0;
    msgs[2].buf = cmnd_buf;
    msgs[2].len = 3;

    // Get data
    msgs[3].flags = I2C_M_RD | I2C_M_NOSTART;
    msgs[3].buf = buffer_;

    msgset.nmsgs = 2;

    do { // If no bytes have been read and timeout hasn't ran out
        timer_start = timer_end;
        
        // Try reading len from slave
        msgs[1].addr = msgs[0].addr = addrs_[last_addr_];
        cmnd_buf[0] = I2CCommands::READ_LEN;
        msgset.msgs = msgs;
        if(ioctl(fd_, I2C_RDWR, &msgset) == 0 && len_buf[0] != 0 && len_buf[1] != 1) {
            // Slave responded and has data to send
            // Try reading data from slave

            if(len_buf[0] > buffer_len_[0] || (len_buf[0] == buffer_len_[0] && len_buf[1] > buffer_len_[1])) {
                len_buf[0] = buffer_len_[0];
                len_buf[1] = buffer_len_[1];
            }

            len = (static_cast<uint16_t>(len_buf[0]) << 8) + (static_cast<uint16_t>(len_buf[1]));

            msgs[3].addr = msgs[2].addr = remote_addr = addrs_[last_addr_];
            cmnd_buf[0] = I2CCommands::REAT_DATA;
            cmnd_buf[1] = len_buf[0];
            cmnd_buf[2] = len_buf[1];
            msgs[3].len = len;
            msgset.msgs += 2;
            if(ioctl(fd_, I2C_RDWR, &msgset) == 0) {
                // Data retrieval sucessful
                bytes_read = len;
            }
            else {
                // Reading data failed. Skipping...
                // TODO: Break?
                // TODO: Logging
            }
        }
        else {
            // Slave is unresponsive. Potentially offline
            // TODO: Break?
            // TODO: Logging
        }

        last_addr_ = last_addr_ >= addrs_.size()-1 ? 0 : last_addr_++;

        timer_end = std::chrono::system_clock::now();
        timeout = timeout_en ? timeout-std::chrono::duration_cast<std::chrono::milliseconds>(
            timer_end - timer_start).count() : 0;
    } while(bytes_read <= 0 && timeout >= 0);

    if (0 < bytes_read)
    {
        input_packet.message.reset(new InputMessage(buffer_, static_cast<size_t>(bytes_read)));
        input_packet.source = I2CEndPoint(remote_addr);
        rv = true;
        transport_rc = TransportRc::ok;

        uint32_t raw_client_key;
        if (Server<I2CEndPoint>::get_client_key(input_packet.source, raw_client_key))
        {
            UXR_AGENT_LOG_MESSAGE(
                UXR_DECORATE_YELLOW("[==>> I2C <<==]"),
                raw_client_key,
                input_packet.message->get_buf(),
                input_packet.message->get_len());
        }
    }
    else {
        transport_rc = timeout < 0 ? TransportRc::timeout_error : TransportRc::connection_error;
    }

    return rv;
}

bool I2CAgent::send_message(
        OutputPacket<I2CEndPoint> output_packet,
        TransportRc& transport_rc)
{
    bool rv = false;
    printf("%ld", output_packet.message->get_len()); // TODO: Remove debug printing

    struct i2c_msg msg;
    struct i2c_rdwr_ioctl_data msgset;

    uint8_t data_buf[output_packet.message->get_len()+3];
    data_buf[0] = I2CCommands::WRITE;
    data_buf[1] = static_cast<uint8_t>((SERVER_BUFFER_SIZE & 0xFF00) >> 8);
    data_buf[2] = static_cast<uint8_t>((SERVER_BUFFER_SIZE & 0x00FF));
    memcpy(data_buf+3, output_packet.message->get_buf(), output_packet.message->get_len()); // FIXME: Probably can be optimized
    // TODO: CRC-16
    // TODO: Split into multiple transmissions so CRC checks happen more often

    msg.addr = output_packet.destination.get_addr();
    msg.flags = 0;
    msg.len = output_packet.message->get_len()+3;
    msg.buf = data_buf;

    msgset.msgs = &msg;
    msgset.nmsgs = 1;

    if(ioctl(fd_, I2C_RDWR, &msgset) == 0) {
        rv = true;
        transport_rc = TransportRc::ok;

        uint32_t raw_client_key;
        if (Server<I2CEndPoint>::get_client_key(output_packet.destination, raw_client_key))
        {
            UXR_AGENT_LOG_MESSAGE(
                UXR_DECORATE_YELLOW("[** <<I2C>> **]"),
                raw_client_key,
                output_packet.message->get_buf(),
                output_packet.message->get_len());
        }
    }
    else {
        UXR_AGENT_LOG_ERROR(
            UXR_DECORATE_RED("Error sending packet"),
            "addr: {}, errno: {}",
            output_packet.destination.get_addr(), errno);

            transport_rc = TransportRc::connection_error;
    }
    
    return rv;
}

} // namespace uxr
} // namespace eprosima
