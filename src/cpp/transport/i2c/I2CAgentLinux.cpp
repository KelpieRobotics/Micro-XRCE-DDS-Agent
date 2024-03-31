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

#include <uxr/agent/transport/i2c/I2CAgentLinux.hpp>
#include <uxr/agent/utils/Conversion.hpp>
#include <uxr/agent/logger/Logger.hpp>

#include <unistd.h>

#include <sys/ioctl.h>

#include <linux/i2c-dev.h>

namespace eprosima {
namespace uxr {

I2CAgent::I2CAgent(
        char const * dev,
        uint8_t agent_addr,
        Middleware::Kind middleware_kind)
    : Server<I2CEndPoint>{middleware_kind}
    , agent_addr_{agent_addr}
    , poll_fd_{}
    , buffer_{0}
    , framing_io_(
          agent_addr,
          std::bind(&I2CAgent::write_data, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
          std::bind(&I2CAgent::read_data, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4))
{}

// TODO: Implement init() and fini()

ssize_t I2CAgent::write_data(
        uint8_t* buf,
        size_t len,
        TransportRc& transport_rc)
{
    size_t rv = 0;
    // FIXME: That's not how to write to a i2c device
    // TODO: fetch remote addr with framing_io_.get_remote_addr()
    ssize_t bytes_written = ::write(poll_fd_.fd, buf, len);
    if (0 < bytes_written)
    {
        rv = size_t(bytes_written);
    }
    else
    {
        transport_rc = TransportRc::server_error;
    }
    return rv;
}

ssize_t I2CAgent::read_data(
        uint8_t* buf,
        size_t len,
        int timeout,
        TransportRc& transport_rc)
{
    ssize_t bytes_read = 0;
    int poll_rv = poll(&poll_fd_, 1, timeout);
    if(poll_fd_.revents & (POLLERR+POLLHUP))
    {
        transport_rc = TransportRc::server_error;;
    }
    else if (0 < poll_rv)
    {
        bytes_read = read(poll_fd_.fd, buf, len);
        if (0 > bytes_read)
        {
            transport_rc = TransportRc::server_error;
        }
    }
    else
    {
        transport_rc = (poll_rv == 0) ? TransportRc::timeout_error : TransportRc::server_error;
    }
    return bytes_read;
}

bool I2CAgent::recv_message(
        InputPacket<I2CEndPoint>& input_packet,
        int timeout,
        TransportRc& transport_rc)
{
    bool rv = false;
    uint8_t remote_addr = 0x00;
    ssize_t bytes_read = 0;

    do
    {
        bytes_read = framing_io_.read_framed_msg(
            buffer_, SERVER_BUFFER_SIZE, remote_addr, timeout, transport_rc);
    }
    while ((0 == bytes_read) && (0 < timeout));

    if (0 < bytes_read)
    {
        input_packet.message.reset(new InputMessage(buffer_, static_cast<size_t>(bytes_read)));
        input_packet.source = I2CEndPoint(remote_addr);
        rv = true;

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
    return rv;
}

bool I2CAgent::send_message(
        OutputPacket<I2CEndPoint> output_packet,
        TransportRc& transport_rc)
{
    bool rv = false;
    ssize_t bytes_written =
            framing_io_.write_framed_msg(
                output_packet.message->get_buf(),
                output_packet.message->get_len(),
                output_packet.destination.get_addr(),
                transport_rc);
    if ((0 < bytes_written) && (
         static_cast<size_t>(bytes_written) == output_packet.message->get_len()))
    {
        rv = true;

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
    return rv;
}

} // namespace uxr
} // namespace eprosima
