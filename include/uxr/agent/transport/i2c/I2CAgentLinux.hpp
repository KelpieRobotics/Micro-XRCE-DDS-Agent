// Copyright 2017-present Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#ifndef UXR_AGENT_TRANSPORT_I2C_I2CAGENTLINUX_HPP_
#define UXR_AGENT_TRANSPORT_I2C_I2CAGENTLINUX_HPP_

#include <uxr/agent/transport/Server.hpp>
#include <uxr/agent/transport/endpoint/I2CEndPoint.hpp>
#include <uxr/agent/transport/stream_framing/StreamFramingProtocol.hpp>
#include <sys/poll.h>

#define DEFAULT_I2C_ADDR "0x01"

namespace eprosima {
namespace uxr {

class I2CAgent : public Server<I2CEndPoint>
{
public:
    I2CAgent(
            char const * dev,
            uint8_t agent_addr,
            Middleware::Kind middleware_kind);

    ~I2CAgent();

    #ifdef UAGENT_DISCOVERY_PROFILE
        bool has_discovery() final { return false; }
    #endif

    #ifdef UAGENT_P2P_PROFILE
        bool has_p2p() final { return false; }
    #endif

private:
    bool init() final;
    bool fini() final;
    bool handle_error(
            TransportRc transport_rc) final;

    bool recv_message(
            InputPacket<I2CEndPoint>& input_packet,
            int timeout,
            TransportRc& transport_rc) final;

    bool send_message(
            OutputPacket<I2CEndPoint> output_packet,
            TransportRc& transport_rc) final;

    ssize_t write_data(
            uint8_t* buf,
            size_t len,
            TransportRc& transport_rc);

    ssize_t read_data(
            uint8_t* buf,
            size_t len,
            int timeout,
            TransportRc& transport_rc);

private:
    const std::string dev_;
    const uint32_t agent_addr_;
    struct pollfd poll_fd_;
    uint8_t buffer_[SERVER_BUFFER_SIZE];
    FramingIO framing_io_;
};

} // namespace uxr
} // namespace eprosima

#endif // UXR_AGENT_TRANSPORT_I2C_I2CAGENTLINUX_HPP_
