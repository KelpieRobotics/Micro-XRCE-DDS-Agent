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

#ifndef _UXR_AGENT_CONFIG_HPP_
#define _UXR_AGENT_CONFIG_HPP_

#include <stdint.h>
#include <chrono>

namespace eprosima {
namespace uxr {

#define UAGENT_FAST_PROFILE
#define UAGENT_CED_PROFILE
#define UAGENT_DISCOVERY_PROFILE
#ifdef UAGENT_CED_PROFILE
#define UAGENT_P2P_PROFILE
#endif
#define UAGENT_SOCKETCAN_PROFILE
#define UAGENT_LOGGER_PROFILE

const uint16_t DISCOVERY_PORT = 7400;
const char* const DISCOVERY_IP = "239.255.0.2";

const uint16_t RELIABLE_STREAM_DEPTH = 16;
static_assert (RELIABLE_STREAM_DEPTH > 0, "RELIABLE_STREAM_DEPTH shall be greater than 0.");

const uint16_t BEST_EFFORT_STREAM_DEPTH = 16;
static_assert (RELIABLE_STREAM_DEPTH > 0, "BEST_EFFORT_STREAM_DEPTH shall be greater than 0.");

const uint16_t HEARTBEAT_PERIOD = 200;
const uint16_t TCP_MAX_CONNECTIONS = 100;
const uint16_t TCP_MAX_BACKLOG_CONNECTIONS = 100;
const uint16_t SERVER_QUEUE_MAX_SIZE = 32000;

constexpr std::chrono::milliseconds CLIENT_DEAD_TIME{30000};

const uint16_t SERVER_BUFFER_SIZE = 65535;

#define UAGENT_TWEAK_XRCE_WRITE_LIMIT

} // namespace uxr
} // namespace eprosima

#endif //_UXR_AGENT_CONFIG_HPP_
