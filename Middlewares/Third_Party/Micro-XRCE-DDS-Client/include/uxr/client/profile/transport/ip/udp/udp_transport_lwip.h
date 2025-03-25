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

#ifndef _UXR_CLIENT_UDP_TRANSPORT_LWIP_H_
#define _UXR_CLIENT_UDP_TRANSPORT_LWIP_H_

#ifdef __cplusplus
extern "C"
{
#endif // ifdef __cplusplus

#include "lwip/udp.h"
#include "lwip/sockets.h"
#include "cringbuf.h"

typedef struct uxrUDPPlatform
{
    struct udp_pcb* pcb;
    size_t tx_pbuf_capacity;
    ip_addr_t server_ip;
    uint16_t server_port;
    ringbuf_t rx_ringbuf;
    uint8_t rx_buf[1024];
} uxrUDPPlatform;

#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif //_UXR_CLIENT_UDP_TRANSPORT_LWIP_H_
