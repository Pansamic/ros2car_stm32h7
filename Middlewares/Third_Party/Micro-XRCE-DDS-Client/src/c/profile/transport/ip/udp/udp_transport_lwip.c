#include <string.h>
#include <uxr/client/profile/transport/ip/udp/udp_transport_lwip.h>
#include "udp_transport_internal.h"
#include "syslog.h"

static void lwip_udp_recv_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p,
    const ip_addr_t *addr, u16_t port)
{
    (void)addr;
    (void)port;
    uxrUDPPlatform* platform = (uxrUDPPlatform*)arg;
    ringbuf_ret_t ringbuf_ret = RINGBUF_OK;
    if (p == NULL)
    {
        // Connection closed by remote host
        LOG_ERROR("Connection closed by server.\n");
        udp_disconnect(pcb);
        udp_remove(pcb);
        return ;
    }
    // NVIC_DisableIRQ(ETH_IRQn);
    for (struct pbuf *q = p; q != NULL; q = q->next)
    {
        ringbuf_ret = ringbuf_write_block(&platform->rx_ringbuf, q->payload, q->len);
        if(ringbuf_ret != RINGBUF_OK)
        {
            LOG_ERROR("lwip udp client ringbuf write error.\n");
            pbuf_free(p);
            return;
        }
    }
    // NVIC_EnableIRQ(ETH_IRQn);

    pbuf_free(p);
}

bool uxr_init_udp_platform(
        uxrUDPPlatform* platform,
        uxrIpProtocol ip_protocol,
        const char* ip,
        const char* port)
{
    (void) ip_protocol;
    platform->server_port = (uint16_t)atoi(port);

    err_t lwip_err = ERR_OK;
    ringbuf_ret_t ringbuf_ret = RINGBUF_OK;
    ringbuf_ret = ringbuf_init(&platform->rx_ringbuf, platform->rx_buf, sizeof(platform->rx_buf), RINGBUF_RULE_DISCARD);
    if(ringbuf_ret != RINGBUF_OK)
    {
        LOG_ERROR("uxr udp platform init error: ringbuf creation failed.\n");
    }
    platform->pcb = udp_new();
    if(platform->pcb == NULL)
    {
        LOG_ERROR("uxr udp platform init error: lwip udp pcb creation failed.\n");
        return false;
    }
    if(!ip4addr_aton(ip, &platform->server_ip))
    {
        LOG_ERROR("uxr udp platform init error: lwip udp convert ipv4 address failed.\n");
        return false;
    }

    udp_bind(platform->pcb, IP_ADDR_ANY, 40001);
    udp_recv(platform->pcb, lwip_udp_recv_callback, platform);
    lwip_err = udp_connect(platform->pcb, &platform->server_ip, platform->server_port);
    if(lwip_err != ERR_OK)
    {
        LOG_ERROR("uxr udp platform init error: lwip udp connect failed.\n");
        return false;
    }
    return true;
}

bool uxr_close_udp_platform(
        uxrUDPPlatform* platform)
{
    udp_disconnect(platform->pcb);
    udp_remove(platform->pcb);
    return true;
}

size_t uxr_write_udp_data_platform(
        uxrUDPPlatform* platform,
        const uint8_t* buf,
        size_t len,
        uint8_t* errcode)
{
    size_t written_len = 0;
    err_t lwip_err = ERR_OK;
    struct pbuf* tx_pbuf = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
    memset(tx_pbuf->payload, 0, len);
    written_len = len;
    memcpy(tx_pbuf->payload, buf, written_len);
    lwip_err = udp_sendto(platform->pcb, tx_pbuf, &platform->server_ip, platform->server_port);
    pbuf_free(tx_pbuf);
    if(lwip_err != ERR_OK)
    {
        *errcode = 1;
    }
    return written_len;
}

size_t uxr_read_udp_data_platform(
        uxrUDPPlatform* platform,
        uint8_t* buf,
        size_t len,
        int timeout,
        uint8_t* errcode)
{
    (void)timeout;
    size_t rv = 0;
    ringbuf_ret_t ringbuf_ret = RINGBUF_OK;
    ringbuf_ret = ringbuf_get_block(&platform->rx_ringbuf, buf, len, &rv);
    if(ringbuf_ret != RINGBUF_OK)
    {
        *errcode = 1;
    }
    return rv;
}


// bool uxr_init_udp_platform(
//         uxrUDPPlatform* platform,
//         uxrIpProtocol ip_protocol,
//         const char* ip,
//         const char* port)
// {
//     (void) ip_protocol;
//     uint16_t iport = (uint16_t)atoi(port);

//     platform->fd = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
//     if(platform->fd < 0)
//     {
//         return false;
//     }

//     memset(&platform->client_addr, 0, sizeof(platform->client_addr));
//     platform->client_addr.sin_family = AF_INET;
//     platform->client_addr.sin_port = htons(40001);
//     platform->client_addr.sin_addr.s_addr = INADDR_ANY;

//     if(lwip_bind(platform->fd, (struct sockaddr *)&platform->client_addr, sizeof(platform->client_addr)) < 0)
//     {
//         lwip_close(platform->fd);
//         return false;
//     }

//     memset(&platform->server_addr, 0, sizeof(platform->server_addr));
//     platform->server_addr.sin_family = AF_INET;
//     platform->server_addr.sin_port = htons(iport);
//     platform->server_addr.sin_addr.s_addr = inet_addr(ip);

//     return true;
// }

// bool uxr_close_udp_platform(
//         uxrUDPPlatform* platform)
// {
//     lwip_close(platform->fd);
//     return true;
// }

// size_t uxr_write_udp_data_platform(
//         uxrUDPPlatform* platform,
//         const uint8_t* buf,
//         size_t len,
//         uint8_t* errcode)
// {
//     size_t rv = 0;

//     int32_t bytes_sent = lwip_sendto(platform->fd, buf, len, 0, 
//         (struct sockaddr *)&platform->server_addr, sizeof(platform->server_addr));

//     if (0 < bytes_sent)
//     {
//         rv = (size_t)bytes_sent;
//         *errcode = 0;
//     }
//     else
//     {
//         *errcode = 1;
//     }

//     return rv;
// }

// size_t uxr_read_udp_data_platform(
//         uxrUDPPlatform* platform,
//         uint8_t* buf,
//         size_t len,
//         int timeout,
//         uint8_t* errcode)
// {
//     size_t rv = 0;

//     int32_t bytes_received = lwip_recv(platform->fd, buf, len, 0);
//     if (0 <= bytes_received)
//     {
//         rv = (size_t)bytes_received;
//         *errcode = 0;
//     }
//     else
//     {
//         *errcode = 1;
//     }

//     return rv;
// }
