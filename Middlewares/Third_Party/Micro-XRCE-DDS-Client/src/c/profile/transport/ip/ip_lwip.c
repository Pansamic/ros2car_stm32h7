#include <string.h>
#include <uxr/client/profile/transport/ip/ip.h>
#include <uxr/client/config.h>

#include "lwip.h"
#include "lwip/api.h"
#include "lwip/sockets.h"

bool uxr_ip_to_locator(
        char const* ip,
        uint16_t port,
        uxrIpProtocol ip_protocol,
        TransportLocator* locator)
{
    bool result = false;
    switch (ip_protocol)
    {
        case UXR_IPv4:
            locator->format = ADDRESS_FORMAT_MEDIUM;
            locator->_.medium_locator.locator_port = port;
            ip4_addr_t ip4;
            ip4addr_aton(ip, &ip4);
            memcpy(&locator->_.medium_locator.address, &ip4.addr, sizeof(locator->_.medium_locator.address));
            if(locator->_.medium_locator.address[0] != 0)
            {
                result = true;
            }
            else
            {
                result = false;
            }
            break;
        case UXR_IPv6:
            break;
        default:
            break;
    }
    return result;
}

bool uxr_locator_to_ip(
        TransportLocator const* locator,
        char* ip,
        size_t size,
        uint16_t* port,
        uxrIpProtocol* ip_protocol)
{
    bool result = false;
    (void)size;
    switch (locator->format)
    {
        case ADDRESS_FORMAT_MEDIUM:
            *port = locator->_.medium_locator.locator_port;
            *ip_protocol = UXR_IPv4;

            // Convert binary IPv4 address to string
            ip4_addr_t ip4;
            memcpy(&ip4, locator->_.medium_locator.address, sizeof(ip4_addr_t));

            // Use ipaddr_ntoa to convert to string
            const char* ip_str = ip4addr_ntoa(&ip4);
            if (ip_str != NULL)
            {
                size_t str_len = strlen(ip_str);
                snprintf(ip, str_len + 1, "%s", ip_str);
                result = true;
            }
            break;
        case ADDRESS_FORMAT_LARGE:
            break;
        default:
            break;
    }
    return result;
}