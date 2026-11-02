#ifndef FAKE_IP_H
#define FAKE_IP_H
#include <stdint.h>
typedef struct { uint32_t addr; int type; } ip_addr_t;
#define ip_2_ip4(a) (a)
#define IP_SET_TYPE(a, t) ((a)->type = (t))
#define IPADDR_TYPE_V4 0
int ip4addr_aton(const char *cp, ip_addr_t *addr);
#endif
