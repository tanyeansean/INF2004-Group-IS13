/**
 * @file    lwipopts.h
 * @brief   lwIP configuration for the Pico W (required by the SDK's lwIP build).
 *
 * Based on pico-examples/pico_w/wifi/lwipopts_examples_common.h, with the
 * extra settings the MQTT client needs.
 */
#ifndef LWIPOPTS_H
#define LWIPOPTS_H

/* ---- Core: no OS, raw API only ----------------------------------------- */
#define NO_SYS                      1
#define LWIP_SOCKET                 0
#define LWIP_NETCONN                0
#define MEM_LIBC_MALLOC             0   /* must be 0 for threadsafe_background */
#define MEM_ALIGNMENT               4
#define MEM_SIZE                    8000

/* ---- Buffers ------------------------------------------------------------ */
#define MEMP_NUM_TCP_SEG            32
#define MEMP_NUM_ARP_QUEUE          10
#define PBUF_POOL_SIZE              24

/* ---- Protocols ---------------------------------------------------------- */
#define LWIP_ARP                    1
#define LWIP_ETHERNET               1
#define LWIP_ICMP                   1   /* lets you ping the car */
#define LWIP_RAW                    1
#define LWIP_IPV4                   1
#define LWIP_TCP                    1
#define LWIP_UDP                    1
#define LWIP_DNS                    1
#define LWIP_DHCP                   1
#define DHCP_DOES_ARP_CHECK         0
#define LWIP_DHCP_DOES_ACD_CHECK    0

/* ---- TCP ---------------------------------------------------------------- */
#define TCP_MSS                     1460
#define TCP_WND                     (8 * TCP_MSS)
#define TCP_SND_BUF                 (8 * TCP_MSS)
#define TCP_SND_QUEUELEN            ((4 * (TCP_SND_BUF) + (TCP_MSS - 1)) / (TCP_MSS))
#define LWIP_TCP_KEEPALIVE          1

/* ---- Netif -------------------------------------------------------------- */
#define LWIP_NETIF_STATUS_CALLBACK  1
#define LWIP_NETIF_LINK_CALLBACK    1
#define LWIP_NETIF_HOSTNAME         1
#define LWIP_NETIF_TX_SINGLE_PBUF   1
#define LWIP_CHKSUM_ALGORITHM       3

/* ---- Stats / debug off (saves RAM and CPU) ------------------------------ */
#define MEM_STATS                   0
#define SYS_STATS                   0
#define MEMP_STATS                  0
#define LINK_STATS                  0

/* ---- MQTT --------------------------------------------------------------- */
/* The MQTT client needs one extra lwIP timer slot for its keep-alive timer. */
#define MEMP_NUM_SYS_TIMEOUT        (LWIP_NUM_SYS_TIMEOUT_INTERNAL + 1)
/* Outgoing buffer: must hold a few telemetry messages queued back-to-back.  */
#define MQTT_OUTPUT_RINGBUF_SIZE    2048
/* Holds incoming topic + header; our topics are short.                      */
#define MQTT_VAR_HEADER_BUFFER_LEN  256
/* QoS 1 publishes/subscribes waiting for an ACK at the same time.           */
#define MQTT_REQ_MAX_IN_FLIGHT      8

#endif /* LWIPOPTS_H */
