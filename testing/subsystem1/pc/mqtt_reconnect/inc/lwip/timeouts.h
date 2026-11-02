#ifndef FAKE_TO_H
#define FAKE_TO_H
#include "lwip/apps/mqtt.h"
typedef void (*sys_timeout_handler)(void *arg);
void sys_timeout(u32_t msecs, sys_timeout_handler handler, void *arg);
void sys_untimeout(sys_timeout_handler handler, void *arg);
u32_t sys_now(void);
#endif
