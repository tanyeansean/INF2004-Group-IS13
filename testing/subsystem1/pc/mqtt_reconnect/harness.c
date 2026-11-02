#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/timeouts.h"
#include "tm/tmonitor.h"
#include "cyw43_utk.h"
void mqtt_utk_start(void);

static u32_t now;
u32_t sys_now(void) { return now; }
/* lwIP-like timer list: sys_untimeout removes the FIRST match only */
typedef struct { sys_timeout_handler fn; void *arg; u32_t due; int on; } tmr_t;
static tmr_t tmrs[64]; static int max_live, lost_timers;
#define POOL 4   /* spare slots, as MEMP_NUM_SYS_TIMEOUT = internal + 4 */
static int live(void) { int n = 0; for (int i = 0; i < 64; i++) n += tmrs[i].on; return n; }
void sys_timeout(u32_t ms, sys_timeout_handler fn, void *arg) {
  if (live() >= POOL) { lost_timers++; return; }          /* NDEBUG: silently dropped */
  for (int i = 0; i < 64; i++) if (!tmrs[i].on) { tmrs[i] = (tmr_t){fn, arg, now + ms, 1}; break; }
  if (live() > max_live) max_live = live();
}
void sys_untimeout(sys_timeout_handler fn, void *arg) {
  for (int i = 0; i < 64; i++) if (tmrs[i].on && tmrs[i].fn == fn && tmrs[i].arg == arg) { tmrs[i].on = 0; return; }
}
static void check_timeouts(void) {
  for (int i = 0; i < 64; i++) if (tmrs[i].on && (int32_t)(now - tmrs[i].due) >= 0) { tmrs[i].on = 0; tmrs[i].fn(tmrs[i].arg); }
}
int ip4addr_aton(const char *cp, ip_addr_t *a) { (void)cp; a->addr = 1; return 1; }
static int lines;
int tm_printf(const UB *fmt, ...) { lines++; if (getenv("VERBOSE")) { va_list ap; va_start(ap, fmt); printf("  [%6u] ", now); vprintf((const char*)fmt, ap); va_end(ap);} return 0; }
int tm_sprintf(UB *s, const UB *fmt, ...) { va_list ap; va_start(ap, fmt); int n = vsprintf((char*)s, (const char*)fmt, ap); va_end(ap); return n; }
void cyw43_utk_get_status(T_CYW43_UTK_STATUS *st) { memset(st, 0, sizeof(*st)); uint8_t m[6] = {0x28,0xcd,0xc1,0x0a,0xb2,0x3f}; memcpy(st->mac, m, 6); }

/* fake MQTT client */
enum { DISC, CONNING, CONN };
struct mqtt_client_s { int st; mqtt_connection_cb_t cb; u32_t outcome_at; };
static u32_t fail_ms = 1000; static struct mqtt_client_s cl; static int connects, isconn; static int broker_up = 1; static char last_id[64];
mqtt_client_t *mqtt_client_new(void) { return &cl; }
u8_t mqtt_client_is_connected(mqtt_client_t *c) { return c->st == CONN; }
err_t mqtt_client_connect(mqtt_client_t *c, const void *ip, u16_t port, mqtt_connection_cb_t cb, void *arg,
                          const struct mqtt_connect_client_info_t *info) {
  (void)ip; (void)port; (void)arg;
  if (c->st != DISC) { isconn++; return ERR_ISCONN; }
  connects++; strcpy(last_id, info->client_id); c->st = CONNING; c->cb = cb;
  c->outcome_at = now + (broker_up ? 50 : fail_ms); return ERR_OK;
}
static void mqtt_events(void) {
  if (cl.st == CONNING && (int32_t)(now - cl.outcome_at) >= 0) {
    if (broker_up) { cl.st = CONN; cl.cb(&cl, NULL, MQTT_CONNECT_ACCEPTED); }
    else { cl.st = DISC; cl.cb(&cl, NULL, MQTT_CONNECT_TIMEOUT); }
  }
}
static void drop(void) { if (cl.st != DISC) { cl.st = DISC; cl.cb(&cl, NULL, MQTT_CONNECT_DISCONNECTED); } }
void mqtt_set_inpub_callback(mqtt_client_t *c, mqtt_incoming_publish_cb_t a, mqtt_incoming_data_cb_t b, void *d) { (void)c;(void)a;(void)b;(void)d; }
err_t mqtt_subscribe(mqtt_client_t *c, const char *t, u8_t q, mqtt_request_cb_t cb, void *a) { (void)c;(void)t;(void)q;(void)cb;(void)a; return ERR_OK; }
err_t mqtt_publish(mqtt_client_t *c, const char *t, const void *p, u16_t l, u8_t q, u8_t r, mqtt_request_cb_t cb, void *a) { (void)c;(void)t;(void)p;(void)l;(void)q;(void)r;(void)cb;(void)a; return ERR_OK; }
/* app hooks: app mode, empty queue */
int app_mqtt_will(char *t, int m, unsigned char *msg) { snprintf(t, m, "x/status"); msg[0] = 0x10; msg[1] = 0; return 1; }
int app_mqtt_subscription(int i, char *t, int m) { if (i < 2) { snprintf(t, m, "x/%d", i); return 1; } return 0; }
int app_mqtt_tx_pop(char *t, int tm, unsigned char *d, int dm, int *q, int *r) { (void)t;(void)tm;(void)d;(void)dm;(void)q;(void)r; return -1; }
void app_mqtt_rx(const char *t, const unsigned char *d, int l) { (void)t;(void)d;(void)l; }
void app_mqtt_link(int c) { (void)c; }
void app_mqtt_publish_failed(void) { }

static void run_until(u32_t t) { while (now < t) { now += 2; check_timeouts(); mqtt_events(); mqtt_utk_start(); } }
static int fails;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

int main(int argc, char **argv) {
  if (argc > 1) fail_ms = (u32_t)atoi(argv[1]);
  printf("(an unreachable broker fails each attempt after %u ms)\n", fail_ms);
  run_until(1000);
  printf("boot: connects=%d id=%s\n", connects, last_id);
  CHECK(connects == 1 && cl.st == CONN, "boot connect");
  run_until(10000); drop(); int c0 = connects; run_until(10020);
  printf("drop of a working link: reconnect attempts within 20 ms = %d\n", connects - c0);
  CHECK(connects - c0 == 1, "immediate reconnect after a working link drops");
  run_until(20000); broker_up = 0; drop(); c0 = connects; int l0 = lines, i0 = isconn;
  run_until(40000);
  printf("broker unreachable for 20 s: connect attempts=%d, 'already connecting'=%d, console lines=%d, max timers=%d, lost timers=%d\n",
         connects - c0, isconn - i0, lines - l0, max_live, lost_timers);
  CHECK(isconn - i0 == 0, "no 'already connecting' spam");
  CHECK(lines - l0 < 30, "console lines bounded");
  CHECK(lost_timers == 0, "no lwIP timer lost");
  broker_up = 1; u32_t t_back = now; while (cl.st != CONN && now < t_back + 10000) run_until(now + 2);
  printf("broker back: reconnected after %u ms\n", now - t_back);
  CHECK(cl.st == CONN && now - t_back <= 4100, "reconnect within ~4 s");
  run_until(now + 500); drop(); c0 = connects; run_until(now + 100);
  printf("drop 0.5 s after connecting: attempts within 100 ms = %d (paced)\n", connects - c0);
  CHECK(connects - c0 == 0, "a flapping link is paced");
  printf(fails ? "%d FAILED\n" : "ALL OK\n", fails);
  return fails != 0;
}
