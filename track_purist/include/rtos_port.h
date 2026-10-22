/**
 * @file rtos_port.h
 * @brief Thin RTOS abstraction for Buddy 3 modules.
 *
 * The line-following and barcode-decode LOGIC is RTOS-agnostic and
 * host-testable. Only this layer touches the RTOS. The intended target
 * is micro T-Kernel (uT-Kernel 3.0) on the Raspberry Pi Pico, but
 * because the SDK+uT-Kernel integration is still unverified (project
 * risk A7), isolating these calls lets us swap the adapter (uT-Kernel,
 * FreeRTOS, or bare-metal) without touching the logic.
 *
 * Adapters:
 *   rtos_port_utk.c  - micro T-Kernel implementation (tk_snd_mbx / tk_set_flg / tk_dly_tsk)
 *   (host tests use a stub / direct calls and do not link this layer)
 */
#ifndef RTOS_PORT_H
#define RTOS_PORT_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Opaque handles so logic files never see RTOS-specific types. */
typedef int rtos_queue_t;   /* maps to a uT-Kernel mailbox/message-buffer id */
typedef int rtos_flag_t;    /* maps to a uT-Kernel event flag id */

typedef enum {
    RTOS_OK        = 0,
    RTOS_TIMEOUT   = 1,
    RTOS_FULL      = 2,   /* non-blocking send failed: queue full */
    RTOS_EMPTY     = 3,   /* non-blocking receive: queue empty */
    RTOS_ERR       = -1
} rtos_status_t;

/* Wait forever / do not wait, mirroring uT-Kernel TMO_FEVR / TMO_POL. */
#define RTOS_WAIT_FOREVER  (-1)
#define RTOS_NO_WAIT        (0)

/**
 * Non-blocking (zero-timeout) send. Returns RTOS_FULL if the queue is
 * full — the caller counts the drop (Buddy 3 queues never block).
 */
rtos_status_t rtos_queue_send(rtos_queue_t q, const void *msg, size_t len);

/**
 * Receive with a timeout in milliseconds (use RTOS_WAIT_FOREVER /
 * RTOS_NO_WAIT). Returns RTOS_TIMEOUT when nothing arrives in time.
 * The Barcode Decode task uses a 20 ms timeout so it keeps setting its
 * heartbeat bit even when idle.
 */
rtos_status_t rtos_queue_recv(rtos_queue_t q, void *msg, size_t len, int32_t timeout_ms);

/**
 * Set bits in an event flag (atomic in the kernel). Used for the shared
 * fault mask and the heartbeat flag. Each task sets only its own bits.
 */
rtos_status_t rtos_flag_set(rtos_flag_t flag, uint32_t bits);

/** Delay the calling task for the given milliseconds (periodic pacing). */
void rtos_delay_ms(uint32_t ms);

/** Monotonic millisecond tick (for junction timing, frame timeouts). */
uint32_t rtos_now_ms(void);

#ifdef __cplusplus
}
#endif

#endif /* RTOS_PORT_H */
