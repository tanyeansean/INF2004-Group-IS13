/**
 * @file rtos_port_utk.c
 * @brief micro T-Kernel (uT-Kernel 3.0) adapter for rtos_port.h.
 *
 * TARGET-ONLY. Not compiled in the host unit-test build.
 *
 * STATUS: STUB. The SDK + uT-Kernel integration is unverified (project
 * risk A7). Fill in the TODOs once the kernel is available. Each wrapper
 * maps directly to a uT-Kernel service; signatures are chosen so the
 * Buddy 3 logic never changes when this is completed.
 *
 * Mapping reference:
 *   rtos_queue_send  -> tk_snd_mbf(id, msg, len, TMO_POL)   (non-blocking)
 *   rtos_queue_recv  -> tk_rcv_mbf(id, msg, &len, tmo)
 *   rtos_flag_set    -> tk_set_flg(id, bits)                (atomic)
 *   rtos_delay_ms    -> tk_dly_tsk(ms)
 *   rtos_now_ms      -> tk_get_otm() / SYSTIM conversion
 *
 * uT-Kernel timeouts: TMO_POL == 0 (poll, no wait), TMO_FEVR == -1.
 * These already match RTOS_NO_WAIT / RTOS_WAIT_FOREVER.
 */
#include "rtos_port.h"

/* #include <tk/tkernel.h>   // enable once uT-Kernel headers are on the path */

rtos_status_t rtos_queue_send(rtos_queue_t q, const void *msg, size_t len)
{
    (void)q; (void)msg; (void)len;
    /* TODO:
     * ER er = tk_snd_mbf((ID)q, (void*)msg, (INT)len, TMO_POL);
     * if (er == E_OK)      return RTOS_OK;
     * if (er == E_TMOUT)   return RTOS_FULL;   // full on zero-timeout
     * return RTOS_ERR;
     */
    return RTOS_OK;
}

rtos_status_t rtos_queue_recv(rtos_queue_t q, void *msg, size_t len, int32_t timeout_ms)
{
    (void)q; (void)msg; (void)len; (void)timeout_ms;
    /* TODO:
     * INT sz;
     * ER er = tk_rcv_mbf((ID)q, msg, &sz, (TMO)timeout_ms);
     * if (er == E_OK)    return RTOS_OK;
     * if (er == E_TMOUT) return RTOS_TIMEOUT;
     * return RTOS_ERR;
     */
    return RTOS_TIMEOUT;
}

rtos_status_t rtos_flag_set(rtos_flag_t flag, uint32_t bits)
{
    (void)flag; (void)bits;
    /* TODO:
     * ER er = tk_set_flg((ID)flag, (UINT)bits);
     * return (er == E_OK) ? RTOS_OK : RTOS_ERR;
     */
    return RTOS_OK;
}

void rtos_delay_ms(uint32_t ms)
{
    (void)ms;
    /* TODO: tk_dly_tsk((RELTIM)ms); */
}

uint32_t rtos_now_ms(void)
{
    /* TODO: SYSTIM t; tk_get_otm(&t); return (uint32_t)(t.lo); */
    return 0u;
}
