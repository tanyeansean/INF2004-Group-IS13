/*
 *	drv_hcsr04.h
 *	Subsystem 5 -- HC-SR04P ultrasonic driver. TRIG GP4, ECHO GP5.
 *	Call only from obstacle_task, the owning task (TEAM.md section 3).
 *
 *	Lab 3: the plain HC-SR04 is a 5 V part; only the HC-SR04P/HC-SR04+
 *	may be wired straight to the Pico. Check the module's marking.
 */

#ifndef DRV_HCSR04_H
#define DRV_HCSR04_H

#include "../app.h"

typedef enum
{
	HCSR04_OK = 0,		/* *p_dist_cm valid (255 = beyond 254 cm)  */
	HCSR04_NO_ECHO,		/* ECHO never rose: wiring/power/sensor    */
	HCSR04_BUSY		/* ECHO still high from the previous ping  */
} hcsr04_status_t;

/* Create the driver's event flag. Call from subsystem5_init(). */
IMPORT ER		drv_hcsr04_create(void);

/*
 * Configure pins and, if S5_OWN_GPIO_IRQ, the GPIO interrupt.
 * Call from obstacle_task: the NVIC and the IO_BANK0 PROCx_INTE
 * registers are per core, so the interrupt lands on the core that
 * calls this. obstacle_task is pinned to core 0 (TP_PRC1).
 */
IMPORT ER		drv_hcsr04_init(void);

/*
 * Trigger one ping, then SLEEP on an event flag (<= OA_ECHO_TIMEOUT_MS)
 * for the echo. Other tasks run meanwhile (Lab 3 Q3), and a missing echo
 * times out instead of hanging (Lab 3 Q2).
 */
IMPORT hcsr04_status_t	drv_hcsr04_measure_cm(UB *p_dist_cm);

#endif /* DRV_HCSR04_H */
