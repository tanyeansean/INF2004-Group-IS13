/**
 * @file    hb_bits.h
 * @brief   Buddy 1 - Safety Monitor heartbeat bits (hb_evt event flag).
 *
 * Each task sets ONLY its own bit, every time it wakes (by event or by its
 * timeout), with tk_set_flg() on the heartbeat event flag. The Safety Monitor
 * reads and clears all the bits in one kernel call when it takes attendance
 * every 20 ms. Fault bits live in their own event flag, never here (G2).
 *
 * Periods and hung limits are from the G9 timing table.
 */
#ifndef HB_BITS_H
#define HB_BITS_H

#define HB_MOTOR_CONTROL    (1u << 0)   /* Buddy 2, 20 ms loop     - hung after 40 ms  */
#define HB_MISSION_CONTROL  (1u << 1)   /* Buddy 2, 20 ms timeout  - hung after 40 ms  */
#define HB_LINE_FOLLOW      (1u << 2)   /* Buddy 3, 5 ms loop      - hung after 40 ms  */
#define HB_BARCODE_DECODE   (1u << 3)   /* Buddy 3, 20 ms timeout  - hung after 40 ms  */
#define HB_TERRAIN_MONITOR  (1u << 4)   /* Buddy 4, 20 ms loop     - hung after 40 ms  */
#define HB_OBSTACLE_AVOID   (1u << 5)   /* Buddy 5, 60 ms waits    - hung after 120 ms */
#define HB_TELEMETRY        (1u << 6)   /* Buddy 1, ~10 ms         - report only       */

#define HB_ALL_TASKS        (0x7Fu)

/* A miss on any of these is a hung task: E_STOP. Telemetry is report-only,
 * because losing Wi-Fi must not stop the car. */
#define HB_STOP_ON_MISS     (HB_ALL_TASKS & ~HB_TELEMETRY)

#endif /* HB_BITS_H */
