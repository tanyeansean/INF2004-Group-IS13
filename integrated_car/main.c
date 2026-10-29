/**
 * @file main.c
 * @brief Integrated bare-metal car firmware (NO uT-Kernel yet):
 *          Buddy 3 track_purist  - IR line following, barcode, nav_cmd
 *          Buddy 1 robocar_comms - WiFi / MQTT / telemetry / barcode map
 *          motor.c               - open-loop PWM drive (from motor_test.c)
 *
 * Buddy 3's modules are compiled straight from the team repo (not copied),
 * so this always uses the current track_purist code.
 *
 * One while(1) loop replaces the RTOS tasks:
 *   comms_poll() + telem_poll()   every pass (WiFi/MQTT must run every ~10 ms)
 *   line_follow_step()            every 5 ms -> steering + junction trigger
 *   barcode edge queue drain      every pass -> letter -> nav_on_barcode()
 *
 * Control:
 *   MQTT (tools/telemetry_monitor.py): start, stop, estop, speed N, map ...
 *   USB serial (115200): S = start, X = stop, ABCD = map (LEFT RIGHT FWD UTURN),
 *                        #Z = fake barcode 'Z'
 * The car does NOT move until it gets "start". Wheels off the ground for
 * the first run.
 *
 * Wiring: see track_purist/HANDOFF.md (IR) and motor.h (motors).
 */
#include <stdio.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"

#include "line_follow.h"
#include "ir_calibrate.h"
#include "barcode_decode.h"
#include "nav_cmd.h"

#include "comms.h"
#include "comms_config.h"   /* CMD_SPEED_MAX_CMS (20 cm/s cap) */
#include "telemetry.h"

#include "motor.h"

/* ---- Pins (Robo Pico Grove ports, as in the self-test) ------------------ */
#define PIN_LEFT_ADC    27u  /* Grove 6 yellow, left front IR,  ADC1 */
#define PIN_RIGHT_ADC   26u  /* Grove 5 yellow, right front IR, ADC0 */
#define ADC_CH_LEFT      1u
#define ADC_CH_RIGHT     0u
#define PIN_BARCODE      7u  /* Grove 7 white, barcode D0 */

/* ---- Tuning (open loop: no encoders yet - tune on the track) ------------ */
/* 0 = "start" drives BOTH wheels straight at the base PWM (no steering).
   1 = steer from the front IR sensors (line following). */
#ifndef STEERING_ENABLED
#define STEERING_ENABLED     1
#endif
/* Simple straddle rules (front sensors either side of the line):
     both white -> centred, drive straight
     left black -> line drifted under LEFT sensor  -> turn slightly left
     right black-> line drifted under RIGHT sensor -> turn slightly right
     both black (junction flag) -> stop (when JUNCTION_STOPS = 1)       */
/* Per-wheel base PWM (0..255). Set different values if one wheel runs
   faster than the other. "speed N" keeps this M1:M2 ratio. */
#define BASE_PWM_M1         30
#define BASE_PWM_M2         30
#define STEER_DELTA         15   /* one wheel +15, other -15 on a gentle turn */
/* Which way to steer when ONE sensor sees black. 1 = speed up the wheel on
   the black sensor's side (turn away from it); 0 = slow that wheel down.
   Set from the car's real behaviour on the track. */
#ifndef STEER_INVERT
#define STEER_INVERT         0   /* verified on car: right sensor black -> turn right */
#endif
#ifndef JUNCTION_STOPS
#define JUNCTION_STOPS       1   /* 1 = stop at a junction, no barcode turn */
#endif
/* PWM for "speed 20" (CMD_SPEED_MAX_CMS, the 20 cm/s cap set by comms).
   85 keeps the old scale (speed 60 -> 255) until it is measured on the track:
   set it to the PWM that really gives about 20 cm/s. */
#ifndef PWM_AT_SPEED_MAX
#define PWM_AT_SPEED_MAX     85
#endif
#define TURN_PWM           130   /* wheel PWM while spinning at a junction */
#define TURN_90_MS         450u  /* spin time for LEFT / RIGHT             */
#define TURN_180_MS        900u  /* spin time for U-TURN                   */
#define CROSS_MS           300u  /* drive straight to clear a junction     */

/* 1 = after every barcode result (VALID or INVALID) print one debug line:
   window edge count, edges in the burst, glitch pulses cancelled, longest
   element and all element widths in ms. Printed from the main loop, outside
   the 5 ms line-follow step. Keep 0 in the final car build. */
#ifndef BARCODE_DEBUG
#define BARCODE_DEBUG        0
#endif

#define CAL_COUNTDOWN_S      3
#define CAL_SAMPLES        150   /* x 10 ms = 1.5 s per phase */
#define CAL_MIN_CONTRAST   200u

/* Mission Control (Buddy 2) owns this at full integration. Telemetry writes
   it via nav_map_apply() when a map arrives on cmd/barcode_map. */
nav_map_t g_nav_map;

/* ---- Barcode edge ISR (the only GPIO callback in this build) ------------ */
static barcode_edge_queue_t g_eq;

static void barcode_gpio_isr(uint gpio, uint32_t events)
{
    (void)events;
    if (gpio == PIN_BARCODE) {
        (void)barcode_eq_push(&g_eq, time_us_32());   /* timestamp only */
    }
}

/* ---- Car state ----------------------------------------------------------- */
typedef enum { CAR_IDLE = 0, CAR_FOLLOW, CAR_MANEUVER } car_mode_t;

static car_mode_t g_mode = CAR_IDLE;
static int16_t    g_base_m1 = BASE_PWM_M1;   /* per-channel base PWM */
static int16_t    g_base_m2 = BASE_PWM_M2;
#if MOTOR_LEFT_IS_M1
#define BASE_LEFT   g_base_m1
#define BASE_RIGHT  g_base_m2
#else
#define BASE_LEFT   g_base_m2
#define BASE_RIGHT  g_base_m1
#endif
static bool       g_comms_ok = false;
static int16_t    g_out_l, g_out_r;  /* last PWM sent while following (status line) */
static uint8_t    g_lb, g_rb;        /* last left/right "on black" flags (status line) */

static uint32_t   g_man_end_ms;      /* end of the current maneuver step   */
static int16_t    g_man_left, g_man_right;
static uint8_t    g_man_phase;       /* 0 = crossing, 1 = turning          */
static uint8_t    g_man_move;

static uint32_t now_ms(void) { return to_ms_since_boot(get_absolute_time()); }

static uint16_t read_left(void)  { adc_select_input(ADC_CH_LEFT);  return (uint16_t)adc_read(); }
static uint16_t read_right(void) { adc_select_input(ADC_CH_RIGHT); return (uint16_t)adc_read(); }

/* Keep WiFi/MQTT alive while waiting (calibration countdowns etc.). */
static void wait_ms(uint32_t ms)
{
    uint32_t end = now_ms() + ms;
    while ((int32_t)(now_ms() - end) < 0) {
        if (g_comms_ok) { comms_poll(); telem_poll(); }
        sleep_ms(1);
    }
}

static void car_stop(robot_state_t why)
{
    motor_stop();
    g_mode = CAR_IDLE;
    if (g_comms_ok) { telem_set_state(why); }
}

static bool g_calibrated = false;   /* start is ignored until calibration is done */

static void car_start(line_follow_state_t *ls)
{
    if (!g_calibrated) {
        printf("[car] start ignored - still calibrating\n");
        return;
    }
    line_follow_reset(ls);
    g_mode = CAR_FOLLOW;
    if (g_comms_ok) { telem_set_state(ROBOT_LINE_FOLLOW); }
    printf("[car] START (base PWM M1=%d M2=%d)\n", g_base_m1, g_base_m2);
}

/* Junction move: cross the junction a little, then spin (timed, open loop). */
static void maneuver_begin(uint8_t move)
{
    g_mode      = CAR_MANEUVER;
    g_man_move  = move;
    g_man_phase = 0u;
    g_man_left  = BASE_LEFT;
    g_man_right = BASE_RIGHT;
    g_man_end_ms = now_ms() + CROSS_MS;
    motor_set(g_man_left, g_man_right);
    if (g_comms_ok) { telem_set_state(ROBOT_TURNING); }
}

static void maneuver_step(line_follow_state_t *ls)
{
    if ((int32_t)(now_ms() - g_man_end_ms) < 0) {
        return;
    }
    if (g_man_phase == 0u && g_man_move != BARCMD_STRAIGHT) {
        g_man_phase = 1u;
        switch (g_man_move) {
            case BARCMD_LEFT:
                g_man_left = (int16_t)-TURN_PWM; g_man_right = TURN_PWM;
                g_man_end_ms = now_ms() + TURN_90_MS;
                break;
            case BARCMD_RIGHT:
                g_man_left = TURN_PWM; g_man_right = (int16_t)-TURN_PWM;
                g_man_end_ms = now_ms() + TURN_90_MS;
                break;
            default: /* BARCMD_U_TURN */
                g_man_left = TURN_PWM; g_man_right = (int16_t)-TURN_PWM;
                g_man_end_ms = now_ms() + TURN_180_MS;
                break;
        }
        motor_set(g_man_left, g_man_right);
        return;
    }
    /* done: back to line following */
    line_follow_reset(ls);
    g_mode = CAR_FOLLOW;
    if (g_comms_ok) { telem_set_state(ROBOT_LINE_FOLLOW); }
}

/* ---- MQTT commands (comms_poll() calls this, main-loop context) ---------- */
static line_follow_state_t *g_ls_for_cmd;

/* Returns true if the command was taken, false if not (the MQTT ack then
   says "rejected" instead of "accepted"). */
static bool on_command(comms_cmd_t const *p_cmd)
{
    bool b_ok = true;
    switch (p_cmd->type) {
        case CMD_START:
            if (!g_calibrated) { b_ok = false; }   /* start is ignored until READY */
            if (g_mode == CAR_IDLE) { car_start(g_ls_for_cmd); }
            break;
        case CMD_STOP:
            car_stop(ROBOT_STOPPED);
            printf("[car] STOP (MQTT)\n");
            break;
        case CMD_ESTOP:
            car_stop(ROBOT_STOPPED);
            printf("[car] ESTOP (MQTT)\n");
            break;
        case CMD_SET_SPEED:
            /* No encoders: "speed N" (cm/s, capped at CMD_SPEED_MAX_CMS by
               comms) is scaled to PWM, CMD_SPEED_MAX_CMS -> PWM_AT_SPEED_MAX. */
            if (p_cmd->arg > 0 && p_cmd->arg <= CMD_SPEED_MAX_CMS) {
                g_base_m1 = (int16_t)((p_cmd->arg * PWM_AT_SPEED_MAX) / CMD_SPEED_MAX_CMS);
                g_base_m2 = (int16_t)((g_base_m1 * BASE_PWM_M2) / BASE_PWM_M1);
                if (g_base_m2 > MOTOR_MAX_SPEED) { g_base_m2 = MOTOR_MAX_SPEED; }
                printf("[car] base PWM M1=%d M2=%d\n", g_base_m1, g_base_m2);
            }
            break;
        case CMD_RESET_PEAK:
            telem_reset_peak();
            break;
        default:
            break;
    }
    return b_ok;
}

static telem_move_t to_telem_move(uint8_t barcmd)
{
    return (barcmd < BARCMD_COUNT) ? (telem_move_t)barcmd : TELEM_MOVE_UNMAPPED;
}

/* One barcode result (VALID or INVALID) from the assembler. Main-loop
   context, never inside the 5 ms line-follow step. */
static void on_barcode_result(const barcode_asm_t *p_asm, barcode_t res,
                              nav_state_t *nav, uint8_t *p_seen_dropped)
{
    barcode_frame_info_t info;

    barcode_asm_last_frame(p_asm, &info);
    if (barcode_eq_overflow_delta(&g_eq, p_seen_dropped) != 0u) {
        res = barcode_invalid();          /* edges were dropped: untrusted */
    }

    if (barcode_is_valid(res)) {
        uint8_t mv = 0;
        uint8_t mapped = nav_map_lookup(nav_map_snapshot(&g_nav_map),
                                        barcode_get_letter(res), &mv);
        printf("[bar] VALID '%c'%s\n", barcode_get_char(res),
               mapped ? "" : " (not in map)");
        nav_on_barcode(nav, res);
        if (g_comms_ok) {
            telem_event_barcode(barcode_get_char(res),
                                mapped ? to_telem_move(mv) : TELEM_MOVE_UNMAPPED);
        }
    } else {
        if (info.outcome == BARCODE_OUT_PARTIAL) {
            printf("[bar] INVALID (partial: %u of 30 edges)\n", (unsigned)info.n_edges);
        } else {
            printf("[bar] INVALID (%u edges, no window decoded)\n", (unsigned)info.burst_edges);
        }
        if (g_comms_ok) { telem_event_barcode('?', TELEM_MOVE_UNMAPPED); }
    }

#if BARCODE_DEBUG
    printf("[bar dbg] %s edges=%u burst=%u chatter=%u longest=%lu.%lu ms widths ms:",
           barcode_is_valid(res) ? "VALID" : "INVALID",
           (unsigned)info.n_edges, (unsigned)info.burst_edges, (unsigned)info.chatter_pairs,
           (unsigned long)(info.longest_us / 1000u), (unsigned long)((info.longest_us % 1000u) / 100u));
    for (uint8_t i = 0u; i < info.n_widths; i++) {
        printf(" %lu.%lu", (unsigned long)(info.widths_us[i] / 1000u),
               (unsigned long)((info.widths_us[i] % 1000u) / 100u));
    }
    printf("\n");
#endif
}

/* ---- Serial commands: S, X, ABCD, #Z -------------------------------------- */
static void handle_serial_cmd(const char *buf, uint8_t len,
                              line_follow_state_t *ls, nav_state_t *nav)
{
    printf("[rx] \"%.*s\" (%u chars)\n", len, buf, len);
    if (len == 1u && (buf[0] | 0x20) == 's') {
        if (g_mode == CAR_IDLE) { car_start(ls); }
    } else if (len == 1u && (buf[0] | 0x20) == 'x') {
        car_stop(ROBOT_STOPPED);
        printf("[car] STOP (serial)\n");
    } else if (len == 1u && (buf[0] == '1' || buf[0] == '2')) {
        uint8_t chn = (uint8_t)(buf[0] - '0');
        if (g_mode != CAR_IDLE) {
            printf("[motor] stop the car first (X)\n");
            return;
        }
        printf("[motor] M%u forward 200 for 1 s...\n", chn);
        motor_set_channel(chn, 200);
        wait_ms(1000);
        printf("[motor] M%u backward 200 for 1 s...\n", chn);
        motor_set_channel(chn, -200);
        wait_ms(1000);
        motor_stop();
        printf("[motor] M%u done\n", chn);
    } else if (len == 2u && buf[0] == '#') {
        char c = (char)(buf[1] & ~0x20);
        if (c >= 'A' && c <= 'Z') {
            nav_on_barcode(nav, barcode_pack_letter((uint8_t)(c - 'A')));
            printf("[bar] fake barcode '%c' pending\n", c);
        }
    } else if (len == BARCMD_COUNT) {
        uint8_t letters[BARCMD_COUNT], payload[NAV_MAP_PAYLOAD_LEN];
        uint8_t st = NAV_MAP_OK;
        for (uint8_t i = 0; i < BARCMD_COUNT; i++) {
            char c = (char)(buf[i] & ~0x20);
            if (c < 'A' || c > 'Z') { st = NAV_MAP_ERR_RANGE; break; }
            letters[i] = (uint8_t)(c - 'A');
        }
        if (st == NAV_MAP_OK) { st = nav_map_encode(letters, payload); }
        if (st == NAV_MAP_OK) { st = nav_map_apply(&g_nav_map, payload, NAV_MAP_PAYLOAD_LEN); }
        printf("[map] serial map %s (code %u)\n", st == NAV_MAP_OK ? "applied" : "rejected", st);
    } else {
        printf("? commands: S start, X stop, 1/2 spin motor M1/M2, ABCD map, #Z fake barcode\n");
    }
}

/* ---- Calibration (same procedure as the self-test) ----------------------- */
static void countdown(const char *msg)
{
    printf("%s\n", msg);
    for (int s = CAL_COUNTDOWN_S; s > 0; s--) {
        printf("  starting in %d...\n", s);
        wait_ms(1000);
    }
    printf("  SAMPLING - hold still (1.5 s)\n");
}

static uint32_t udiff(uint32_t a, uint32_t b) { return a > b ? a - b : b - a; }

static void calibrate(line_follow_cfg_t *cfg)
{
    ir_cal_accum_t cl, cr;
    ir_cal_reset(&cl);
    ir_cal_reset(&cr);

    countdown("CALIBRATE 1/2: put BOTH front sensors over the BLACK LINE");
    for (int i = 0; i < CAL_SAMPLES; i++) {
        ir_cal_add_line(&cl, read_left());
        ir_cal_add_line(&cr, read_right());
        wait_ms(10);
    }
    countdown("CALIBRATE 2/2: put BOTH front sensors over WHITE");
    for (int i = 0; i < CAL_SAMPLES; i++) {
        ir_cal_add_background(&cl, read_left());
        ir_cal_add_background(&cr, read_right());
        wait_ms(10);
    }

    cfg->junction_hold_ms = 30;
    cfg->error_gain       = 1;
    cfg->lost_timeout_ms  = 0;
    if (!ir_cal_apply(&cl, &cr, cfg)) {
        printf("CALIBRATION FAILED - halting (motors off)\n");
        motor_stop();
        while (1) { wait_ms(100); }
    }

    uint32_t lb = cl.line_sum / cl.line_count, lw = cl.bg_sum / cl.bg_count;
    uint32_t rb = cr.line_sum / cr.line_count, rw = cr.bg_sum / cr.bg_count;
    printf("L black=%lu white=%lu | R black=%lu white=%lu\n",
           (unsigned long)lb, (unsigned long)lw, (unsigned long)rb, (unsigned long)rw);
    if (udiff(lb, lw) < CAL_MIN_CONTRAST || udiff(rb, rw) < CAL_MIN_CONTRAST) {
        printf("WARNING: low contrast on a sensor - check height / wiring\n");
    }
}

int main(void)
{
    stdio_init_all();
    motor_init();                      /* motors stopped first thing */

    absolute_time_t deadline = make_timeout_time_ms(10000);
    while (!stdio_usb_connected() && !time_reached(deadline)) { sleep_ms(100); }
    sleep_ms(300);
    printf("\n=== integrated_car v0.8, comms v0.3.1 (base M1=%d M2=%d, start = %s, steer_invert = %d, junction = %s) ===\n",
           BASE_PWM_M1, BASE_PWM_M2,
           STEERING_ENABLED ? "line steering" : "straight, both wheels",
           STEER_INVERT,
           JUNCTION_STOPS ? "stop" : "barcode turn");
    printf("Serial commands work only AFTER calibration (when READY is printed).\n");

    adc_init();
    adc_gpio_init(PIN_LEFT_ADC);
    adc_gpio_init(PIN_RIGHT_ADC);

    barcode_eq_init(&g_eq);
    gpio_init(PIN_BARCODE);
    gpio_set_dir(PIN_BARCODE, GPIO_IN);
    gpio_pull_up(PIN_BARCODE);
    gpio_set_irq_enabled_with_callback(PIN_BARCODE,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &barcode_gpio_isr);

    nav_map_init(&g_nav_map);

    line_follow_state_t ls;
    line_follow_reset(&ls);
    g_ls_for_cmd = &ls;

    g_comms_ok = comms_init();
    if (g_comms_ok) {
        telem_init();
        comms_set_cmd_handler(on_command);
        telem_set_state(ROBOT_IDLE);
    } else {
        printf("WiFi chip init failed - running WITHOUT MQTT (serial control only)\n");
    }

    line_follow_cfg_t cfg;
    calibrate(&cfg);
    g_calibrated = true;

    nav_state_t nav;
    nav_reset(&nav);

    barcode_asm_t basm;
    barcode_asm_init(&basm);           /* size/speed independent, see barcode_decode.h */
    uint8_t seen_dropped = 0;
    (void)barcode_eq_overflow_delta(&g_eq, &seen_dropped);

    printf("READY. Send 'start' over MQTT or type S. X = stop. 1/2 = spin M1/M2.\n");

    char     rx[8];
    uint8_t  rx_len = 0;
    uint32_t rx_last = 0;
    uint32_t next_line_ms = now_ms();
    uint32_t next_print_ms = now_ms();
    line_err_t le = line_err_pack(0, 0, 1);

    while (1) {
        uint32_t t = now_ms();

        if (g_comms_ok) { comms_poll(); telem_poll(); }

        /* ---- serial input (ends on Enter or 300 ms idle) ---- */
        int ch = getchar_timeout_us(0);
        while (ch != PICO_ERROR_TIMEOUT) {
            if (ch == '\r' || ch == '\n') {
                if (rx_len) { handle_serial_cmd(rx, rx_len, &ls, &nav); rx_len = 0; }
            } else if (rx_len < sizeof(rx)) {
                rx[rx_len++] = (char)ch;
                rx_last = t;
            }
            ch = getchar_timeout_us(0);
        }
        if (rx_len && (t - rx_last) >= 300u) {
            handle_serial_cmd(rx, rx_len, &ls, &nav);
            rx_len = 0;
        }

        /* ---- line following, every 5 ms ---- */
        if ((int32_t)(t - next_line_ms) >= 0) {
            next_line_ms += LINE_FOLLOW_PERIOD_MS;

            line_sample_t s;
            s.left_adc   = read_left();
            s.center_adc = read_right();
            s.now_ms     = t;
            le = line_follow_step(&cfg, &ls, &s);

            int16_t err   = line_err_get_error(le);
            uint8_t l_blk = line_follow_on_black(&cfg, s.left_adc, 0u);
            uint8_t r_blk = line_follow_on_black(&cfg, s.center_adc, 1u);
            g_lb = l_blk;
            g_rb = r_blk;

            if (g_comms_ok) {
                int16_t pos = (int16_t)(err / 8);          /* -64..63: 7-bit telemetry field */
                telem_set_line((uint8_t)((l_blk ? 0x1u : 0u) | (r_blk ? 0x2u : 0u)),
                               pos, !line_err_get_lost(le));
            }

            /* junction trigger: feed EVERY line message */
            uint8_t mv = 0, let = 0;
            uint8_t ev = nav_on_line(&nav, le, nav_map_snapshot(&g_nav_map), &mv, &let);
            if (ev == NAV_EVT_MOVE) {
                printf("[nav] junction: '%c' -> move %u\n", 'A' + let, mv);
                if (g_comms_ok) { telem_event_junction((char)('A' + let), to_telem_move(mv)); }
                if (g_mode == CAR_FOLLOW && !JUNCTION_STOPS) { maneuver_begin(mv); }
            } else if (ev == NAV_EVT_UNMAPPED) {
                printf("[nav] junction: '%c' has NO mapping - driving on\n", 'A' + let);
                if (g_comms_ok) { telem_event_junction((char)('A' + let), TELEM_MOVE_UNMAPPED); }
            }

            if (g_mode == CAR_FOLLOW) {
#if STEERING_ENABLED
                (void)err;
                if (line_err_get_lost(le)) {
                    car_stop(ROBOT_LINE_RECOVERY);
                    printf("[car] line data unusable - stopped\n");
                } else if (line_err_get_junction(le) && JUNCTION_STOPS) {
                    car_stop(ROBOT_STOPPED);
                    printf("[car] JUNCTION (both black) - stopped. Send start to go again.\n");
                } else {
                    /* +1: line under LEFT sensor, -1: under RIGHT, 0: centred */
                    int16_t side = (l_blk && !r_blk) ? 1 : ((r_blk && !l_blk) ? -1 : 0);
#if STEER_INVERT
                    side = (int16_t)-side;
#endif
                    /* side = +1 -> left wheel slower, right faster (and vice versa) */
                    g_out_l = (int16_t)(BASE_LEFT  - side * STEER_DELTA);
                    g_out_r = (int16_t)(BASE_RIGHT + side * STEER_DELTA);
                    motor_set(g_out_l, g_out_r);
                }
#else
                /* straight drive: both wheels the same, sensors ignored */
                (void)err;
                g_out_l = BASE_LEFT;
                g_out_r = BASE_RIGHT;
                motor_set(g_out_l, g_out_r);
#endif
            } else if (g_mode == CAR_MANEUVER) {
                maneuver_step(&ls);
            }
        }

        /* ---- barcode: drain edges, assemble, decode ----
           A result can come from either call: an edge only counts after
           its 2 ms chatter check (barcode_decode.h). */
        barcode_edge_t e;
        barcode_t res;
        while (barcode_eq_pop(&g_eq, &e)) {
            if (barcode_asm_edge_arrived(&basm, e.t_us, &res)) {
                on_barcode_result(&basm, res, &nav, &seen_dropped);
            }
        }
        if (barcode_asm_time_passed(&basm, time_us_32(), &res)) {
            on_barcode_result(&basm, res, &nav, &seen_dropped);
        }

        /* ---- status line, 2 Hz ---- */
        if ((int32_t)(t - next_print_ms) >= 0) {
            uint8_t pend = 0;
            next_print_ms += 500u;
            printf("mode=%u L=%c R=%c err=%4d jn=%u lost=%u pending=%c pwm=%d/%d link=%s\n",
                   (unsigned)g_mode, g_lb ? 'B' : 'W', g_rb ? 'B' : 'W',
                   line_err_get_error(le), line_err_get_junction(le),
                   line_err_get_lost(le),
                   nav_has_pending(&nav, &pend) ? (char)('A' + pend) : '-',
                   g_mode == CAR_FOLLOW ? g_out_l : 0, g_mode == CAR_FOLLOW ? g_out_r : 0,
                   g_comms_ok ? comms_link_name(comms_get_link()) : "off");
        }

        sleep_ms(1);
    }
}
