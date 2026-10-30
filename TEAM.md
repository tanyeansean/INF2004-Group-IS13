# Team and subsystems

The template's example rows were replaced by the robot car on 29 Oct 2026:
subsystems 1-3 run the car, ported from the bare-metal `integrated_car/` bench.
Subsystem 4 is an empty template. Subsystem 5 is compiled but not started
(`APP_WITH_S5` in `app_program/app.h`) until it is tested.

Fill this in at the start of the project and keep it up to date. It is the
single place that says **who owns what**, so that five people can work in
one firmware image without clashing over pins, priorities, or kernel objects.

A **subsystem** is one student's part of the product, with its own folder.
It may contain several tasks and own several peripherals (see section 3).

Rule: **only the owner edits a subsystem's folders.** To change another
subsystem, ask its owner or open a pull request for them to review.

## 1. Members and subsystems

| # | Subsystem (what it does) | Student ID | GitHub username | Code | Tests |
|---|---|---|---|---|---|
| 1 | Telemetry: packed MQTT live data, events, commands, barcode map (Buddy 1, Aurelius Ng Ker Fong) | 2500612 | | `app_program/subsystem1/` | `testing/subsystem1/` |
| 2 | Mission Control + Motor Control: steering, junction stop / turns, start / stop, PWM (Buddy 2, Tan Yean Sean) | 2500224 | tanyeansean | `app_program/subsystem2/` | `testing/subsystem2/` |
| 3 | Line Follow + Barcode Decode: front IR pair, Code 39 barcode (Buddy 3, Wong Yong Quan; logic in `track_purist/`) | 2500377 | yongquan1337 | `app_program/subsystem3/` | `testing/subsystem3/`, `track_purist/test/` |
| 4 | IMU / terrain (Buddy 4, Lim Jin Hin): empty template | 2500928 | | `app_program/subsystem4/` | `testing/subsystem4/` |
| 5 | Obstacle Avoid (Buddy 5, Joe Teo Wei Xuan): code in repo, not started yet | 2500177 | | `app_program/subsystem5/` | `testing/subsystem5/` |

Shared files (`app_program/app_main.c`, `app_program/app.h`, this file)
are changed only by agreement of the team. The exception is claiming an empty
row in this file (a pin, a peripheral, an interface ID), which anyone may do
(section 2).

## 2. Pins

One owner per pin. Claim a pin here **before** you wire or code it.

**Why:** two subsystems using the same pin is one of the most common and
hardest-to-find mistakes in a team project. The code builds without any
error, but the hardware misbehaves. Each subsystem can pass its own tests
and then fail once they are combined, and two outputs driving the same pin
against each other can damage it. Writing the claim down first means a clash
is spotted when someone edits this table, not hours later on the bench.

**No instructor check needed.** The team keeps this table itself. You do not
need the instructor, or the rest of the team, to check or approve a claim:
if the row is empty, fill it in and commit. If two people claim the same
pin, they agree between themselves who moves. Only the *reserved* rows are
fixed.

| Pin | Owner | Used for |
|---|---|---|
| GP23, GP24, GP25, GP29 | *reserved* | Pico W radio |
| GP0 | | (taken whenever the team builds with `CRASH_REPORT=1`) |
| GP1 | | (taken whenever the team builds with `CRASH_REPORT=1`) |
| GP2 | | |
| GP3 | | |
| GP4 | S5 | HC-SR04 TRIG (S5 not started yet) |
| GP5 | S5 | HC-SR04 ECHO, edge interrupt (S5 not started yet) |
| GP6 | | |
| GP7 | S3 | barcode IR digital out (D0), both-edge interrupt, pull-up |
| GP8 | S2 | motor M1A (left wheel), PWM slice 4A |
| GP9 | S2 | motor M1B (left wheel), PWM slice 4B |
| GP10 | S2 | motor M2A (right wheel), PWM slice 5A |
| GP11 | S2 | motor M2B (right wheel), PWM slice 5B |
| GP12 | S5 | servo PWM (S5 not started yet) |
| GP13 | | |
| GP14 | | |
| GP15 | | |
| GP16 | | |
| GP17 | | |
| GP18 | | |
| GP19 | | |
| GP20 | S2 | Robo Pico button: start / stop (active low, pull-up) |
| GP21 | | |
| GP22 | | |
| GP26 | S3 | right front IR, ADC0 |
| GP27 | S3 | left front IR, ADC1 |
| GP28 | | (ADC2 capable) |

## 3. Peripherals

**Subsystems, tasks and peripherals are three different things:**

| Term | What it is | Example |
|---|---|---|
| **Subsystem** | one student's part of the product, one folder | "environment sensing", "motor control", "user interface" |
| **Task** | one thread of execution. A subsystem may have **as many tasks as it needs** | a sampling task and a filtering task, both in "environment sensing" |
| **Peripheral** | one hardware block on the RP2040 | I2C0, UART1, the ADC |

A driver is **not** a subsystem. It is code *inside* the subsystem that
needs that hardware: the "environment sensing" subsystem contains the I2C
sensor code, and owns I2C0. Write it as `drv_<name>.c` / `drv_<name>.h` in
that subsystem's folder, never under `device/`, and call it only from the
peripheral's owning task. The kernel's own drivers in `device/` (serial,
I2C, ADC) may be used instead; switching one on in `config/config_device.h`
changes shared configuration, so record it in the table below first.

**The rule: each peripheral is driven by exactly one task, its *owning
task*.** That task can be in any subsystem, and one subsystem may own
several peripherals (one owning task each, or one task owning all of them).
Every other task, in the same subsystem or another one, asks the owning task
through a kernel object (section 5) and never touches that peripheral itself.

*Why:* this port runs tasks on both cores at the same time. If two tasks
drive the same I2C or UART block at once, their transfers get mixed up, and
the drivers have no locking to stop it.

| Peripheral | Owner subsystem | Owning task | Pins |
|---|---|---|---|
| UART1 | | | |
| I2C0 | | | |
| I2C1 | | | |
| SPI0 | | | |
| SPI1 | | | |
| ADC (kernel driver `adca`) | S3 | `line_task` | GP26, GP27 |
| PWM slices 4, 5 | S2 | `motor_task` | GP8–GP11 (10 kHz, wrap 255) |
| GPIO interrupt IO_IRQ_BANK0 (IRQ 13, only one `tk_def_int`) | S3 (handler in `drv_track.c`) | ISR, enabled by `barcode_task` on core 0 | GP7 now. S5's GP5 echo and S2's encoders must be added to this handler, not registered again |
| TIMER (1 µs counter, read only) | S3 starts it | any task may read | – |
| WiFi radio, TCP/IP, MQTT | *kernel* | `cyw43_utk_task` (built in) | GP23–25, GP29 |

### WiFi and MQTT are special

The radio and the network stack belong to the kernel's built-in WiFi service
task, `cyw43_utk_task` (`lib/libwifi/sysdepend/pico_rp2040/`). **No student
task may call `cyw43_...`, lwIP or MQTT functions, not even from a
"WiFi subsystem".** The calls are not safe from any other task, on either core.

What students *can* do:

- **Read the connection state** from any task, which is always safe:
  ```c
  #include "cyw43_utk.h"
  T_CYW43_UTK_STATUS st;
  cyw43_utk_get_status(&st);   /* st.ready, st.link_status, st.ip_addr, ... */
  ```
- **Send or receive data over the network** by adding code that *runs
  inside the service task*. The MQTT client is in
  `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`. It publishes a heartbeat
  today. A "cloud / reporting" subsystem would add a kernel object
  (e.g. a message buffer) that application tasks fill. The MQTT code then
  reads it with `TMO_POL` (never blocking) and publishes it.
  Because this changes shared library code, **agree it with the team and
  the instructor first**, and record it as an interface in section 5.
  It needs the `NET=lwip MQTT=1` build (README section 3).

  **Done for the car (IF-06):** `mqtt_utk.c` calls optional `app_mqtt_*()`
  hooks that subsystem 1 provides in `s1_mqtt_bridge.c`. They run inside
  the service task and only move bytes through two S1 message buffers
  (`mbf_mqtt_tx`, `mbf_mqtt_rx`), with zero-timeout calls on both sides.
  Without the hooks the client still runs its heartbeat demo.

## 4. Task priorities

1 is the highest. 1–4 are used by the kernel (USB console 3, WiFi 4). Each subsystem gets its own band by default;
change it only by team agreement. Write down every task you create.

| Subsystem | Priority band | Tasks (name – priority – core) |
|---|---|---|
| 3 | 12–13 | line_task – 12 – core 1; barcode_task – 13 – core 0 |
| 2 | 14–15 | motor_task – 14 – core 1; mission_task – 15 – core 1 |
| 1 | 16–17 | telemetry_task – 16 – core 0 |
| 4 | 18–19 | |
| 5 | 20–21 | obstacle_task – 20 – core 0 (not started) |

The bands of subsystems 1 and 3 are swapped (29 Oct): Telemetry must be
the lowest application priority (NFR5), and the 5 ms line sensing the highest.

Core: `TP_PRC1` = core 0, `TP_PRC2` = core 1, or "any".

## 5. Interfaces between subsystems

Every arrow between two subsystems is listed here. The **provider** creates
the kernel object and publishes its ID in its own `subsystemN.h`; the **user** only
sends to it or waits on it. Agree on the data format before coding.

| ID | From → To | Mechanism | Object ID name | Data / meaning | Agreed by |
|---|---|---|---|---|---|
| IF-01 | S3 → S2 | message buffer (owned by S2) | `mbf_line_err` | `line_err_t` (`track_types.h`): 15 bits in 2 bytes, every 5 ms | Buddies 2, 3 |
| IF-02 | S3 → S2 | message buffer (owned by S2) | `mbf_barcode` | `barcode_t`: 1 byte per VALID or INVALID read | Buddies 2, 3 |
| IF-03 | S2 → S3 | event flag (owned by S3) | `flg_s3` | `S3_FLG_LINE_RESET`: reset the junction / last-side state (on START) | Buddies 2, 3 |
| IF-04 | S1 → S2 | message buffer (owned by S2) | `mbf_cmd` | `comms_cmd_t`: 1 byte (type 3 bits + arg 5 bits) | Buddies 1, 2 |
| IF-05 | S2 → S1 | message buffer (owned by S1) | `mbf_status` | 16-bit packed state / line / barcode / junction (`subsystem1.h`) | Buddies 1, 2 |
| IF-06 | S1 ↔ kernel WiFi task | message buffers (owned by S1) + `app_mqtt_*()` hooks | `mbf_mqtt_tx`, `mbf_mqtt_rx` | MQTT publish / receive (`s1_mqtt_bridge.c`, `mqtt_utk.c`) | Buddy 1; instructor OK given |
| IF-07 | all → S1 | event flag (owned by S1) | `flg_hb` | heartbeat bits `HB_*` (`hb_bits.h`); the Safety Monitor that reads them is not built yet | all |
| IF-08 | S1 writes, S2 reads | shared data, one aligned 32-bit word | `g_nav_map` (defined in S2) | barcode map: `nav_map_apply()` / `nav_map_snapshot()` (`nav_cmd.h`) | Buddies 1, 2, 3 |

Each interface is tested by at least one integration test
(`testing/integration/TEST_LOG.md`).

## 6. Kernel object budget

Limits are set in `config/config.h`. If the team needs more, raise the
limit there. Subsystem 5's objects are not created while `APP_WITH_S5` = 0
(in brackets: its stand-alone bench objects).

| Object | Limit | S1 | S2 | S3 | S4 | S5 |
|---|---|---|---|---|---|---|
| Task | 32 | 1 | 2 | 2 | | 1 (+1) |
| Semaphore | 16 | | | | | |
| Event flag | 16 | 1 | | 1 | | 2 (+2) |
| Mailbox | 12 | | | | | |
| Mutex | 12 | | | | | |
| Message buffer | 12 | 3 | 4 | | | (+1) |
| Fixed memory pool | 12 | | | | | |
| Variable memory pool | 12 | | | | | |
| Cyclic handler | 12 | | | | | |
| Alarm handler | 12 | | | | | |

(The kernel's USB and WiFi service tasks also use a few of these.)
