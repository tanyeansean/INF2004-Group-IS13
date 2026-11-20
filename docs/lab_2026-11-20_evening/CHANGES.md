# SUS EVENING TEST 20/11/26 – NOT TESTED ON THE CAR

> **Use the noon testing branch if this does not work, and work from there:**
> `aurelius/v19-lab-noon-tested-2026-11-20` (tested on the car 11:54-12:33: A 5/5, Q 7/7, F 2/2, Z 4/6).
>
> This branch = noon + the afternoon fixes (#5, #6, #7 straight-on, #8, #9, #10 below). They were checked in
> simulation and unit tests only. Flash it, and if anything is worse than noon, go back to the noon branch.

# Changes to v19 (lab, 20 Nov 2026)

Base: `v19-latest-20/11/26` @ `a34497a`. Local only, not pushed.
Last updated: 20 Nov 2026, 13:20.

## Use
| File | For |
|---|---|
| `IS13_v19lab_12_DRIVE.uf2` (not in git: hotspot password compiled in; ask Aurelius) | Driving. Lift stop ON, obstacle ON, `dbg/bar` ON |
| `IS13_v19lab_11_WIDTH_BENCH_TEST_no_driving.uf2` (not in git) | Box width check on the desk (car does not drive) |

- Both are built for the Decoraze hotspot with the laptop at 172.20.10.7. Any other network needs a rebuild.
- Run at dashboard speed 10.
- Always Stop, then Calibrate, then Start.
- Build from source: this branch as it is, `make NET=lwip MQTT=1` (Wi-Fi settings from your own `config/wifi_secrets.h`). Gives a file byte-identical to `12_DRIVE`.

## Changes
| # | Area | Change | Why (evidence) |
|---|---|---|---|
| 1 | Barcode sampler | 8-sample up/down counter on GP7 | Chatter-proof edges |
| 2 | Barcode decoder | Bars compared with bars, spaces with spaces (2 wide bars + 1 wide space) | v19 failed small barcodes when the sensor biases bar width |
| 3 | Barcode timing | Edges timed by wheel distance, not clock (sensor 90 mm from centre) | Steering wobble distorted widths (model 3 → 27 reads) |
| 4 | Barcode abort | Floor 150 → 600 ms | Lost the first bar on every forward pass (replay 1/4 → 4/4) |
| 5 | Barcode repair: speck | Window 30 → 32 edges; a tiny element is merged and the frame re-decoded | Z failing at 12:11 and 12:27 |
| 6 | Barcode repair: cut-off / run-on end | End mark missing 1–3 elements, or last bar running into something dark, is completed from the other `*` | A failing at 10:06–10:33; simulation 9/9 fails before |
| 7 | Junction | Unreadable barcode goes straight on, no reverse (`JN_RESCAN_MAX 0`); "both white = line lost" removed from the reverse | 4 ERROR stops from rescans |
| 8 | Speed | Dashboard speeds below 10 are raised to PWM 42 (speed 10) | Speed 4 did not move; speed 8 crawled and failed Z |
| 9 | Obstacle | Fine sweep 5 pings per angle, median kept | Width jumped 18/14/11/0 for the same box |
| 10 | Obstacle | Reported width uses beam 17° (`OA_BEAM_WIDTH_HALF_DEG`); bypass still uses 14° | Box is 15 cm wide face / 10 cm narrow face; read 18 |
| 11 | Diagnostic | `dbg/bar` MQTT topic (letter, edges, widths); lower-case letter = read with a repair | Lab debugging |

Repairs (5, 6) still require both `*`, a valid letter, clean splits and both gap guards.

## Test results
- Decoder unit tests: 163 checks, 0 failed.
- Today's 358 logged passes replayed: 8 more reads than v19's rules, 0 wrong letters.
- Clutter stress, 9 million barcode-like edges: 10 false reads (v19: 5).
- 20 simulated laps (`simulation_20_laps.txt` here): 15/20 pass. Barcodes 74/80, 0 misreads. All 6 barcode fails are 1–2 mm specks on 3–4 mm barcodes. Box width within ±2 cm in 18/20 runs, and in all 10 at ~20 cm.
- On the lab car (files 8–9, before #5, #6, #10): F, Q, A, Z all read at speed 10; no ERROR stops.

## Not code (setup)
- IMU dropped out 3 times (fault `0x0020`), and once read 63.5° tilt on the hump. Secure the board and its plugs.
- Barcode screw: set so the light is solidly on over bare mat and off on a thin bar.
- Barcode paper must be flat and clean, with white past both ends.

## Open
- [ ] Width bench test: 15 / 10 cm faces at 20 and 30 cm. Retune `OA_BEAM_WIDTH_HALF_DEG` (1° ≈ 0.7 cm).
- [ ] Lift stop test while driving (after the IMU is secured).
- [ ] 5 hands-off laps on 12_DRIVE.
- [ ] Measure the barcode sensor's distance from the centre line (`S3_BAR_ARM_MM`, now 90).
- [ ] An unmapped letter stops the car. Change to straight on?
- [ ] Barcode bars read as junctions when the car is off the line (12:18, 8 events).
- [ ] One obstacle ERROR (10:03), cause unknown.
- [ ] Change the hotspot password.
