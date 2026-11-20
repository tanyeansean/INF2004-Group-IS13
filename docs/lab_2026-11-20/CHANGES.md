# IS13 – v19 as tested on the lab car, 20 Nov 2026 ~12:00

Base: `v19-latest-20/11/26` @ `a34497a`. Local only, not pushed.
This is the version that ran on the car from 11:54 to 12:33. It came before the simulations and the later fixes; for those, see `IS13_v19_lab_2026-11-20.zip` (file 12).

## Use
`IS13_v19lab_09_TESTED_1200.uf2` (not in git: it has the hotspot password compiled in; ask Aurelius)
- Built for the Decoraze hotspot with the laptop at 172.20.10.7.
- Run at dashboard speed 10. Stop, then Calibrate, then Start.
- **Lift stop OFF** in this build. A hump or a lift is reported but does not stop the car.
- Obstacle ON. `dbg/bar` ON.
- Build from source: this branch as it is, then `make NET=lwip MQTT=1`. That gives a file byte-identical to this .uf2.

## Changes to v19
| # | Area | Change | Why |
|---|---|---|---|
| 1 | Barcode sampler | 8-sample up/down counter on GP7 | Chatter-proof edges |
| 2 | Barcode decoder | Bars compared with bars, spaces with spaces | v19 failed small barcodes when the sensor biases bar width |
| 3 | Barcode timing | Edges timed by wheel distance (sensor 90 mm from centre) | Steering wobble distorted widths |
| 4 | Barcode abort | Floor 150 → 600 ms | Lost the first bar on every forward pass (replay 1/4 → 4/4) |
| 5 | Junction rescan | "Both front sensors white = line lost" removed from the reverse | Every rescan ended in ERROR (4 times) |
| 6 | Junction rescan | Rescans 3 → 1 | 3 rescans cost ~45 s per unreadable barcode |
| 7 | Build switches (0006) | `S3_BAR_DIST_CLOCK 1`, `S3_BAR_ARM_MM 90`, `S3_BAR_DIAG_MQTT 1`, `S4_LIFT_STOPS 0` | As flashed |

Hardware and setup done in the lab: barcode sensor screw set by the light, white paper under the end of barcode A, and the IMU reseated.

## Lab result, 11:54–12:33 (`lab_results_1146-1232.txt`)
| Barcode | Read / passes |
|---|---|
| A | 5 / 5 |
| Q | 7 / 7 |
| F | 2 / 2 |
| Z | 4 / 6 |

- Every turn matched the map.
- Both Z failures were a dark speck in a gap (fixed later by the speck repair in file 12). One of them was also at speed 8, which crawls at 3 cm/s.
- Speed 10 is the lowest that reads reliably.
- Obstacle: bypassed and back on the line 8 of 8 times while driving (12:06–12:30). Width read 0, 15, 11, 18, 14, 18, 18, 18 cm for the same box (fixed later in file 12). The scans after 12:31 were with the car stopped.
- IMU: fault `0x0020` (not answering) from 12:02 to 12:19, until it was reseated. Lift flags (`0x0004`) at 12:19–12:31 did not stop the car because the lift stop is off in this build.
- 12:18: 8 false junctions in 10 s (car off the line), ending in ERROR at 12:18:34.
