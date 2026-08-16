# K230 Ball Vision — Steel-Ball Position Sensor for a Balance-Bar Car

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
![Platform](https://img.shields.io/badge/Platform-K230%20CanMV-blue)

Real-time steel-ball position detection on a **CanMV K230**, built for the
**2025 NUEDC (National Undergraduate Electronic Design Contest) H-Problem**:
a vehicle-mounted balance-bar ball control system (车载平衡滚球运动控制系统).

The camera looks straight down at a 25 cm white PVC balance bar. This program
finds the 1 cm steel ball inside a fixed ROI, converts its pixel position to
millimetres measured **from the left end of the bar**, and streams a
CRC-protected 10-byte binary frame to the main controller over UART at up to
**90 frames per second**.

> **中文版文档见 [README.zh-CN.md](README.zh-CN.md)**

---

## Table of Contents

- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Wiring](#wiring)
- [Quick Start](#quick-start)
- [UART Protocol](#uart-protocol)
- [Calibration](#calibration)
- [Key Parameters](#key-parameters)
- [How It Works](#how-it-works)
- [Integrating With a Main Controller](#integrating-with-a-main-controller)
- [Directory Structure](#directory-structure)
- [License & Acknowledgements](#license--acknowledgements)

---

## Features

- **90 FPS** detection at 640×480 (RGB565), processing only a small ROI
  cropped around the bar (≈601×65 px) for speed and robustness.
- **LAB-threshold based bar detection**: the white PPR bar is segmented in LAB
  color space, filtered by shape (aspect ratio 3–30), and stabilised by a
  rotated-rectangle tracker (exponential smoothing + corner-correspondence).
- **Circle-score ball detection**: the black steel ball is scored by
  roundness × fill-ratio similarity; a temporal continuity bonus allows
  tracking fast motion-blurred streaks instead of losing the ball.
- **Millimetre output**: the ball centre is projected onto the bar axis and
  mapped to `0..250 mm` (left end = 0), ready for the controller's PID loop.
- **CRC-8 protected UART frame** (`AA 55 ... CRC8`), with invalid-position
  reporting and a loss-hold mode (keeps the last valid position after
  startup, flagged with a dedicated status bit).
- **3-panel IDE preview** (original / binary / calibration overlay) plus
  lightweight FPS and frame logging for on-site tuning.

---

## Hardware Requirements

| Item | Requirement |
| --- | --- |
| Board | CanMV K230 (MicroPython firmware, e.g. CanMV v1.x) |
| Camera | Default MIPI sensor supported by the board (e.g. OV5647), 640×480@90fps capable |
| Bar | 25 cm white PPR pipe, 1 cm steel ball, camera rigidly mounted above the bar centre, optical axis roughly perpendicular to the bar |
| Lighting | Soft, uniform diffuse light; avoid strong specular highlights on the steel ball |

No YOLO/kmodel or external dependencies are required — this is pure
MicroPython + the built-in `image` library.

---

## Wiring

```
K230 IO5 (UART2 TX) ────►  main controller RX
K230 IO6 (UART2 RX) ◄────  main controller TX (optional, unused by this program)
K230 GND             ────  common ground
```

UART settings: **115200 baud, 8 data bits, no parity, 1 stop bit (8N1)**.

---

## Quick Start

1. Copy `k230_ball_vision.py` to the K230 filesystem (or open it with
   CanMV IDE and run).
2. First run **with the preview enabled** (it always is). Verify:
   - image orientation is correct (`CAMERA_HMIRROR` / `CAMERA_VFLIP`);
   - the ROI `(24, 146) – (625, 211)` tightly encloses the bar (adjust
     `ROI_TOP_LEFT` / `ROI_BOTTOM_RIGHT` for your mounting);
   - the white bar is detected (`LAB_THRESHOLD`), the ball is framed.
3. Tune `LAB_THRESHOLD` (LAB color space) so that the bar is white and the
   background + ball are black in the BINARY panel.
4. Calibrate (see [Calibration](#calibration)).
5. Wire IO5 → controller RX and check the logged frames, e.g.:

```
UART2 left_mm=125.0 valid=1 hold=0 tx=AA55...
```

6. For the highest frame rate, keep the preview open only while tuning;
   on the K230 LCD-less setup the preview is IDE-only (`Display.VIRT`).

---

## UART Protocol

Fixed **10-byte** binary frame, little-endian, sent every frame:

| Offset | Size | Content |
| ---: | --- | --- |
| 0 | 1 | `0xAA` header |
| 1 | 1 | `0x55` header |
| 2 | 1 | Sequence number, 0–255 wrap |
| 3–4 | 2 | Ball position `int16`, **0.1 mm units, distance from the left end** (`0..2500` = `0..250 mm`) |
| 5–6 | 2 | Ball velocity `int16` (currently always 0) |
| 7 | 1 | Confidence (255 valid / 0 invalid) |
| 8 | 1 | Flags |
| 9 | 1 | CRC-8/ATM over bytes 0–8 (poly 0x07, init 0x00) |

Flags:

| Bit | Meaning |
| --- | --- |
| 0 | Position valid |
| 1 | Calibration active (always set) |
| 2 | Position is HELD after a detection dropout (sent after startup delay `UART_LOSS_HOLD_DELAY_MS`) |

Invalid position is sent as `-32768` (`UART_INVALID_POSITION_X10`).

A PC-side parser for testing is provided in
[`docs/frame_parser.py`](docs/frame_parser.py).

---

## Calibration

The pixel→millimetre mapping is a **two-point linear fit** using the detected
bar geometry itself — no manual pixel anchors are needed:

1. The bar's two short-edge midpoints define the **bar axis**.
2. The ball centre is projected onto this axis.
3. The projection ratio is multiplied by `BAR_LENGTH_MM = 250`.

Because the mapping is relative to the live-detected bar, it automatically
tolerates small mounting shifts and bar rotation in the image. The only
assumption is that the camera sees the **full 25 cm bar** inside the ROI.

Verification procedure:

- Place the ball at −10, −5, 0, +5, +10 cm relative to the bar centre;
- Read `left_mm` on the serial log; expected: 75, 100, 125, 150, 175 mm
  (±2 mm recommended);
- Centre jitter peak-to-peak ≤ 3 mm at standstill;
- No frequent `BALL INVALID` while rolling slowly over the full travel;
- Detection frame rate ≥ 30 Hz with the preview closed (on IDE-display
  setups the preview itself caps display FPS at 60).

---

## Key Parameters

All tunables are grouped at the top of `k230_ball_vision.py` under
"User settings".

| Parameter | Default | Purpose |
| --- | --- | --- |
| `ROI_TOP_LEFT` / `ROI_BOTTOM_RIGHT` | (24,146) / (625,211) | Crop window; must tightly enclose the bar |
| `LAB_THRESHOLD` | (15,90,-66,-25,-28,39) | LAB range that selects the **white bar** |
| `BAR_MIN_LONG_SIDE_PX` etc. | 200 / 200 / 3.0 / 30.0 / 0.25 | Bar shape filter (long side, short side, aspect, fill) |
| `RECT_SMOOTH_ALPHA` | 0.25 | Bar corner smoothing; lower = smoother, more lag |
| `RECT_MAX_CORNER_JUMP_PX` | 50.0 | Reject corner jumps above this |
| `BALL_BLACK_THRESHOLD` | (0,20,-10,10,-10,10) | LAB range that selects the **black ball** |
| `BALL_MIN/MAX_DIAMETER_PX` | 8 / 80 | Ball size filter |
| `BALL_MIN/MAX_ASPECT_RATIO` | 0.50 / 6.00 | Allows motion-blurred streaks |
| `BALL_MIN_CIRCLE_SCORE` | 0.45 | Acquisition threshold for circularity |
| `BALL_MOTION_MIN_CIRCLE_SCORE` | 0.12 | Tracking threshold near previous position |
| `BALL_TRACK_SEARCH_DISTANCE_PX` | 160.0 | Continuity search radius |
| `BALL_EDGE_MARGIN_PX` | 3 | Reject candidates closer than this to the bar edge |
| `BALL_MAX_AXIS_DISTANCE_RATIO` | 0.25 | Ball must stay in the central 50 % of the bar width |
| `BAR_LENGTH_MM` | 250.0 | Physical bar length for mm mapping |
| `UART_LOSS_HOLD_DELAY_MS` | 5000 | After this startup time, hold last valid position on dropout |
| `DIAGNOSTIC_LOGS` | False | Enable full per-candidate rejection logging |

---

## How It Works

```text
snapshot (640x480, RGB565, 90fps)
  └─ crop ROI ──► find_blobs(LAB_THRESHOLD)      white bar
  │                 └─ shape filter ── min_corners() ── RectangleTracker (smooth/hold)
  │                                                    └─ bar axis = midpoints of short edges
  └─ binary image ─► find_blobs(BALL_BLACK_THRESHOLD)  black ball inside bar bounding box
                        └─ geometry + circle-score + continuity ── PointTracker
                                                                   └─ project onto bar axis
                                                                      └─ ratio x 250mm
                                                                         └─ 10-byte UART frame
```

Detection pipeline notes:

- The **bar is detected first** and stabilised; the ball search ROI is derived
  from the bar bounding box, which makes the ball detector immune to
  background changes outside the bar.
- The ball's `circle_score = roundness × fill_similarity` rejects pipe-end
  fragments and reflections without running a slow Hough-circle detector.
- During tracking, candidates within `BALL_TRACK_SEARCH_DISTANCE_PX` of the
  previous ball position get a lower circularity threshold and a score bonus,
  which keeps a fast (motion-blurred) ball locked.

---

## Integrating With a Main Controller

This program was designed to pair with an MSPM0G3507 main controller that:

- parses the `AA 55 ... CRC8` frame in a UART RX interrupt and validates
  `(flags & 0x03) == 0x03`, `0 ≤ position_x10 ≤ 2500`, confidence ≠ 0;
- treats `-32768` as "invalid" and never feeds it into the control loop;
- switches to a safe state when no valid frame arrives for ~100 ms;
- expects **distance from the left bar end in 0.1 mm** — the centre of a
  250 mm bar is therefore `1250`.

The velocity field is intentionally kept at 0: the controller computes
velocity by differencing positions at its own control rate, which avoids
double filtering.

---

## Directory Structure

```text
k230-ball-vision/
├── k230_ball_vision.py   # main program (the exact code that was flashed)
├── docs/
│   └── frame_parser.py   # PC-side 10-byte frame parser (protocol unit test)
├── README.md             # this file
├── README.zh-CN.md       # Chinese documentation
└── LICENSE               # MIT
```

---

## License & Acknowledgements

This project is released under the **MIT License** — see [LICENSE](LICENSE).

- This repository contains **only the vision part** of the 2025 NUEDC
  H-Problem system. No motor, line-following, or ball-balancing control
  code is included here.
- The balance-bar mechanism uses a closed-loop stepper motor (EMM); its
  vendor driver is not part of this repository.

> ⚠️ This is a student competition project. Use it as a reference and always
> validate safety-critical behaviour (limits, emergency stop) on your own
> hardware before any contest use.
