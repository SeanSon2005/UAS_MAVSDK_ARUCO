# Giannis UAS — Codebase Documentation

## Overview

Giannis is a **vision-guided autonomous drone landing system** built for a Raspberry Pi companion computer. It runs two processes — a **flight controller** and a **vision worker** — that communicate over UDP loopback. The drone takes off to a target altitude, uses a downward-facing camera to detect a red landing target, aligns itself over it with PID control, descends while maintaining alignment, and executes a final land command when it reaches a switch altitude. Hardware integration is handled through MAVSDK (flight controller link) and hardware PWM (servo for camera/mechanism positioning).

---

## Runtime Architecture

```
┌──────────────────────────────────┐     UDP (vision_sample)     ┌──────────────────────────────────┐
│       Main Process               │ ◄────────────────────────── │       Vision Worker Process       │
│                                  │                             │                                  │
│  FlightController                │     UDP (control_status)    │  VisionController                │
│  ├─ MavsdkLink (MAVSDK)         │ ──────────────────────────► │  ├─ Picamera2Camera              │
│  ├─ TelemetryCache              │                             │  ├─ ColorDetector                │
│  ├─ OffboardPublisher           │                             │  ├─ PreviewRenderer              │
│  ├─ PID controller              │                             │  ├─ VisionSampleUdpSender        │
│  ├─ ServoManager                │                             │  └─ ControlStatusReceiver        │
│  ├─ VisionUdpReceiver           │                             │                                  │
│  └─ ControlStatusUdpSender      │                             │                                  │
└──────────────────────────────────┘                             └──────────────────────────────────┘
```

- **Main process** (`app.main`): orchestrates the flight mission — connect, arm, takeoff, align, descend, land.
- **Vision worker process** (`app.vision_worker`): captures camera frames, detects the red target, publishes `vision_sample` packets, and renders a debug preview window.
- **IPC**: two unidirectional UDP channels on localhost — one for vision samples (vision → flight) and one for control status (flight → vision).

---

## Mission Sequence

| Phase | Description |
|---|---|
| **INIT** | Default state before connection. |
| **TAKEOFF** | Ascend at a constant velocity until target altitude is reached (or timeout). Servo is moved to landing position. |
| **ALIGN** | Hover at altitude while PID-correcting horizontal position using vision error. Runs for a configurable timeout. |
| **LAND_DESCENT** | Continue PID alignment while descending. If vision is lost for too long, descent pauses. At switch altitude, issues a MAVSDK land command. |
| **SHUTDOWN** | Stops offboard mode, sends land command if still armed, tears down all subsystems. Always executes even on error. |

---

## Directory Structure & File Responsibilities

### Root

| File | Responsibility |
|---|---|
| [main.py](main.py) | Top-level entry point. Delegates to `app.main.main()`. |
| [requirements.txt](requirements.txt) | Python dependencies: numpy, opencv, rpi-hardware-pwm, mavsdk, picamera2, PyYAML. |
| [config/default.yaml](config/default.yaml) | Single source of truth for all tuneable parameters (drone connection, takeoff/landing altitudes and velocities, PID gains, IPC ports, servo config, vision HSV thresholds, etc.). |
| [runbook.md](runbook.md) | Operational runbook: how to start the system, log locations, failure handling notes, and bench validation steps. |
| [docs_architecture.md](docs_architecture.md) | Brief architectural overview (runtime topology, subsystem list, mission sequence, entrypoints). |

---

### `src/app/` — Application Bootstrap & Wiring

| File | Responsibility |
|---|---|
| [src/app/main.py](src/app/main.py) | Application entry point. Spawns the vision worker as a separate `multiprocessing.Process`, then runs the flight controller's async event loop in the main process. Ensures the vision worker is terminated on exit. |
| [src/app/vision_worker.py](src/app/vision_worker.py) | Target function for the vision worker process. Builds a `VisionController` via wiring and runs its async loop. Catches `KeyboardInterrupt` for clean shutdown. |
| [src/app/wiring.py](src/app/wiring.py) | **Dependency injection / composition root.** Contains `build_flight_controller()` and `build_vision_controller()` factory functions that load config, create loggers, and wire up all concrete implementations. Also defines `LoggerFactory` which creates named loggers that output to both stdout and rotating log files. |

---

### `src/config/` — Configuration Loading & Schemas

| File | Responsibility |
|---|---|
| [src/config/load.py](src/config/load.py) | Loads `config/default.yaml` from the project root and constructs a fully-typed `UasConfig` object. All raw YAML values are explicitly cast to their correct Python types. |
| [src/config/schemas.py](src/config/schemas.py) | Typed dataclass schemas for every configuration section: `DroneConfig`, `TakeoffConfig`, `LandingConfig`, `AlignmentConfig`, `RuntimeConfig`, `IpcConfig`, `ServoConfig`, `VisionConfig`, and the top-level `UasConfig` aggregate. All use `@dataclass(slots=True)` for memory efficiency. |

---

### `src/comms/` — Inter-Process Communication

| File | Responsibility |
|---|---|
| [src/comms/protocol.py](src/comms/protocol.py) | Defines the **IPC wire protocol**. Contains `VisionSample` and `ControlStatus` dataclasses (the two message types), encode/decode functions (`encode_payload`, `decode_vision_sample`, `decode_control_status`), and protocol versioning (`PROTO_VERSION = 1`). Messages are serialised as compact JSON over UTF-8. Decoding is strict — any missing field, wrong type, or version mismatch returns `None`. |
| [src/comms/udp.py](src/comms/udp.py) | **UDP transport layer** with four classes: |
| | **`VisionUdpReceiver`** — asyncio datagram endpoint used by the flight controller to receive `vision_sample` packets. Uses `asyncio.DatagramProtocol` for non-blocking reception. |
| | **`ControlStatusUdpSender`** — non-blocking UDP socket used by the flight controller to publish `control_status` packets to the vision process. |
| | **`VisionSampleUdpSender`** — non-blocking UDP socket used by the vision worker to publish `vision_sample` packets to the flight controller. |
| | **`ControlStatusReceiver`** — blocking-mode UDP socket (set to non-blocking) used by the vision worker to poll for the latest `control_status` packet. Drains the socket buffer on each `poll()` call and keeps only the most recent message. |

---

### `src/flight/` — Flight Control Logic

| File | Responsibility |
|---|---|
| [src/flight/controller.py](src/flight/controller.py) | **`FlightController`** — the central orchestrator. Holds references to all flight subsystems (link, telemetry, vision receiver, status sender, actuator, PID, state). Its `run()` method executes the mission phases sequentially: TAKEOFF → ALIGN → LAND_DESCENT. Handles vision sample ingestion (with sequence deduplication), publishes control status each tick, and runs a comprehensive `shutdown()` that always attempts to land and clean up resources. |
| [src/flight/offboard.py](src/flight/offboard.py) | **`OffboardPublisher`** — an asyncio background task that sends the current desired velocity to the flight controller at a fixed rate (`loop_hz`, default 50 Hz). This is the MAVSDK offboard heartbeat — if it stops, the autopilot will exit offboard mode. Tracks timing jitter and logs diagnostics every 0.5 s. Uses a wall-clock tick scheduler to maintain consistent frequency. |
| [src/flight/pid.py](src/flight/pid.py) | **`PID`** — a general-purpose 2D PID controller using NumPy arrays. Supports configurable Kp, Ki, Kd gains, output clamping (`vmax`), and a `reset()` method with optional initial measurement to avoid derivative kick. Used to convert pixel-space alignment errors into body-frame velocity commands. |
| [src/flight/state.py](src/flight/state.py) | Core state types and enums: |
| | **`MissionPhase`** — enum: INIT, TAKEOFF, ALIGN, LAND_DESCENT, SHUTDOWN. |
| | **`StaleVisionMode`** — enum: FRESH (sample within TTL), HOLD (stale but within grace window — keep last velocity), ZERO (too stale — zero velocity). |
| | **`VelocityCommand`** — dataclass: `vx`, `vy`, `vz`, `yaw_rate` (all default 0.0). |
| | **`VisionState`** — dataclass: tracks `target_visible`, last valid velocities, and timestamp of last valid target sighting. |
| | **`ControlState`** — dataclass: tracks current phase, armed status, offboard status, and whether a land command has been sent. |

---

### `src/flight/flight_states/` — Mission Phase Implementations

| File | Responsibility |
|---|---|
| [src/flight/flight_states/common.py](src/flight/flight_states/common.py) | **Shared domain logic** (pure functions and data structures) used by alignment and landing phases: |
| | **`AlignPolicyInput` / `AlignPolicyOutput`** — input/output dataclasses for the alignment policy. |
| | **`LandingPolicyInput` / `LandingPolicyOutput`** — input/output dataclasses for the landing policy. |
| | **`stale_mode_from_age()`** — classifies vision sample freshness into FRESH / HOLD / ZERO based on configurable TTL thresholds. |
| | **`compute_xy_from_vision()`** — the core alignment policy. When vision is FRESH and target is visible, runs the PID controller on pixel errors and outputs clamped body-frame velocity. When HOLD, retains the last valid velocity. When ZERO, resets PID and outputs zero velocity. |
| | **`should_allow_descent_without_fresh_vision()`** — safety gate: allows blind descent only if the target was seen recently enough AND altitude is above the switch threshold. |
| | **`compute_landing_command()`** — the descent policy. At or below switch altitude, triggers the land command. With fresh vision, descends at configured velocity while maintaining XY correction. Without fresh vision, descent is conditionally allowed or halted based on the safety gate. |
| [src/flight/flight_states/takeoff.py](src/flight/flight_states/takeoff.py) | **Takeoff phase** — commands a constant downward velocity (NED: negative vz = ascend) until the telemetry altitude reaches the target. Moves the servo to landing position at phase start. Raises `RuntimeError` on timeout. |
| [src/flight/flight_states/alignment.py](src/flight/flight_states/alignment.py) | **Alignment phase** — runs a fixed-rate loop for up to `alignment.timeout_s` seconds. Each tick calls `compute_xy_from_vision()` with the latest vision sample to produce XY velocity corrections. Publishes control status and logs state every 0.5 s. |
| [src/flight/flight_states/landing_descent.py](src/flight/flight_states/landing_descent.py) | **Landing descent phase** — combines alignment with vertical descent. Each tick runs `compute_xy_from_vision()` for XY correction, then `compute_landing_command()` to determine vertical velocity (or trigger a land command). If switch altitude is reached, calls `send_land_command_if_needed()` and breaks. At phase end (or timeout), ensures land command is sent and velocity is zeroed. |

---

### `src/hw/` — Hardware Abstraction

| File | Responsibility |
|---|---|
| [src/hw/mavsdk_link.py](src/hw/mavsdk_link.py) | **`MavsdkLink`** — wraps the MAVSDK `System` object. Provides `connect_and_wait_ready()` (waits for connection, sensor calibration, and local position estimate), `arm()`, `offboard_start()` / `offboard_stop()`, `set_velocity_body()`, and `land()`. This is the only file that directly calls MAVSDK APIs. |
| [src/hw/servo_pwm.py](src/hw/servo_pwm.py) | **`ServoManager`** — drives a servo via Linux hardware PWM (`rpi-hardware-pwm`). Converts angles (0–180°) to duty cycle steps using a linear mapping. Used to tilt the camera/mechanism to landing position during takeoff. Gracefully handles initialization failures (e.g., missing sudo permissions). |
| [src/hw/telemetry.py](src/hw/telemetry.py) | **`TelemetryCache`** — runs a background asyncio task that subscribes to the MAVSDK `position_velocity_ned` telemetry stream and caches the latest altitude (NED down → positive-up conversion). Provides a thread-safe `get_altitude()` accessor. Filters out NaN and infinity values. |

---

### `src/vision/` — Vision Pipeline

| File | Responsibility |
|---|---|
| [src/vision/camera_picam2.py](src/vision/camera_picam2.py) | **`Picamera2Camera`** — async wrapper around Picamera2. Configures a 640×480 BGR888 video stream at ~30 fps. `capture_frame()` runs the blocking capture in a thread executor to avoid blocking the event loop. |
| [src/vision/detect_color.py](src/vision/detect_color.py) | **`ColorDetector`** — red target detection using OpenCV. Pipeline: Gaussian blur → convert to HSV → dual-range red mask (handles hue wraparound at 0°/180°) → find contours → select largest → filter by minimum area → compute centroid via image moments → return pixel error from frame centre. Returns `err_x_px`, `err_y_px`, the binary mask, centroid coordinates, largest contour, and contour area for debug rendering. |
| [src/vision/controller.py](src/vision/controller.py) | **`VisionController`** — the vision process main loop. Captures frames, runs detection (in a thread), rate-limits UDP sample sending (`send_hz_cap`), polls for incoming control status, and rate-limits debug preview rendering (`render_fps`). Builds `VisionSample` packets with sequence numbers and monotonic timestamps. Cleans up all resources in its `finally` block. |
| [src/vision/render_cv.py](src/vision/render_cv.py) | **`PreviewRenderer`** — OpenCV `imshow`-based debug visualization. Draws a crosshair at frame centre, the detected contour (green), the centroid dot, and an overlay showing target visibility, pixel errors, contour area, and velocity commands. When control status is available, also displays the current mission phase, velocities, and altitude. Can be closed by pressing 'q'. Gracefully disables itself if the display environment is unavailable. |

---

### `tests/` — Test Suite

| File | Responsibility |
|---|---|
| [tests/test_messages.py](tests/test_messages.py) | **Unit tests for IPC protocol.** Tests `VisionSample` round-trip encode/decode, rejection of wrong message types, and rejection of incomplete payloads. |
| [tests/test_policies.py](tests/test_policies.py) | **Unit tests for flight policies.** Tests `stale_mode_from_age()` classification, `should_allow_descent_without_fresh_vision()` allow/block cases, and `compute_landing_command()` switch-altitude trigger. |
| [tests/unit/test_domain_policies.py](tests/unit/test_domain_policies.py) | **Unit test for alignment policy.** Uses a fake PID and a fake vision sample to verify that `compute_xy_from_vision()` produces FRESH mode and `target_visible=True` when given a fresh sample with a target. |
| [tests/unit/test_ipc_contract.py](tests/unit/test_ipc_contract.py) | **Unit test for `ControlStatus` round-trip.** Encodes and decodes a `ControlStatus` message and verifies field integrity. |
| [tests/integration/test_vision_policy_loop.py](tests/integration/test_vision_policy_loop.py) | **Integration test.** Verifies that in HOLD mode with an expired vision window, the landing policy correctly blocks descent (vz = 0). |

---

### `tools/` — Developer Utilities

| File | Responsibility |
|---|---|
| [tools/aligner_preview.py](tools/aligner_preview.py) | **Standalone alignment preview tool.** Runs the full vision + alignment pipeline locally without a flight controller or drone. Captures camera frames, runs color detection, feeds the results through `compute_xy_from_vision()` with a real PID controller, and renders the debug preview. Useful for tuning PID gains, HSV thresholds, and verifying the alignment logic on a bench. |

---

## Key Data Flows

### Vision → Flight (alignment correction)

1. `Picamera2Camera.capture_frame()` → raw frame
2. `ColorDetector.detect_with_debug()` → `err_x_px`, `err_y_px`, debug artefacts
3. `VisionController` builds a `VisionSample` and sends via `VisionSampleUdpSender`
4. `VisionUdpReceiver` (asyncio datagram protocol in flight process) delivers to `FlightController._on_vision_sample()`
5. Flight state loop calls `compute_xy_from_vision()` → PID → `VelocityCommand`
6. `OffboardPublisher` continuously sends the latest velocity to MAVSDK at 50 Hz

### Flight → Vision (status overlay)

1. Each flight tick calls `FlightController.publish_control_status()` → `ControlStatusUdpSender.publish()`
2. `ControlStatusReceiver.poll()` in the vision process drains the socket
3. `PreviewRenderer.render()` overlays phase, velocity, and altitude on the debug window

### Vision Staleness Handling

The system tracks how old the most recent vision sample is and classifies it:

- **FRESH** (age ≤ `sample_ttl_s` = 0.15 s): use PID output directly.
- **HOLD** (age ≤ `drop_to_zero_after_s` = 0.30 s): keep the last valid velocity (coast).
- **ZERO** (age > 0.30 s or no sample): reset PID, zero all velocities.

During landing descent, an additional safety gate prevents blind descent if the target hasn't been seen within `no_vision_descent_max_s` (2.0 s) or the drone is already at or below the switch altitude.

---

## Configuration Reference

All values are set in [config/default.yaml](config/default.yaml):

| Section | Key Parameters |
|---|---|
| `drone` | `connection_string` — MAVSDK serial connection URI |
| `takeoff` | `alt_m` (target altitude), `vel_mps` (ascent speed), `timeout_s` |
| `landing` | `switch_alt_m` (altitude to issue land cmd), `vel_mps` (descent speed), `descent_timeout_s`, `no_vision_descent_max_s` |
| `alignment` | `timeout_s`, `pid_kp`, `pid_ki`, `pid_kd` (2D gains) |
| `runtime` | `loop_hz` (50), `max_speed_mps` (velocity clamp) |
| `ipc` | `host`, UDP ports, `sample_ttl_s`, `drop_to_zero_after_s`, `render_fps`, `send_hz_cap` |
| `servo` | PWM chip/channel, duty cycle calibration, `angle_landing` |
| `vision` | `frame_color_order`, HSV red ranges (two ranges for hue wraparound), `red_min_area_px` |
| `logs` | `dir` — log output directory |

---

## How to Run

```bash
# Activate virtual environment
source .venv/bin/activate

# Run the full system
PYTHONPATH=src python3 main.py

# Run the alignment preview tool (bench testing, no drone needed)
PYTHONPATH=src python3 tools/aligner_preview.py

# Run tests
PYTHONPATH=src python3 -m pytest tests/
```
