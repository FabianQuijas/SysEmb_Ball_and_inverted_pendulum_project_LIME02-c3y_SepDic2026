# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Active initiatives

- **Documentation site**: `website/` is an Astro Starlight site published to
  GitHub Pages at <https://ferrolho.github.io/rotary-inverted-pendulum/>, with
  an in-browser demo of the deployed policy (official MuJoCo WASM + the weights
  parsed from the sketch's committed default weights header).

  **All prose documentation lives in `website/src/content/docs/` — there is no
  `docs/` directory any more.** That tree is the source of truth, not a
  generated copy: edit those files directly. Pages are plain `.md`; `.mdx` only
  where a page genuinely needs a component.

  **Never hard-code firmware constants in prose.**
  `website/scripts/extract_constants.mjs` reads them from `RLControl.ino`,
  its selected weights header and `pendulum_env.py` into `src/generated/constants.json`;
  render them via `ConstantsTable.astro` or by importing that JSON in `.mdx`.
  This guard exists because the runbook asserted a 35 Hz control rate for weeks
  after the firmware moved to 50 Hz (reconciled in 352c809).

  Only machine-readable facts are generated (`prepare.mjs`: constants, policy
  weights, Draco decoder). Meshes, the MJCF snapshot, visual transforms and the
  replay capture are committed; regenerate with `scripts/export_assets.py`.

  Tests: `npm test` (the flashed policy still balances in MuJoCo; the 3D
  transform chain is correct) and `npm run test:smoke` (real headless browser
  against the built site).

- **RL controller**: a multi-phase effort to replace the hand-tuned PID with a learned swing-up + balance policy. The entry point is `website/src/content/docs/train/pipeline.mdx` — the pipeline from bare rig to standalone balancing. Read that file before working on anything under `firmware/LowLevelServer/`, `firmware/RLControl/`, or `policy/`.
  **Canonical operating point: 50 Hz, velocity mode, ±3.5 rad/s, K=4 frames, 4-tap actuator action smoothing, mirror augmentation, observation-staleness DR 2–20 ms.** Every entry point — `train_sac.py`, `curriculum_train.sh`, `distill_student.sh`, `finetune_async.py`, `run_policy.py`, `RLControl.ino` — is set to this, so a bare end-to-end run reproduces the current champion; a bare `train_sac.py` run writes a `config.json` identical to the champion's on all 17 shared keys. Do not change one default in isolation, since train/fine-tune/deploy must agree or the policy silently misbehaves (`run_config.check_config` aborts on mismatch). Legacy behaviours are reachable explicitly: `--action-mode accel`, `--no-firmware-obs-model`, `--no-mirror-augment`, `--reward-stillness-bonus-weight 0`. `website/src/content/docs/reference/control-rate.md` concludes 35 Hz; that predates actuator action smoothing and is superseded (see the note at its top). Companion docs:
  - `website/src/content/docs/reference/transitions.md` — the `(s, a, r, s')` transition contract in plain English.
  - `website/src/content/docs/reference/transport-delay.md` — measured action-delay history and the decision log of hardware/firmware changes (including the position → acceleration action-mode switch).
  - `website/src/content/docs/reference/domain-randomization.md` — what is randomized, by how much, and why.
  - `website/src/content/docs/reference/async-control.md` — the threaded runtime that holds the configured control rate strictly during fine-tuning.
  - `website/src/content/docs/reference/control-rate.md` — how to pick `control_freq_hz` and `max_action_delta_rad` from sysid measurements.
  - `website/src/content/docs/train/sysid.md` — the measurement procedure for the inputs the two docs above depend on.

## Where physical parameters live

The rotary inverted pendulum has one source of truth per parameter
class. Updating a number in any other place is a bug — the chain is:

- **Pendulum body geometry** (mass, COM, inertia tensor): authored in
  Onshape → exported to `model/model.urdf` → parsed at import time by
  `policy/pendulum_geometry.py` →
  consumed by `pendulum_env.py` (MuJoCo sim + DR), `sysid_core.py`
  (friction derivation, sanity-check against measured period), and
  `website/scripts/export_assets.py` (glTF for the 3D demo). To change
  pendulum mass/COM/inertia, edit Onshape → export the URDF; nothing else.

- **Per-rig dynamic state** (viscous + Coulomb friction): measured by
  `sysid_wizard.py` from a free-swing recording on the actual hardware,
  written to `policy/sysid_params_<rig>.json`,
  loaded into `PendulumParams` alongside the URDF constants. These do
  vary between rebuilds (bearings, grease, temperature) and are the
  only quantities the sysid pipeline measures.

- **Arm geometry** (length, mass, COM): hard-coded constants in
  `pendulum_env.py` (`ARM_*`). `model/model.urdf`'s `arm` link *does*
  carry a CAD inertial, but nothing reads it and the two disagree (~6.1e-5
  vs ~4.7e-5 kg·m² about the motor axis; 62 mm vs 65 mm reach). Switching
  the env over to the URDF changes the training plant, so treat it as a
  plant change — not a refactor — and re-validate the champion.

- **Visual geometry**: the printable STLs in `model/meshes/`, authored in
  millimetres. `model/model.urdf` references them with `scale="0.001 …"`,
  and `website/scripts/export_assets.py` decimates the same files into
  glTF for the 3D demo. One file per part, so the URDF, the printed part
  and the website cannot end up on different CAD revisions — which is
  exactly what the now-deleted `.dae` exports did.

- **Hardware/firmware constants** (motor max accel, AS5600 resolution,
  hard-stop limits): module constants in `pendulum_env.py`, with the
  Arduino firmware as the upstream source for the motor-side values.

## Overview

This is a rotary inverted pendulum: a pendulum on a rotary base that must be
balanced upright by moving the base. It is balanced by a reinforcement-learning
policy — trained in MuJoCo, fine-tuned on the real rig, and distilled to 689
parameters that run standalone on an Arduino Nano. The repository holds the
mechanical design, the electronics, that pipeline, and the documentation site.

**There is one control stack: the learned policy.** The hand-tuned PID
(`PIDControl.ino`) and the Julia MPC/LQR exploration that preceded it were
removed in the reorganisation that put the RL pipeline at the centre of the
tree; both are reachable at commit `5fe3bfa` if you ever need them. Do not
reintroduce either without being asked.

## Project Structure

The layout follows the pipeline, not the implementation language:

- **`policy/`**: the RL pipeline, as a flat set of scripts run from that
  directory (`cd policy && uv run python …`). Sim env, SAC training, sysid,
  real-rig bridge, distillation, weight export and the analysis tools.
  `uv`-managed via `policy/pyproject.toml` + `uv.lock`.
- **`firmware/`**: Arduino C++ for the Nano.
  - `RLControl/`: the standalone learned controller — the end state
  - `LowLevelServer/`: state/command server used for tethered fine-tuning
  - `TestEncoder/`, `TestMotor/`, `TestSerial/`, `TestServer/`,
    `TestHeartbeat/`: bring-up sketches, used by the first-power-on guide
- **`model/`**: `model.urdf` plus the printable STLs in `meshes/` that it
  references.
- **`hardware/`**: wiring diagrams (drawio sources + exports) and component
  photos. Tracked in Git LFS — keep anything the website renders out of it.
- **`website/`**: all prose documentation (`src/content/docs/`) and the Astro
  Starlight site, including the in-browser MuJoCo demo.

## Hardware Architecture

The system uses:
- **Arduino Nano**: Microcontroller for sensor reading and motor control
- **AS5600 Magnetic Encoder**: Measures pendulum angle (I2C communication)
- **Stepper Motor (NEMA17)**: Rotates the base arm (via a TMC2209 — recommended, silent; DRV8825/A4988 also supported)
- **AccelStepper Library**: Controls stepper motor with acceleration profiles

Communication between Arduino and computer is via serial at 2,000,000 baud.

## Control Approaches

One controller, run two ways:

1. **Standalone** (the end state): the distilled policy runs entirely on the
   Nano, no laptop attached.
   - Firmware: `firmware/RLControl/RLControl.ino` + a `policy_weights_*.h`
   - Scored by `policy/analyze_onboard.py` over the telemetry stream

2. **Tethered** (training and evaluation): the Nano is a low-level server, and
   the host reads state, decides an action, and sends it back.
   - Firmware: `firmware/LowLevelServer/LowLevelServer.ino`
   - Client: `policy/lowlevel_client.py` → `real_env.py`, used by
     `finetune_async.py` and `run_policy.py`

## Serial Communication Protocol

### Binary protocol (LowLevelServer)
Commands are single bytes, some followed by a little-endian 4-byte float:
- `0x01`: Check ready (echoes `0x01`)
- `0x02`: Get state — 20-byte reply: `uint32` timestamp µs, then floats for
  motor position, pendulum position, motor velocity, pendulum velocity.
  Values are in the firmware frame — the one frame used everywhere, with no
  flips on either side of the wire; velocities are windowed finite
  differences, and the timestamp is the sample time, not the reply time.
- `0x03`: Set **angular acceleration** (rad/s²) — was `CMD_SET_TARGET` in
  position mode; the switch to accel is what made on-device deployment
  viable (see `website/src/content/docs/reference/transport-delay.md`)
- `0x04`: Engage motor
- `0x05`: Disengage motor
- `0x06`: Tare pendulum (re-zero to the current AS5600 reading)
- `0x07`: Set target motor position (rad) — position mode
- `0x08`: Zero the motor step counter to the arm's current position (called after the operator re-centres the arm between fine-tune episodes)

## Arduino Development

### Prerequisites
Libraries required (install via Arduino IDE Library Manager):
- [AccelStepper](https://www.airspayce.com/mikem/arduino/AccelStepper/)
- [AS5600](https://github.com/Seeed-Studio/Seeed_Arduino_AS5600) (included in `libs/`)

### Flashing Arduino
1. Open `.ino` file in Arduino IDE
2. Select Board: "Arduino Nano"
3. Select Port: `/dev/cu.usbserial-*` (macOS) or appropriate COM port
4. Upload sketch

### Key Arduino Concepts

**Stepper Motor Configuration:**
- Microstepping: set by the single `MICROSTEPS` constant in each sketch.
  **The recommendation is 1/32 → 6400 steps/revolution on either driver**,
  but the pin levels differ — TMC2209 MS1=HIGH/MS2=LOW, DRV8825
  M0=M1=M2=HIGH — because the two carriers decode the same board positions
  differently. Steps/rev, the speed cap and all rad↔step math derive from
  it, and it must match `MOTOR_MICROSTEPS` in `pendulum_env.py` (recorded
  per run as `motor_microsteps` in `config.json`). Both tables are in
  [`website/src/content/docs/build/electronics.md`](website/src/content/docs/build/electronics.md).
- Enable pin inverted (both TMC2209 and DRV8825 use active-low enable)
- Max speed: 200,000 steps/sec
- Acceleration: 100,000 steps/sec²

**AS5600 Encoder:**
- Provides 12-bit resolution (0-4095 raw values)
- Maps to 0-360° or 0-2π radians
- Handles multi-revolution tracking with wraparound logic
- **The tracked angle is an accumulator that never resets, so every sketch that
  accumulates it must reject implausible per-sample jumps** (>500 LSB): one
  corrupted I²C read otherwise offsets the angle for the rest of the run, and
  the controller then balances against a false vertical. Guarded in
  `LowLevelServer` and `RLControl`; deliberately not in `TestEncoder`,
  where seeing raw glitches is the point.
- Check magnet strength on startup

## Python Development

Everything lives in `policy/` as flat scripts, run from that directory. The
environment is `uv`-managed (`policy/pyproject.toml` + `uv.lock`):

```bash
cd policy
uv run python train_sac.py --help
```

On macOS anything that opens the MuJoCo viewer needs `uv run mjpython …`
rather than `python`.

## Common Development Tasks

### Testing Hardware
- **Test encoder**: Flash `TestEncoder/TestEncoder.ino`, open Serial Monitor/Plotter
- **Test motor**: Flash `TestMotor/TestMotor.ino`
- **Test serial communication**: Flash `TestSerial/TestSerial.ino`

### Hardware-in-the-Loop Testing
For automated testing with hardware connected, use the serial monitoring script:

```bash
./firmware/scripts/monitor_serial.sh <port> <baud_rate> <duration>
```

This script properly handles Arduino reset on serial connection and flushes old buffer data to provide clean output. Useful for verifying Arduino behavior during development without manual intervention.

Example:
```bash
./firmware/scripts/monitor_serial.sh /dev/cu.usbserial-10 115200 10
```

**Note:** This approach avoids common issues with direct `cat` or `stty` usage that can cause double resets or capture stale buffered data.

### Serial Port Issues
On macOS, the Arduino typically appears as `/dev/cu.usbserial-110` or similar. Update the `--port` argument if different.

### Current Limiting
Set the driver current limit with the onboard trim pot. **The TMC2209's Vref sets RMS current, the DRV8825's sets peak** — carrying a DRV8825 number over to a TMC2209 over-drives the motor (a factory-default 1.2–1.3 V ≈ 0.9 A RMS cooked the motor within 5 minutes). TMC2209: **≈0.9 V (0.64 A RMS)**; DRV8825: 0.45 V (0.9 A peak). Verify by temperature — warm, never too hot to touch. Full procedure and the wiring gotchas (remove the DRV8825's RESET–SLEEP bridge; coil pin order differs) in [`website/src/content/docs/build/electronics.md`](website/src/content/docs/build/electronics.md).

## Control Theory Notes

The underlying system state is `[motor_angle, pendulum_angle, motor_velocity,
pendulum_velocity]`. The policy does **not** see that directly — it sees a
24-dimensional observation (K=4 stacked frames of a trigonometric encoding).
The action is a velocity setpoint in ±3.5 rad/s, converted to an acceleration
command by a saturating P-law; see
`website/src/content/docs/reference/transitions.md`.

**State machine** (`RLControl.ino`, `enum State { WAITING, RUNNING }`):
- `WAITING`: motor disabled. Boot takes the pendulum's resting pose as the
  encoder zero, so hold it still and hanging through the settle delay.
- `RUNNING`: the policy drives swing-up and balance alike — there is no mode
  switch between them, which is the point of a learned controller. Straying
  past the hard limit disengages the motor and trips back to `WAITING`; `E`
  over serial re-arms it.

## URDF and Visualization

`model/model.urdf` is the canonical robot description. `policy/pendulum_geometry.py`
parses it for the pendulum's mass/COM/inertia; `website/scripts/export_assets.py`
decimates the STLs it references into glTF for the site's 3D demo.

## Serial Port Configuration

Arduino baud rate: 2,000,000 (high-speed for real-time control)
- Read timeout: 50ms (typical)
- Write timeout: 10ms (typical)

Always call `wait_until_ready(arduino)` after opening serial connection to synchronize with Arduino.
