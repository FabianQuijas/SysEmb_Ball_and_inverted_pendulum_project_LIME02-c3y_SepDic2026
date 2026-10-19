/**
 * RLControl.ino — Standalone on-device RL controller for the rotary inverted pendulum.
 *
 * Runs a distilled student MLP (24 -> H -> H -> 1, ReLU/ReLU/tanh, float32
 * weights in PROGMEM) at a fixed CONTROL_FREQUENCY_HZ (50 Hz, the canonical
 * operating point) to swing up + balance the pendulum without any laptop
 * tether. Produced by `distill_student.sh` (behaviour cloning + DAgger at the
 * device transport) and exported by `export_weights.py`.
 *
 * Action mode: VELOCITY. The policy's tanh output is a velocity setpoint
 * (action × MAX_VELOCITY_RAD_S). A saturating P-law converts it to an
 * acceleration command each tick — with feedback from the controller's OWN
 * commanded-velocity integrator (v_cmd), NOT the measured velocity: the
 * measurement is quantised to ~±0.5 rad/s and multiplying that error by
 * the control-frequency gain would inject a ±17 rad/s² accel dither (the
 * defect removed from the tethered host on 2026-07-21). A slow
 * complementary correction from the measured velocity heals integrator
 * drift. The accel command is issued via moveByAcceleration(), exactly the
 * transport the policy was trained against.
 *
 * Observation: K=4 stacked frames, oldest -> newest, each frame
 *   [motor_pos, sin(theta), cos(theta), motor_vel, pen_vel, prev_action]
 * Velocities are (newest - oldest)/dt finite differences over a 5-sample /
 * 8 ms window of 500 Hz encoder+step-counter samples — the SAME
 * computation LowLevelServer's GET_STATE serves the tethered stack, so the
 * on-device policy sees identical measurement statistics to its training
 * and fine-tuning data. Flash weights from a policy trained with
 * `--action-mode velocity --obs-history-len 4` at the SAME rate as
 * CONTROL_FREQUENCY_HZ below; run_config.check_config enforces this.
 *
 * Step generation runs from a Timer1 ISR via FastAccelStepper. The main
 * loop is therefore free to spend ~10 ms on inference without stalling the
 * stepper ramp; between control ticks it services the 500 Hz sampler.
 *
 * Wiring: STEP must be on pin 9 (Timer1 OC1A on ATmega328); DIR on pin 2 and
 * ENABLE on pin 5 are unconstrained.
 *
 * Frame conventions (match `LowLevelServer` + `run_policy.py`):
 *   - The policy is trained in the Arduino's raw stepper/encoder frame —
 *     the ONE frame used everywhere: LowLevelServer sends GET_STATE
 *     unflipped and no host negates. Do not introduce flips anywhere.
 *   - phi = 0 means pendulum hanging down (encoder zeros at engage).
 *   - theta = wrap_pi(phi - pi); theta = 0 means upright.
 *   - motor_pos = 0 at engage (stepper position re-zeroed).
 *
 * Boot procedure:
 *   1. Power on or finish flashing. The sketch waits for a valid AS5600
 *      magnet detection, then waits a 1 s settle delay before engaging.
 *      The pose at the END of that delay becomes the policy's frame
 *      (phi = 0 = current pendulum angle, motor_pos = 0 = current
 *      stepper position). UX: have the pendulum hanging straight down
 *      before / during the 1 s delay; the LED is solid HIGH then drops
 *      to LOW once the motor engages.
 *   2. The policy runs; it swings up and balances.
 *   3. If the motor reaches a hard limit, the sketch disengages and waits.
 *      Press 'E' (after re-positioning) to re-engage with a fresh frame.
 *
 * Serial commands (500 kbaud, optional — the sketch is fully autonomous):
 *   'P' / 'p' : toggle CSV telemetry
 *   'E' / 'e' : engage motor (re-arm after a hard-limit trip)
 *   'D' / 'd' : disengage motor (manual stop)
 *   'M' / 'm' : print AS5600 magnet diagnostics
 *
 * Telemetry CSV (when toggled on, one line per control tick):
 *   t_us, motor_pos_rad×1000, phi_rad×1000, action×1000, state, freq_hz,
 *   overruns, latency_us, latency_max_us
 * latency_* are the sample->command delay (what the sim calls
 * obs_staleness_s); analyze_onboard.py treats them as optional so older
 * seven-column captures still parse.
 */

#include <FastAccelStepper.h>
#include <AS5600.h>
#include <Wire.h>

// Which champion header to compile. Named policy_weights_<rig>_<driver>_<microsteps>.h:
// the rig because each one is fine-tuned against its own measured friction, the
// driver and MICROSTEPS because the policy is trained against that quantisation.
// Both rigs now run TMC2209s, which is why the rig comes first — the driver alone
// stopped identifying a champion. Select at compile time, no file edits:
//   arduino-cli compile --upload -p <port> --fqbn arduino:avr:nano:cpu=atmega328 \
//     --build-property 'build.extra_flags=-DPOLICY_WEIGHTS_H="policy_weights_rig2_tmc2209_32.h"' \
//     firmware/RLControl
#ifndef POLICY_WEIGHTS_H
#define POLICY_WEIGHTS_H "policy_weights_rig1_tmc2209_32.h"
#endif
#include POLICY_WEIGHTS_H

// =============================================================================
// PINS
// =============================================================================
// On the ATmega328 (Nano), FastAccelStepper drives STEP from a Timer1
// hardware ISR — STEP must therefore be on pin 9 (OC1A) or pin 10 (OC1B).
// We use pin 9 by convention. DIR and ENABLE can be any digital pin.
const int DIR_PIN = 2;
const int STEP_PIN = 9;
const int ENABLE_PIN = 5;

// =============================================================================
// HARDWARE CONSTANTS
// =============================================================================
// Microstepping: the ONLY place to change it. Everything below derives from it.
// 32 is the recommended ratio on either driver, but the pin levels differ —
// DRV8825 M0=M1=M2=HIGH, TMC2209 MS1=HIGH/MS2=LOW. Legacy: 16 = TMC2209
// MS1=MS2=HIGH, 8 = DRV8825 M0=M1=HIGH. Must match MOTOR_MICROSTEPS in
// pendulum_env.py; see the microstepping tables on the electronics page.
const int MICROSTEPS = 32;
const long STEPS_PER_REVOLUTION = 200L * MICROSTEPS;
const float STEPS_PER_RAD = STEPS_PER_REVOLUTION / (2.0f * (float)PI);
const float RAD_PER_STEP = (2.0f * (float)PI) / (float)STEPS_PER_REVOLUTION;

// =============================================================================
// COMMUNICATION
// =============================================================================
const long SERIAL_BAUD_RATE = 500000;  // matches the test sketches
const long I2C_CLOCK_HZ = 400000;

// =============================================================================
// MOTOR ENVELOPE — mirrors LowLevelServer so the on-device transport matches
// what the policy trained against.
// =============================================================================
// Boot-time speed cap ≈ 5 rad/s: same as LowLevelServer's MOTOR_MIN_STEP_US.
// The velocity-mode P-law keeps the commanded speed inside ±MAX_VELOCITY_RAD_S
// (3.5); this cap is the physical backstop above it.
const uint32_t MOTOR_MIN_STEP_US =  // ≈ 5 rad/s, derived so it tracks MICROSTEPS
    (uint32_t)(1.0e6f * 2.0f * (float)PI / (5.0f * (float)STEPS_PER_REVOLUTION));

// Brake authority when past the rail — 150 rad/s², matching
// pendulum_env.py MAX_ACCEL_RAD_S2. See rail handling in control_tick().
const int32_t MOTOR_BRAKE_ACCEL_STEPS_S2 =
    (int32_t)(150.0f * STEPS_PER_RAD);

// =============================================================================
// CONTROL PARAMETERS
// =============================================================================
// Fixed control rate — MUST match the rate the flashed policy was trained at.
// 50 Hz is the canonical operating point across the whole stack (Python
// defaults included), so this agrees with a bare end-to-end runbook run. A
// policy trained at one rate and deployed at another produces a spinner, not
// a balancer.
const float CONTROL_FREQUENCY_HZ = 50.0f;
const unsigned long CONTROL_PERIOD_US = (unsigned long)(1000000.0f / CONTROL_FREQUENCY_HZ);
const float CONTROL_DT_S = 1.0f / CONTROL_FREQUENCY_HZ;

// Velocity-mode action scaling — must match training config.json:
//   max_velocity_rad_s = 3.5 (action scale), max_accel_rad_s2 = 150.
const float MAX_VELOCITY_RAD_S = 3.5f;
const float MAX_ACCEL_RAD_S2 = 150.0f;
// Complementary correction gain pulling v_cmd toward the measured velocity
// (per tick). Matches run_policy.py / real_env.py.
const float V_CMD_LAMBDA = 0.1f;

// Actuator-side action smoothing: the velocity law tracks the moving
// average of the last N policy outputs (1 = off). A boxcar of length 4
// has exact nulls at rate/2 and rate/4 — where learned PWM dither lives —
// so high-frequency action flips never reach the motor and cannot excite
// the base resonance. MUST match the policy's training config
// (action_smooth_window in config.json): the policy is trained expecting
// this filter's 1.5-tick delay. The raw action still feeds the
// observation's prev_action channel and telemetry.
const uint8_t ACTION_SMOOTH_WINDOW = 4;

// Observation stacking — must match training config.json (obs_history_len).
const uint8_t OBS_FRAMES = 4;
const uint8_t FRAME_DIM = 6;
#if defined(POLICY_OBS_DIM)
#if POLICY_OBS_DIM != 24
#error "policy_weights.h obs dim != 24 — flash weights from a K=4 velocity-mode policy"
#endif
#endif

// Motor position safety limits in policy frame.
//   SAFE_LIMIT (±125°) — matches MOTOR_SAFE_LIMIT_RAD in pendulum_env.py.
//   HARD_LIMIT (±132°) — slightly inside the ±135° mechanical hard stops.
//     Crossing it disengages the motor and returns to WAITING.
const float MOTOR_SAFE_LIMIT_RAD = 2.18166f;   // 125° × π/180
const float MOTOR_HARD_LIMIT_RAD = 2.30383f;   // 132° × π/180

// =============================================================================
// MEASUREMENT SAMPLER — port of LowLevelServer's 500 Hz ring buffer.
// GET_STATE-equivalent read: positions from the newest sample, velocities as
// (newest - oldest)/Δt over VEL_WINDOW samples (5 samples = 8 ms). The
// policy was trained and fine-tuned on exactly these statistics.
// =============================================================================
const uint16_t SAMPLE_PERIOD_US = 2000;
const uint8_t SAMPLE_BUFFER_SIZE = 16;
const uint8_t VEL_WINDOW = 5;

static int32_t motor_step_buf[SAMPLE_BUFFER_SIZE];
static float pen_rad_buf[SAMPLE_BUFFER_SIZE];
static uint32_t time_us_buf[SAMPLE_BUFFER_SIZE];
static uint8_t buf_head = 0;
static bool buf_filled = false;
static uint32_t last_sample_us = 0;

static void update_sample_buffer();

// =============================================================================
// STATE
// =============================================================================
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;
AS5600 as5600;

enum State { WAITING, RUNNING };
State state = WAITING;

// Observation frame ring: frames[0] = oldest ... frames[OBS_FRAMES-1] = newest.
static float frames[OBS_FRAMES][FRAME_DIM];

// Commanded-velocity integrator (rad/s) — the P-law feedback state.
static float v_cmd = 0.0f;
// Velocities as consumed by the policy on the last tick — telemetry only.
static float tele_motor_vel = 0.0f;
static float tele_pen_vel = 0.0f;
// Time spanned by the velocity finite-difference window on the last read.
// Designed 8 ms; grows when inference blocks the sampler and samples bunch.
static uint16_t tele_vel_span_us = 0;

// Action-smoothing ring (boxcar of the last ACTION_SMOOTH_WINDOW actions).
static float a_smooth_ring[ACTION_SMOOTH_WINDOW];
static uint8_t a_smooth_idx = 0;

// Telemetry / diagnostics
unsigned int loop_overruns = 0;
unsigned int loop_count_for_freq = 0;
unsigned long prev_time_us = 0;
bool print_enabled = false;
float last_action = 0.0f;

// =============================================================================
// UTILITY
// =============================================================================

static inline float wrap_pi(float x)
{
    // ((x + π) mod 2π) - π
    while (x >  (float)PI) x -= 2.0f * (float)PI;
    while (x < -(float)PI) x += 2.0f * (float)PI;
    return x;
}

static inline float read_motor_pos_rad()
{
    return (float)stepper->getCurrentPosition() * RAD_PER_STEP;
}

/**
 * Read the AS5600 with multi-revolution tracking; returns cumulative angle in
 * radians, zeroed by the most recent reset_pendulum_tracking() call. Raw
 * frame (no sign flip) — see header comment.
 *
 * Re-zeroing lives here because the policy's frame requires `phi = 0` ↔
 * pendulum hanging down, captured at every (re-)engagement after the user
 * has had time to position the rig.
 */
static volatile bool _encoder_zero_pending = true;

static void reset_pendulum_tracking()
{
    _encoder_zero_pending = true;
}

static float read_pendulum_rad()
{
    const long AS5600_RES = 4096;
    const long WRAP_THRESH = AS5600_RES / 2;
    const long PEN_RAW_MAX_DELTA_LSB = 500;
    const float RAD_PER_SEG = (2.0f * (float)PI) / (float)AS5600_RES;

    static long raw_prev = 0;
    static float pos = 0.0f;

    long raw = (long)as5600.rawAngle();

    if (_encoder_zero_pending)
    {
        raw_prev = raw;
        pos = 0.0f;
        _encoder_zero_pending = false;
        return 0.0f;
    }

    long delta = raw - raw_prev;
    if (delta >  WRAP_THRESH) delta -= AS5600_RES;
    if (delta < -WRAP_THRESH) delta += AS5600_RES;

    // Reject corrupted I2C reads: `pos` never resets, so one bad sample
    // offsets theta for the rest of the run (observed: 230 s of balance, then
    // flailing against a false vertical). Same guard as LowLevelServer.
    if (delta > PEN_RAW_MAX_DELTA_LSB || delta < -PEN_RAW_MAX_DELTA_LSB)
    {
        return pos;
    }

    pos += (float)delta * RAD_PER_SEG;
    raw_prev = raw;
    return pos;
}

// =============================================================================
// SAMPLER
// =============================================================================

static void reset_sample_buffer()
{
    buf_head = 0;
    buf_filled = false;
    last_sample_us = micros();
}

/** Take one sample if SAMPLE_PERIOD_US has elapsed. Called every loop() pass;
 *  self-paces to ~500 Hz. Costs one I2C read (~0.2 ms) when it fires. */
static void update_sample_buffer()
{
    uint32_t now_us = micros();
    if ((uint32_t)(now_us - last_sample_us) < SAMPLE_PERIOD_US) return;
    last_sample_us = now_us;

    motor_step_buf[buf_head] = stepper->getCurrentPosition();
    pen_rad_buf[buf_head] = read_pendulum_rad();
    time_us_buf[buf_head] = now_us;
    buf_head = (buf_head + 1) % SAMPLE_BUFFER_SIZE;
    if (buf_head == 0) buf_filled = true;
}

// Timestamp of the sample the policy actually read. Paired with the micros()
// taken just before moveByAcceleration, it measures sample->command latency —
// the quantity pendulum_env models as `obs_staleness_s`.
static uint32_t used_sample_us = 0;
static uint16_t latency_us = 0;
static uint16_t latency_max_us = 0;

/** GET_STATE-equivalent snapshot: newest positions + window-diff velocities. */
static void read_measured_state(float* motor_pos, float* phi,
                                float* motor_vel, float* pen_vel)
{
    uint8_t n_samples = buf_filled ? SAMPLE_BUFFER_SIZE : buf_head;
    if (n_samples == 0)
    {
        used_sample_us = micros();
        *motor_pos = read_motor_pos_rad();
        *phi = read_pendulum_rad();
        *motor_vel = 0.0f;
        *pen_vel = 0.0f;
        return;
    }

    uint8_t newest = (uint8_t)((buf_head + SAMPLE_BUFFER_SIZE - 1) % SAMPLE_BUFFER_SIZE);
    used_sample_us = time_us_buf[newest];
    *motor_pos = (float)motor_step_buf[newest] * RAD_PER_STEP;
    *phi = pen_rad_buf[newest];

    if (n_samples < VEL_WINDOW)
    {
        *motor_vel = 0.0f;
        *pen_vel = 0.0f;
        return;
    }
    uint8_t oldest = (uint8_t)((buf_head + SAMPLE_BUFFER_SIZE - VEL_WINDOW) % SAMPLE_BUFFER_SIZE);
    float dt_s = (float)((uint32_t)(time_us_buf[newest] - time_us_buf[oldest])) * 1e-6f;
    tele_vel_span_us = (uint16_t)((uint32_t)(time_us_buf[newest] - time_us_buf[oldest]));
    if (dt_s <= 0.0f)
    {
        *motor_vel = 0.0f;
        *pen_vel = 0.0f;
        return;
    }
    int32_t motor_step_delta = motor_step_buf[newest] - motor_step_buf[oldest];
    *motor_vel = ((float)motor_step_delta * RAD_PER_STEP) / dt_s;
    *pen_vel = (pen_rad_buf[newest] - pen_rad_buf[oldest]) / dt_s;
}

// =============================================================================
// POLICY FORWARD PASS
// =============================================================================
//
// 24 -> H -> H -> 1 MLP, ReLU/ReLU/tanh. Weights live in PROGMEM and are
// read with pgm_read_*(); only the H+H activation buffers + the input
// live in SRAM. H=16 is the production width: it fits inside the tick with
// margin, while H=32 float does not and silently sags the loop (measured at
// 35 Hz, 2026-07-22: 28.6 ms tick -> 25 Hz actual, which turned a balancing
// policy into a spinner). The current per-MAC cost and tick budget are
// recorded in website/src/content/docs/train/distill.md — keep the numbers in
// one place rather than duplicating them here. The imitation pipeline also
// works best at H=16 anyway.
// Stepping runs from the Timer1 ISR so inference never stalls the motor —
// but the 500 Hz measurement sampler DOES run in the main loop, so the
// hidden-layer row loops call update_sample_buffer() between rows
// (~0.7-0.9 ms each) to keep the velocity window fed during inference.

static void policy_forward(const float obs[POLICY_OBS_DIM], float* action)
{
    float h1[POLICY_HIDDEN_DIM];
    float h2[POLICY_HIDDEN_DIM];

    for (int i = 0; i < POLICY_HIDDEN_DIM; i++)
    {
        update_sample_buffer();  // keep the 500 Hz window fed during inference
        float sum = pgm_read_float(&POLICY_B1[i]);
        for (int j = 0; j < POLICY_OBS_DIM; j++)
        {
            sum += obs[j] * pgm_read_float(&POLICY_W1[i][j]);
        }
        h1[i] = sum > 0.0f ? sum : 0.0f;
    }

    for (int i = 0; i < POLICY_HIDDEN_DIM; i++)
    {
        update_sample_buffer();
        float sum = pgm_read_float(&POLICY_B2[i]);
        for (int j = 0; j < POLICY_HIDDEN_DIM; j++)
        {
            sum += h1[j] * pgm_read_float(&POLICY_W2[i][j]);
        }
        h2[i] = sum > 0.0f ? sum : 0.0f;
    }

    float sum = pgm_read_float(&POLICY_B3[0]);
    for (int j = 0; j < POLICY_HIDDEN_DIM; j++)
    {
        sum += h2[j] * pgm_read_float(&POLICY_W3[0][j]);
    }
    *action = tanhf(sum);
}

// =============================================================================
// OBSERVATION FRAMES
// =============================================================================

static void fill_frame(float* f, float motor_pos, float theta,
                       float motor_vel, float pen_vel, float prev_action)
{
    f[0] = motor_pos;
    f[1] = sinf(theta);
    f[2] = cosf(theta);
    f[3] = motor_vel;
    f[4] = pen_vel;
    f[5] = prev_action;
}

/** Shift the ring left (drop oldest) and write the newest frame in place. */
static void push_frame(float motor_pos, float theta,
                       float motor_vel, float pen_vel, float prev_action)
{
    memmove(&frames[0][0], &frames[1][0],
            sizeof(float) * FRAME_DIM * (OBS_FRAMES - 1));
    fill_frame(frames[OBS_FRAMES - 1], motor_pos, theta,
               motor_vel, pen_vel, prev_action);
}

// =============================================================================
// STATE MACHINE
// =============================================================================

static void prime_initial_state()
{
    // Mirror the sim/real reset: frame ring seeded with OBS_FRAMES copies
    // of the initial frame; velocities and prev_action start at zero;
    // v_cmd starts at zero (motor at rest).
    float motor_pos = read_motor_pos_rad();          // 0 after re-zero
    float phi = read_pendulum_rad();                 // 0 after re-zero
    float theta = wrap_pi(phi - (float)PI);
    for (uint8_t k = 0; k < OBS_FRAMES; k++)
    {
        fill_frame(frames[k], motor_pos, theta, 0.0f, 0.0f, 0.0f);
    }
    v_cmd = 0.0f;
    last_action = 0.0f;
    for (uint8_t k = 0; k < ACTION_SMOOTH_WINDOW; k++) a_smooth_ring[k] = 0.0f;
    a_smooth_idx = 0;
    reset_sample_buffer();
}

static void transition_to_running()
{
    // Recapture both the encoder zero (phi=0 ↔ current pendulum position)
    // and the stepper origin (motor_pos=0 ↔ current motor position) so the
    // policy sees the same frame conventions it was trained in regardless
    // of how the user reset/positioned the rig.
    reset_pendulum_tracking();
    stepper->setCurrentPosition(0);
    prime_initial_state();
    stepper->enableOutputs();
    state = RUNNING;
}

static void transition_to_waiting()
{
    stepper->forceStop();
    stepper->disableOutputs();
    state = WAITING;
}

// =============================================================================
// CONTROL TICK (called once per CONTROL_PERIOD_US)
// =============================================================================

static void control_tick()
{
    // 1. GET_STATE-equivalent read: newest sample + window-diff velocities.
    float motor_pos, phi, motor_vel, pen_vel;
    read_measured_state(&motor_pos, &phi, &motor_vel, &pen_vel);

    // 2. Hard-limit safety: trip back to WAITING if the motor strayed past
    // the mechanical envelope.
    if (fabs(motor_pos) > MOTOR_HARD_LIMIT_RAD)
    {
        transition_to_waiting();
        return;
    }

    // 3. Push the newest observation frame. prev_action is the action
    // applied during the PREVIOUS tick — same convention as the sim env
    // and run_policy.py (the frame the policy reads always carries the
    // most recently applied action).
    float theta = wrap_pi(phi - (float)PI);
    push_frame(motor_pos, theta, motor_vel, pen_vel, last_action);
    // Stash the velocities the policy consumed, so telemetry captures the
    // full observation and a standalone run can be replayed offline.
    tele_motor_vel = motor_vel;
    tele_pen_vel = pen_vel;

    // 4. Forward pass on the flattened frame stack (oldest -> newest).
    // frames[][] is contiguous, so it IS the obs vector.
    float action;
    policy_forward(&frames[0][0], &action);
    if (action > 1.0f) action = 1.0f;
    else if (action < -1.0f) action = -1.0f;
    last_action = action;

    // 4b. Actuator-side smoothing: the velocity law below tracks the boxcar
    // average of the last ACTION_SMOOTH_WINDOW actions (no-op at window 1).
    // last_action (raw) is what the observation and telemetry carry.
    float action_cmd = action;
    if (ACTION_SMOOTH_WINDOW > 1)
    {
        a_smooth_ring[a_smooth_idx] = action;
        a_smooth_idx = (uint8_t)((a_smooth_idx + 1) % ACTION_SMOOTH_WINDOW);
        float acc = 0.0f;
        for (uint8_t k = 0; k < ACTION_SMOOTH_WINDOW; k++) acc += a_smooth_ring[k];
        action_cmd = acc / (float)ACTION_SMOOTH_WINDOW;
    }

    // 5. Velocity-mode P-law on the commanded integrator (host layer of the
    // tethered stack, verbatim): accel = clip((v_des - v_cmd) * f), zeroed
    // at the rail when pushing outward; v_cmd integrates the applied accel
    // and takes a slow correction from the measured velocity.
    float v_des = action_cmd * MAX_VELOCITY_RAD_S;
    float accel_cmd = (v_des - v_cmd) * CONTROL_FREQUENCY_HZ;
    if (accel_cmd >  MAX_ACCEL_RAD_S2) accel_cmd =  MAX_ACCEL_RAD_S2;
    if (accel_cmd < -MAX_ACCEL_RAD_S2) accel_cmd = -MAX_ACCEL_RAD_S2;
    if (motor_pos >= MOTOR_SAFE_LIMIT_RAD && accel_cmd > 0.0f) accel_cmd = 0.0f;
    else if (motor_pos <= -MOTOR_SAFE_LIMIT_RAD && accel_cmd < 0.0f) accel_cmd = 0.0f;

    v_cmd += accel_cmd * CONTROL_DT_S;
    if (v_cmd >  MAX_VELOCITY_RAD_S) v_cmd =  MAX_VELOCITY_RAD_S;
    if (v_cmd < -MAX_VELOCITY_RAD_S) v_cmd = -MAX_VELOCITY_RAD_S;
    v_cmd += V_CMD_LAMBDA * (motor_vel - v_cmd);

    // 6. Firmware layer (LowLevelServer CMD_SET_ACCEL, verbatim): past the
    // rail, override with a fixed opposing brake — moveByAcceleration(0)
    // would coast at current speed, not stop.
    int32_t accel_steps_s2 = (int32_t)(accel_cmd * STEPS_PER_RAD);
    int32_t cur_steps = stepper->getCurrentPosition();
    int32_t safe_limit_steps = (int32_t)(MOTOR_SAFE_LIMIT_RAD * STEPS_PER_RAD);
    if (cur_steps >= safe_limit_steps)
    {
        accel_steps_s2 = -MOTOR_BRAKE_ACCEL_STEPS_S2;
    }
    else if (cur_steps <= -safe_limit_steps)
    {
        accel_steps_s2 = +MOTOR_BRAKE_ACCEL_STEPS_S2;
    }
    latency_us = (uint16_t)(micros() - used_sample_us);
    if (latency_us > latency_max_us) latency_max_us = latency_us;

    stepper->moveByAcceleration(accel_steps_s2, true);
}

// =============================================================================
// SERIAL
// =============================================================================

static void handle_serial()
{
    if (!Serial.available()) return;
    char cmd = Serial.read();
    while (Serial.available()) Serial.read();
    switch (cmd)
    {
    case 'P': case 'p': print_enabled = !print_enabled; break;
    case 'E': case 'e': if (state == WAITING) transition_to_running(); break;
    case 'D': case 'd': if (state == RUNNING) transition_to_waiting(); break;
    case 'M': case 'm':
        Serial.print(F("[AS5600] magnet="));
        if (as5600.magnetTooWeak()) Serial.println(F("WEAK"));
        else if (as5600.magnetTooStrong()) Serial.println(F("STRONG"));
        else Serial.println(F("OK"));
        break;
    }
}

// Telemetry is drained a few bytes per loop() pass, writing only what the
// UART TX buffer accepts — a full-line Serial.println here would BLOCK once
// the line outgrew the 64-byte TX buffer, and that ~0.6 ms/tick stall
// measurably degrades balance (A/B: 1.000 -> 0.930 on the same policy).
static char tx_line[128];
static uint8_t tx_len = 0;
static uint8_t tx_pos = 0;

static void service_telemetry()
{
    while (tx_pos < tx_len)
    {
        int room = Serial.availableForWrite();
        if (room <= 0) return;
        uint8_t n = (uint8_t)min(room, (int)(tx_len - tx_pos));
        Serial.write((const uint8_t*)&tx_line[tx_pos], n);
        tx_pos += n;
    }
}

static void print_telemetry(unsigned long now_us, unsigned int freq_hz)
{
    if (!print_enabled) return;
    if (tx_pos < tx_len) return;  // previous line still draining: skip this tick
    // CSV: t_us, motor_pos_rad*1000, phi_rad*1000, action*1000, state, freq_hz,
    //      overruns, latency_us, latency_max_us
    // Integer transmission avoids the ~500 µs Serial.print(float) cost.
    // Positions come from the sampler ring's newest entry (same source the
    // policy reads) so host-side analysis sees the policy's own inputs.
    uint8_t n_samples = buf_filled ? SAMPLE_BUFFER_SIZE : buf_head;
    uint8_t newest = (uint8_t)((buf_head + SAMPLE_BUFFER_SIZE - 1) % SAMPLE_BUFFER_SIZE);
    float motor_pos = n_samples ? (float)motor_step_buf[newest] * RAD_PER_STEP
                                : read_motor_pos_rad();
    float phi = n_samples ? pen_rad_buf[newest] : 0.0f;
    char* p = tx_line;
    ltoa((long)now_us, p, 10); p += strlen(p); *p++ = ',';
    ltoa((long)(motor_pos * 1000.0f), p, 10); p += strlen(p); *p++ = ',';
    ltoa((long)(phi * 1000.0f), p, 10); p += strlen(p); *p++ = ',';
    ltoa((long)(last_action * 1000.0f), p, 10); p += strlen(p); *p++ = ',';
    *p++ = (state == RUNNING) ? '1' : '0'; *p++ = ',';
    utoa(freq_hz, p, 10); p += strlen(p); *p++ = ',';
    utoa(loop_overruns, p, 10); p += strlen(p); *p++ = ',';
    utoa(latency_us, p, 10); p += strlen(p); *p++ = ',';
    utoa(latency_max_us, p, 10); p += strlen(p); *p++ = ',';
    // Appended fields (indices 9-11) so older parsers keep working: the
    // velocities the policy consumed this tick, and the P-law integrator.
    ltoa((long)(tele_motor_vel * 1000.0f), p, 10); p += strlen(p); *p++ = ',';
    ltoa((long)(tele_pen_vel * 1000.0f), p, 10); p += strlen(p); *p++ = ',';
    ltoa((long)(v_cmd * 1000.0f), p, 10); p += strlen(p); *p++ = ',';
    utoa(tele_vel_span_us, p, 10); p += strlen(p);
    *p++ = '\n';
    tx_len = (uint8_t)(p - tx_line);
    tx_pos = 0;
    service_telemetry();  // send what fits now; loop() drains the rest
}

// =============================================================================
// LED
// =============================================================================

static void update_led()
{
    static unsigned long last_ms = 0;
    static bool on = false;
    unsigned long now = millis();
    unsigned long period = (state == RUNNING) ? 100 : 500;
    if (now - last_ms >= period)
    {
        last_ms = now;
        on = !on;
        digitalWrite(LED_BUILTIN, on ? HIGH : LOW);
    }
}

// =============================================================================
// SETUP / LOOP
// =============================================================================

void setup()
{
    Serial.begin(SERIAL_BAUD_RATE);
    Wire.begin();
    Wire.setClock(I2C_CLOCK_HZ);
    as5600.begin();

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    // Initialise FastAccelStepper. stepperConnectToPin must be on a Timer1
    // OC pin (pin 9 = OC1A on ATmega328); it returns NULL if the pin is
    // unsupported, which would silently disable stepping — guard with a
    // halt + LED-on so the failure is visible.
    engine.init();
    stepper = engine.stepperConnectToPin(STEP_PIN);
    if (!stepper)
    {
        Serial.println(F("[FATAL] FastAccelStepper failed to claim STEP pin"));
        digitalWrite(LED_BUILTIN, HIGH);
        while (true) {}
    }
    stepper->setDirectionPin(DIR_PIN);
    stepper->setEnablePin(ENABLE_PIN);  // default low_active=true matches DRV8825
    stepper->setAutoEnable(false);      // we manually enable/disable on state changes
    int8_t rc_speed = stepper->setSpeedInUs(MOTOR_MIN_STEP_US);
    int8_t rc_accel = stepper->setAcceleration(MOTOR_BRAKE_ACCEL_STEPS_S2);
    if (rc_speed != 0 || rc_accel != 0)
    {
        Serial.print(F("[FATAL] FastAccelStepper config rejected: speed_rc="));
        Serial.print(rc_speed);
        Serial.print(F(" accel_rc="));
        Serial.println(rc_accel);
        digitalWrite(LED_BUILTIN, HIGH);
        while (true) {}
    }
    // Same forward-planning window as LowLevelServer — shortens command→
    // motion latency to the value the policy's delay DR was centred on.
    stepper->setForwardPlanningTimeInMs(8);
    stepper->disableOutputs();

    while (!as5600.detectMagnet())
    {
        delay(500);
    }

    // Forward-pass self-test: compute the action for a fixed reference obs
    // and print it. Compare against the PyTorch student's prediction for
    // the same obs to confirm PROGMEM access + indexing are correct
    // (values are policy-specific and change every distill).
    {
        float test_act;
        // Hanging-down, still, all 4 frames identical:
        // [motor=0, sin(-π)≈0, cos(-π)=-1, mvel=0, pvel=0, prev_a=0]
        for (uint8_t k = 0; k < OBS_FRAMES; k++)
            fill_frame(frames[k], 0.0f, wrap_pi(-(float)PI), 0.0f, 0.0f, 0.0f);
        policy_forward(&frames[0][0], &test_act);
        Serial.print(F("[boot] policy(hanging) = "));
        Serial.println(test_act, 6);
        // Upright, still:
        for (uint8_t k = 0; k < OBS_FRAMES; k++)
            fill_frame(frames[k], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        policy_forward(&frames[0][0], &test_act);
        Serial.print(F("[boot] policy(upright) = "));
        Serial.println(test_act, 6);
    }

    // 1 s settle delay before engaging — gives the user a moment to verify
    // the pendulum is hanging straight down (LED stays HIGH during the
    // delay). Whatever pose the rig is in at the END of this delay
    // becomes the policy's frame (encoder zero + stepper origin captured
    // by transition_to_running).
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    transition_to_running();

    prev_time_us = micros();
}

void loop()
{
    // FastAccelStepper drives stepping from a Timer1 ISR; between control
    // ticks the loop services the 500 Hz measurement sampler and drains any
    // pending telemetry into the UART buffer without blocking.
    update_sample_buffer();
    service_telemetry();

    unsigned long now_us = micros();
    unsigned long elapsed_us = now_us - prev_time_us;
    if (elapsed_us < CONTROL_PERIOD_US)
    {
        return;
    }

    if (elapsed_us > CONTROL_PERIOD_US * 3UL / 2UL)
    {
        loop_overruns++;
    }

    prev_time_us = now_us;
    loop_count_for_freq++;

    handle_serial();
    update_led();

    if (state == RUNNING)
    {
        control_tick();
    }

    // Telemetry: PER TICK while enabled ('P'), so a host capture of the
    // stream can compute the same honest balance metrics as tethered
    // deploys (see analyze_onboard.py). ~50 bytes/tick at the control rate is
    // negligible at 500 kbaud. Rate/overrun counters still reset each
    // second so freq_hz stays meaningful.
    static unsigned long last_freq_us = 0;
    static unsigned int freq_hz = 0;
    if (now_us - last_freq_us >= 1000000UL)
    {
        freq_hz = (unsigned int)((unsigned long)loop_count_for_freq * 1000000UL
                                 / (now_us - last_freq_us));
        loop_count_for_freq = 0;
        last_freq_us = now_us;
    }
    print_telemetry(now_us, freq_hz);
}
