#include <FastAccelStepper.h>
#include <AS5600.h>
#include <Wire.h>

// Communication speed
const long BAUD_RATE = 2000000;

// Command bytes
#define CMD_READY 0x01
#define CMD_GET_STATE 0x02
#define CMD_SET_ACCEL 0x03   // was CMD_SET_TARGET (position-mode); now angular accel (rad/s²)
#define CMD_ENGAGE_MOTOR 0x04
#define CMD_DISENGAGE_MOTOR 0x05
#define CMD_TARE_PENDULUM 0x06   // re-zero pen_position_rad to current AS5600 reading
#define CMD_SET_TARGET 0x07      // position-mode: commanded motor position (rad) → moveTo
#define CMD_ZERO_MOTOR 0x08      // re-zero the motor step counter to the arm's current position

// Pin assignments. STEP must be on pin 9 (Timer1 OC1A on ATmega328) for
// FastAccelStepper. DIR and ENABLE can be any digital pin.
#define DIR_PIN 2
#define STEP_PIN 9
#define ENABLE_PIN 5

// Microstepping: the ONLY place to change it. Everything below derives from
// STEPS_PER_REVOLUTION. 32 is the recommended ratio on either driver, but the
// pin levels differ — DRV8825 M0=M1=M2=HIGH, TMC2209 MS1=HIGH/MS2=LOW.
// Legacy: 16 = TMC2209 MS1=MS2=HIGH, 8 = DRV8825 M0=M1=HIGH. Must match
// MOTOR_MICROSTEPS in pendulum_env.py and MICROSTEPS in RLControl.ino.
const int MICROSTEPS = 32;
const long STEPS_PER_REVOLUTION = 200L * MICROSTEPS;
const float STEPS_PER_RAD = (float)STEPS_PER_REVOLUTION / (2.0f * (float)PI);
const float RAD_PER_STEP = (2.0f * (float)PI) / (float)STEPS_PER_REVOLUTION;

// Accel-mode envelope. See pendulum_env.py for the corresponding sim
// constants. The velocity cap below corresponds to MAX_VELOCITY_RAD_S
// = 5 rad/s: 5 × (STEPS_PER_REVOLUTION / 2π) steps/s (≈2546 at 3200) ⇒ MOTOR_MIN_STEP_US below.
// NB: this boot-time speed cap applies to BOTH command paths — the accel
// mode it was sized for AND position-mode moveTo() tracking. Position-mode
// commanded slew is max_action_delta × control rate (3.5 rad/s at defaults),
// comfortably inside; raise those knobs past ~5 rad/s and this cap becomes
// the binding limit here while sim and RLControl.ino (~196 rad/s) model none.
const uint32_t MOTOR_MIN_STEP_US =  // ≈ 5 rad/s, derived so it tracks MICROSTEPS
    (uint32_t)(1.0e6f * 2.0f * (float)PI / (5.0f * (float)STEPS_PER_REVOLUTION));

// Ramp acceleration for position-mode moveTo() (CMD_SET_TARGET); matches
// RLControl.ino. Must be re-asserted on every CMD_SET_TARGET: CMD_ENGAGE_MOTOR's
// moveByAcceleration(0) resets the library acceleration to 1 step/s², which
// would make the motor merely creep toward the target. Accel mode ignores this.
const int32_t MOTOR_ACCEL_STEPS_S2 = 50000;

// Position safety limit (matches MOTOR_SAFE_LIMIT_RAD on the Python side,
// ±125°). Past the rail the firmware actively brakes (commands a fixed
// opposing accel) so the motor decelerates even if the host stops
// sending commands (USB hiccup, host hang). Clamping to zero instead
// would just let moveByAcceleration(0, true) coast the motor past the
// rail at peak velocity.
const int32_t MOTOR_SAFE_LIMIT_STEPS = (int32_t)((125.0f * PI / 180.0f) *
                                                  ((float)STEPS_PER_REVOLUTION / (2.0f * PI)));
// Brake authority when past the rail. 150 rad/s² matches the
// pendulum_env.py MAX_ACCEL_RAD_S2 — strong enough to bleed off the
// 5 rad/s vel cap within ~33 ms.
const int32_t MOTOR_BRAKE_ACCEL_STEPS_S2 =
    (int32_t)(150.0f * ((float)STEPS_PER_REVOLUTION / (2.0f * PI)));

// Encoder samples are kept in a ring buffer updated at 500 Hz; GET_STATE
// returns velocity computed as (newest - oldest)/Δt over a window of 5
// samples = 4 inter-sample gaps = 8 ms. Window halved from 10 → 5 on
// 2026-05-20 to cut ~9 ms of observation lag — the largest tunable
// component of the rig's policy-loop lag. Tradeoff: ~√2× noisier
// per-sample velocity estimate. See website/src/content/docs/reference/transport-delay.md.
const uint16_t SAMPLE_PERIOD_US = 2000;
const uint8_t  BUFFER_SIZE      = 16;
const uint8_t  VEL_WINDOW       = 5;
// Discard impossibly-large per-sample wraps as I²C glitches. Real
// pendulum tops out at ~50 rad/s, so in one 2 ms sample period the
// raw AS5600 reading can change by at most 50·0.002·4096/(2π) ≈ 65 LSB.
// Anything more is almost certainly a corrupted I²C transaction —
// keep the previous reading rather than letting it pollute the
// accumulator with a spurious ±2π wrap.
const long PEN_RAW_MAX_DELTA_LSB = 500;

static int32_t motor_step_buf[BUFFER_SIZE];   // raw stepper position (steps)
static float   pen_rad_buf[BUFFER_SIZE];      // accumulated pendulum angle (rad)
static uint32_t time_us_buf[BUFFER_SIZE];     // sample timestamps (µs)
static uint8_t  buf_head = 0;                 // next write index
static bool     buf_filled = false;           // becomes true after first full lap
static uint32_t last_sample_us = 0;

// Continuously-tracked pendulum angle (independent of GET_STATE cadence so
// rapid wraparounds aren't missed).
static long    pen_raw_prev   = -1;           // -1 = first read sentinel
static float   pen_position_rad = 0.0f;

// State variables
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;
AS5600 as5600;

// `motor_engaged` is only touched from loop() / handleCommand() — no ISR
// access — so `volatile` would only mislead future readers. Plain bool.
bool motor_engaged = false;

// Function prototypes
void handleCommand();
void sendState();
void sampleState();
void computeVelocities(float* motor_vel_rad_s, float* pen_vel_rad_s);

void setup()
{
    Serial.begin(BAUD_RATE);
    Wire.begin();
    Wire.setClock(400000);   // I²C fast mode for short transaction times.
    as5600.begin();

    engine.init();
    stepper = engine.stepperConnectToPin(STEP_PIN);
    if (!stepper)
    {
        while (true) { /* halt: STEP_PIN is not Timer1 OC1A/OC1B */ }
    }
    stepper->setDirectionPin(DIR_PIN);
    stepper->setEnablePin(ENABLE_PIN);
    stepper->setAutoEnable(false);

    int8_t rc_speed = stepper->setSpeedInUs(MOTOR_MIN_STEP_US);
    if (rc_speed != 0)
    {
        while (true) {}
    }
    // Acceleration is set per tick in CMD_SET_TARGET, not here (see that
    // constant's note).
    // Shrink the forward-planning window from the library's 20 ms default
    // to its documented minimum (8 ms = two cyclic-task periods). Default
    // adds ~20 ms of lag between a new accel command and any change in
    // emitted step intervals, because step pulses already queued can't be
    // retroactively edited. Measured (accel_step_probe.py) total firmware
    // lag at default: ~37 ms → expected ~15 ms after this change. At our
    // top step rate (≈1270 steps/s) 8 ms still queues ~10 pulses, well
    // above the "stepper starves and stalls at full speed" failure mode
    // the library warns about.
    stepper->setForwardPlanningTimeInMs(8);
    stepper->disableOutputs();

    while (!Serial) { ; }
    while (!as5600.detectMagnet()) { delay(500); }

    last_sample_us = micros();
}

void loop()
{
    uint32_t now_us = micros();
    if ((uint32_t)(now_us - last_sample_us) >= SAMPLE_PERIOD_US)
    {
        last_sample_us = now_us;
        sampleState();
    }
    if (Serial.available() > 0)
    {
        handleCommand();
    }
}

/*
 * Append a (time, motor_position, pendulum_position) sample to the ring
 * buffer and update the pendulum wraparound accumulator.
 */
void sampleState()
{
    int32_t motor_step = stepper->getCurrentPosition();
    long raw = as5600.rawAngle();

    // Pendulum wraparound tracking (AS5600 is 12-bit; wrap threshold ±2048).
    if (pen_raw_prev < 0)
    {
        pen_raw_prev = raw;
    }
    else
    {
        long delta = raw - pen_raw_prev;
        if (delta >  2048) delta -= 4096;
        if (delta < -2048) delta += 4096;
        // Reject implausibly-large single-step deltas. One bad I²C read
        // sample that we incorrectly classify as a wrap would add ±2π to
        // pen_position_rad and contaminate every subsequent observation
        // (the accumulator never resets). Skip the update on glitches —
        // velocity for this tick will be slightly stale but stays sane.
        if (delta > PEN_RAW_MAX_DELTA_LSB || delta < -PEN_RAW_MAX_DELTA_LSB)
        {
            // Don't update pen_raw_prev either: next good read will
            // compute the delta against the last trustworthy reading.
        }
        else
        {
            pen_position_rad += (float)delta * ((2.0f * PI) / 4096.0f);
            pen_raw_prev = raw;
        }
    }

    motor_step_buf[buf_head] = motor_step;
    pen_rad_buf[buf_head]    = pen_position_rad;
    time_us_buf[buf_head]    = last_sample_us;
    buf_head = (buf_head + 1) % BUFFER_SIZE;
    if (buf_head == 0) buf_filled = true;
}

/*
 * Compute velocity over the most recent VEL_WINDOW samples as
 * (newest - oldest)/Δt. Returns 0 until the buffer holds enough samples.
 */
void computeVelocities(float* motor_vel_rad_s, float* pen_vel_rad_s)
{
    uint8_t n_samples = buf_filled ? BUFFER_SIZE : buf_head;
    if (n_samples < VEL_WINDOW)
    {
        *motor_vel_rad_s = 0.0f;
        *pen_vel_rad_s   = 0.0f;
        return;
    }

    uint8_t newest = (uint8_t)((buf_head + BUFFER_SIZE - 1)            % BUFFER_SIZE);
    uint8_t oldest = (uint8_t)((buf_head + BUFFER_SIZE - VEL_WINDOW)   % BUFFER_SIZE);

    uint32_t t_new = time_us_buf[newest];
    uint32_t t_old = time_us_buf[oldest];
    float dt_s = (float)((uint32_t)(t_new - t_old)) * 1e-6f;
    if (dt_s <= 0.0f)
    {
        *motor_vel_rad_s = 0.0f;
        *pen_vel_rad_s   = 0.0f;
        return;
    }

    int32_t motor_step_delta = motor_step_buf[newest] - motor_step_buf[oldest];
    *motor_vel_rad_s = ((float)motor_step_delta * ((2.0f * PI) / (float)STEPS_PER_REVOLUTION)) / dt_s;

    float pen_delta = pen_rad_buf[newest] - pen_rad_buf[oldest];
    *pen_vel_rad_s = pen_delta / dt_s;
}

/*
 * Handle incoming commands.
 */
void handleCommand()
{
    uint8_t command = Serial.read();

    switch (command)
    {
    case CMD_READY:
        Serial.write(CMD_READY);
        break;

    case CMD_GET_STATE:
        sendState();
        break;

    case CMD_SET_ACCEL:
        {
            // Read 4 bytes (size of float) with a short timeout. If a byte
            // gets dropped — UART overrun under stepper EMI, or a flipped
            // command byte that desyncs the parser — readBytes returns
            // short and we bail. The next command re-syncs.
            Serial.setTimeout(5);
            float accel_rad_s2;
            size_t n = Serial.readBytes((char *)&accel_rad_s2, sizeof(float));
            Serial.setTimeout(1000);  // restore Stream default
            if (n != sizeof(float)) break;

            if (!motor_engaged) break;

            // Convert rad/s² to steps/s² (STEPS_PER_REVOLUTION microsteps per revolution).
            // moveByAcceleration takes int32_t.
            int32_t accel_steps_s2 =
                (int32_t)(accel_rad_s2 * ((float)STEPS_PER_REVOLUTION / (2.0f * PI)));

            // Position-limit safety: past the rail, ignore the host's
            // command and actively brake instead. Just zeroing accel here
            // is dangerous — with allow_reverse=true, an accel of 0 means
            // "hold current speed", so a motor at +5 rad/s heading
            // outbound would coast indefinitely if the host went quiet.
            // Brake with a fixed opposing accel so we decelerate
            // regardless of host liveness.
            int32_t cur_pos = stepper->getCurrentPosition();
            if (cur_pos >= MOTOR_SAFE_LIMIT_STEPS)
            {
                accel_steps_s2 = -MOTOR_BRAKE_ACCEL_STEPS_S2;
            }
            else if (cur_pos <= -MOTOR_SAFE_LIMIT_STEPS)
            {
                accel_steps_s2 = +MOTOR_BRAKE_ACCEL_STEPS_S2;
            }

            // allow_reverse=true makes the library decelerate smoothly through
            // zero when the sign of accel opposes the current velocity — no
            // state machine needed on our side.
            stepper->moveByAcceleration(accel_steps_s2, true);
        }
        break;

    case CMD_SET_TARGET:
        {
            // Position-mode command: absolute motor target in radians. The host
            // integrates the policy's per-tick deltas and sends the absolute
            // target each tick; the stepper ramps toward it (moveTo). Coexists
            // with CMD_SET_ACCEL. Sign matches CMD_SET_ACCEL: target arrives
            // un-flipped (positions flip only on GET_STATE output).
            Serial.setTimeout(5);
            float target_rad;
            size_t n = Serial.readBytes((char *)&target_rad, sizeof(float));
            Serial.setTimeout(1000);  // restore Stream default
            if (n != sizeof(float)) break;

            if (!motor_engaged) break;

            // Clamp to the safety rail so the policy can never command a
            // target past the mechanical hard stop.
            int32_t target_steps = (int32_t)lroundf(target_rad * STEPS_PER_RAD);
            if (target_steps >  MOTOR_SAFE_LIMIT_STEPS) target_steps =  MOTOR_SAFE_LIMIT_STEPS;
            if (target_steps < -MOTOR_SAFE_LIMIT_STEPS) target_steps = -MOTOR_SAFE_LIMIT_STEPS;

            // Re-assert the ramp acceleration (CMD_ENGAGE_MOTOR left it at
            // 1 step/s²; see the constant's note).
            stepper->setAcceleration(MOTOR_ACCEL_STEPS_S2);
            stepper->moveTo(target_steps);
        }
        break;

    case CMD_ENGAGE_MOTOR:
        motor_engaged = true;
        stepper->enableOutputs();
        // Start in zero-accel state. moveByAcceleration(0, true) means
        // "hold current speed", and since we've just enabled the driver
        // the stepper is at rest — so this leaves it at rest until the
        // host sends its first CMD_SET_ACCEL.
        // Note: ENGAGE does not re-zero `getCurrentPosition()` or
        // `pen_position_rad`. Position counters persist across
        // engage/disengage cycles (and across episodes) by design — the
        // host is responsible for tracking the frame.
        stepper->moveByAcceleration(0, true);
        break;

    case CMD_DISENGAGE_MOTOR:
        motor_engaged = false;
        // forceStop() drains the Timer1 step queue immediately; without this,
        // queued steps would continue advancing the firmware position counter
        // even though the driver's enable pin is HIGH.
        stepper->forceStop();
        stepper->disableOutputs();
        break;

    case CMD_TARE_PENDULUM:
        // Re-zero pen_position_rad to the current AS5600 reading. Used by
        // real_env.reset() after the pendulum has settled at rest so that
        // each fine-tune episode samples a fresh bias from the rig's
        // static-friction-bounded rest distribution (±1.9°). Without this,
        // all fine-tune episodes share the firmware-boot bias and the
        // policy overfits to that single calibration offset.
        //
        // Shift pen_position_rad AND every entry in pen_rad_buf by the
        // current pen_position_rad. The buffer's relative deltas (used
        // by computeVelocities) are preserved, so velocity calculation
        // continues uninterrupted across the tare. Disable interrupts so
        // sampleState() doesn't run mid-update with a partially-shifted
        // buffer.
        noInterrupts();
        {
            float offset = pen_position_rad;
            for (uint8_t i = 0; i < BUFFER_SIZE; i++)
            {
                pen_rad_buf[i] -= offset;
            }
            pen_position_rad = 0.0f;
        }
        interrupts();
        Serial.write(CMD_TARE_PENDULUM);  // ack
        break;

    case CMD_ZERO_MOTOR:
        // Re-zero the step counter to the arm's current position, for when the
        // operator has moved the arm by hand between episodes. Shift the ring
        // buffer by the same offset (as CMD_TARE_PENDULUM does) so velocity
        // stays continuous; interrupts off for a consistent buffer.
        noInterrupts();
        {
            int32_t offset = stepper->getCurrentPosition();
            for (uint8_t i = 0; i < BUFFER_SIZE; i++)
            {
                motor_step_buf[i] -= offset;
            }
            stepper->setCurrentPosition(0);
        }
        interrupts();
        Serial.write(CMD_ZERO_MOTOR);  // ack
        break;

    default:
        break;
    }
}

/*
 * Send the current state of the system:
 * - Current time in microseconds   (4 bytes, uint32)
 * - Stepper motor position in rad  (4 bytes, float)
 * - Pendulum position in rad       (4 bytes, float)
 * - Stepper motor velocity in rad/s (4 bytes, float)
 * - Pendulum velocity in rad/s     (4 bytes, float)
 */
void sendState()
{
    // Take the most recent buffer entry as the instantaneous position,
    // and compute velocity from the (newest - oldest)/Δt regression
    // window. The timestamp is the sample time of `newest` — not a
    // fresh micros() — so the host gets a self-consistent (t, pos, vel)
    // tuple it can time-align without inheriting up-to-one-sample of
    // bias.
    uint8_t newest = (uint8_t)((buf_head + BUFFER_SIZE - 1) % BUFFER_SIZE);
    uint32_t current_time = time_us_buf[newest];
    float motor_position_radians = (float)motor_step_buf[newest] * RAD_PER_STEP;
    float pendulum_position_radians = pen_rad_buf[newest];

    float motor_velocity_rad_s, pendulum_velocity_rad_s;
    computeVelocities(&motor_velocity_rad_s, &pendulum_velocity_rad_s);

    // Values go out exactly as the firmware measures them (step counter,
    // AS5600 accumulator) — one frame everywhere, no flips on either side.

    // Pack and send the data
    Serial.write((byte *)&current_time, sizeof(current_time));
    Serial.write((byte *)&motor_position_radians, sizeof(motor_position_radians));
    Serial.write((byte *)&pendulum_position_radians, sizeof(pendulum_position_radians));
    Serial.write((byte *)&motor_velocity_rad_s, sizeof(motor_velocity_rad_s));
    Serial.write((byte *)&pendulum_velocity_rad_s, sizeof(pendulum_velocity_rad_s));
}
