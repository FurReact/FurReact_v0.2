#![no_std]
#![no_main]

use core::fmt::Write as _;
use embassy_time::{Duration, Instant, with_timeout};
use embedded_io_async::{Read, Write as IoWrite};
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    gpio::DriveMode,
    interrupt::software::SoftwareInterruptControl,
    ledc::{
        LSGlobalClkSource, Ledc, LowSpeed,
        channel::{self, ChannelHW, ChannelIFace},
        timer::{self, LSClockSource, TimerIFace},
    },
    time::Rate,
    timer::timg::TimerGroup,
    usb_serial_jtag::UsbSerialJtag,
};
use heapless::String;

esp_bootloader_esp_idf::esp_app_desc!();

// ─────────────── TUNABLES ───────────────
// Input signal: parse_line yields four values in a fixed canonical
// order — left_x, left_y, right_x, right_y. Nominal 0–1024, physical
// 256–768, center ≈ 512.
const INPUT_MIN:    i32 = 256;
const INPUT_MAX:    i32 = 768;
const INPUT_CENTER: i32 = 512;

// Servo electrical limits (shared by all four channels).
const SERVO_MIN_US: u32 = 500;
const SERVO_MAX_US: u32 = 2400;
const PERIOD_US:    u32 = 20_000;
const LEDC_BITS:    u8  = 14;

/// One of the four values parse_line produces, in its canonical order.
#[derive(Copy, Clone)]
enum InputAxis {
    LeftX  = 0,
    LeftY  = 1,
    RightX = 2,
    RightY = 3,
}

/// How one physical servo channel turns an input axis into an angle.
struct ServoRoute {
    /// Set false to leave this channel's PWM idle (duty 0, no pulses) in
    /// every mode — handy for bringing up one servo at a time.
    enabled: bool,
    /// Which parsed input drives this servo. Re-point this to rewire
    /// eyes↔ears or pan↔tilt however the harness was actually soldered
    /// (replaces the old SWAP_EYES).
    source: InputAxis,
    /// Flip travel direction (replaces the old INVERT_* flags).
    invert: bool,
    /// Rest angle, 0–180. 90 = servo mechanical center.
    neutral_deg: i32,
    /// Max swing from neutral at full input deflection, in degrees.
    deflection_deg: i32,
}

// ── SERVO ROUTING — the EE-wiring knob ──
// One entry per physical output channel, in hardware order:
//   [0] ch0 / GPIO1 / D0      [2] ch2 / GPIO3 / D2
//   [1] ch1 / GPIO2 / D1      [3] ch3 / GPIO4 / D3
// Each output picks its input axis (`source`), its direction
// (`invert`), and its own neutral/deflection so individual servos can
// be trimmed to the mechanics they're bolted to.
const ROUTES: [ServoRoute; 4] = [
    ServoRoute { enabled: true, source: InputAxis::LeftX,  invert: false, neutral_deg: 90, deflection_deg: 30 },
    ServoRoute { enabled: true, source: InputAxis::LeftY,  invert: false, neutral_deg: 90, deflection_deg: 30 },
    ServoRoute { enabled: true, source: InputAxis::RightX, invert: false, neutral_deg: 90, deflection_deg: 30 },
    ServoRoute { enabled: true, source: InputAxis::RightY, invert: false, neutral_deg: 90, deflection_deg: 30 },
];

// State is determined purely by `stale_ms` — time since the last valid
// gaze line. "Never received any line" is treated as stale-for-uptime,
// so a fresh boot naturally lands in Sweep after SWEEP_AFTER_MS.
//
//   stale_ms < FAILSAFE_TIMEOUT_MS         → Follow
//                  (active gaze; ears track eyes)
//   FAILSAFE_TIMEOUT_MS ≤ stale_ms < SWEEP_AFTER_MS
//                                          → Center
//                  (brief input gap — hold neutral so we don't twitch
//                  into Sweep every time the forwarder hiccups for
//                  a frame)
//   stale_ms ≥ SWEEP_AFTER_MS              → Sweep
//                  (standalone self-test: slowly walks each servo
//                  across its full active range — visible whenever
//                  we've been disconnected from gaze data for a few
//                  seconds, including running off a charger / battery
//                  with no host attached at all)
//
// PWM is always being driven in every state — Center isn't a "go limp"
// mode, it just holds neutral. To silence the servos, cut their power.
const FAILSAFE_TIMEOUT_MS: u64 = 500;
const SWEEP_AFTER_MS:      u64 = 3_000;

// Sweep tunables — slow, asymmetric triangle waves on each axis. Periods
// are coprime-ish so the 4D trajectory doesn't repeat in ~minutes.
const SWEEP_PERIODS_MS: [u64; 4] = [3_700, 5_100, 4_300, 6_100];

// ────────────────────────────────────────

const MAX_DUTY: u32 = 1 << LEDC_BITS;

fn pulse_us_from_input(v: i32, route: &ServoRoute) -> u32 {
    let v = v.clamp(INPUT_MIN, INPUT_MAX);
    let mut off = v - INPUT_CENTER;
    if route.invert {
        off = -off;
    }
    let half = ((INPUT_MAX - INPUT_MIN) / 2).max(1);
    let deflection = (off * route.deflection_deg) / half;
    let angle = (route.neutral_deg + deflection).clamp(0, 180) as u32;
    SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * angle / 180
}

fn duty_for_pulse_us(pulse_us: u32) -> u32 {
    let d = pulse_us as u64 * MAX_DUTY as u64 / PERIOD_US as u64;
    (d as u32).min(MAX_DUTY - 1)
}

/// Triangle wave from INPUT_MIN..=INPUT_MAX over `period_ms`.
fn sweep_axis(t_ms: u64, period_ms: u64) -> i32 {
    let p = t_ms % period_ms;
    let half = period_ms / 2;
    // unit ramps 0..1000..0 over the period; integer math, no libm.
    let unit_ms = if p < half {
        (p as i64) * 1000 / (half as i64)
    } else {
        1000 - ((p - half) as i64) * 1000 / (half as i64)
    };
    let span = (INPUT_MAX - INPUT_MIN) as i64;
    INPUT_MIN + (span * unit_ms / 1000) as i32
}

fn sweep_targets(uptime_ms: u64) -> [i32; 4] {
    [
        sweep_axis(uptime_ms, SWEEP_PERIODS_MS[0]),
        sweep_axis(uptime_ms, SWEEP_PERIODS_MS[1]),
        sweep_axis(uptime_ms, SWEEP_PERIODS_MS[2]),
        sweep_axis(uptime_ms, SWEEP_PERIODS_MS[3]),
    ]
}

fn parse_line(buf: &[u8]) -> Option<(i32, i32, i32, i32)> {
    let s = core::str::from_utf8(buf).ok()?;
    let s = s.trim_end_matches('\r');
    let mut it = s.split(',');
    let a = it.next()?.trim().parse().ok()?;
    let b = it.next()?.trim().parse().ok()?;
    let c = it.next()?.trim().parse().ok()?;
    let d = it.next()?.trim().parse().ok()?;
    if it.next().is_some() {
        return None;
    }
    Some((a, b, c, d))
}

// USB TX is observability only. Bound every write+flush by this timeout:
// the ESP32-S3 USB-Serial-JTAG flush() blocks until the host drains the
// FIFO, and on battery / with no open tty no host ever does — an
// unbounded flush would wedge us before PWM even starts. Errors and
// timeouts are deliberately ignored.
const TX_TIMEOUT: Duration = Duration::from_millis(20);

async fn emit<W: IoWrite>(tx: &mut W, bytes: &[u8]) {
    let _ = with_timeout(TX_TIMEOUT, tx.write_all(bytes)).await;
    let _ = with_timeout(TX_TIMEOUT, tx.flush()).await;
}

#[esp_rtos::main]
async fn main(_spawner: embassy_executor::Spawner) {
    let p = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));

    let sw_int = SoftwareInterruptControl::new(p.SW_INTERRUPT);
    let timg0 = TimerGroup::new(p.TIMG0);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    let (mut rx, mut tx) = UsbSerialJtag::new(p.USB_DEVICE).into_async().split();

    let mut ledc = Ledc::new(p.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let mut lstimer0 = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    lstimer0
        .configure(timer::config::Config {
            duty:         timer::config::Duty::Duty14Bit,
            clock_source: LSClockSource::APBClk,
            frequency:    Rate::from_hz(50),
        })
        .unwrap();

    let mut ch0 = ledc.channel(channel::Number::Channel0, p.GPIO1); // D0 — left pan
    let mut ch1 = ledc.channel(channel::Number::Channel1, p.GPIO2); // D1 — left tilt
    let mut ch2 = ledc.channel(channel::Number::Channel2, p.GPIO3); // D2 — right pan
    let mut ch3 = ledc.channel(channel::Number::Channel3, p.GPIO4); // D3 — right tilt
    for ch in [&mut ch0, &mut ch1, &mut ch2, &mut ch3] {
        ch.configure(channel::config::Config {
            timer:      &lstimer0,
            duty_pct:   0,
            drive_mode: DriveMode::PushPull,
        })
        .unwrap();
    }

    emit(&mut tx, b"BOOT solcatears-fw v0.1\n").await;

    let mut buf = [0u8; 32];
    let mut idx = 0usize;
    let mut rbuf = [0u8; 64];

    let boot = Instant::now();
    let mut last_valid_line: Option<Instant> = None;
    let mut last_targets: [i32; 4] = [INPUT_CENTER; 4];
    let mut current_mode = Mode::Center;
    // Avoid re-issuing identical PWM writes (Sweep changes every tick;
    // Follow/Center are usually steady). Sentinel = first apply forced.
    let mut last_applied: [i32; 4] = [i32::MIN; 4];
    // Sweep-mode heartbeat (seconds since boot at last log).
    let mut last_sweep_log_s: u64 = u64::MAX;

    loop {
        // Poll for USB input with a short timeout, so we can also tick
        // the state machine at ~50 Hz when no data is flowing.
        let res = with_timeout(Duration::from_millis(20), rx.read(&mut rbuf)).await;

        if let Ok(Ok(n)) = res {
            for &c in &rbuf[..n] {
                if c == b'\n' {
                    if let Some((lx, ly, rxv, ry)) = parse_line(&buf[..idx]) {
                        // Stored in canonical input order; ROUTES decides
                        // which output channel each one ends up driving.
                        last_targets = [lx, ly, rxv, ry];
                        last_valid_line = Some(Instant::now());

                        // Echo for host smoke test (firmware/scripts/echo_test.py).
                        let mut msg: String<128> = String::new();
                        let _ = writeln!(&mut msg, "OK {},{},{},{}", lx, ly, rxv, ry);
                        emit(&mut tx, msg.as_bytes()).await;
                    } else {
                        emit(&mut tx, b"BAD\n").await;
                    }
                    idx = 0;
                } else if c != b'\r' {
                    if idx < buf.len() {
                        buf[idx] = c;
                        idx += 1;
                    } else {
                        idx = 0; // overrun: drop line
                    }
                }
            }
        }

        // ── decide state ─────────────────────────────────────────────
        let now = Instant::now();
        let uptime_ms = (now - boot).as_millis();
        // If we've never received a valid line, treat the staleness as
        // "stale for the whole uptime" — so the boot path naturally goes
        // Center → Sweep at the same thresholds as a mid-run disconnect.
        let stale_ms = match last_valid_line {
            Some(t) => (now - t).as_millis(),
            None    => uptime_ms,
        };

        let new_mode = if stale_ms < FAILSAFE_TIMEOUT_MS {
            Mode::Follow
        } else if stale_ms < SWEEP_AFTER_MS {
            Mode::Center
        } else {
            Mode::Sweep
        };

        // Announce mode transitions for debugging via the host smoke test.
        if new_mode != current_mode {
            let s: &[u8] = match new_mode {
                Mode::Follow => b"MODE follow\n",
                Mode::Center => b"MODE center\n",
                Mode::Sweep  => b"MODE sweep\n",
            };
            emit(&mut tx, s).await;
            current_mode = new_mode;
        }

        // ── apply ────────────────────────────────────────────────────
        // Resolve a per-OUTPUT input value. Follow/Center work in input
        // space and route each output to its `source`; Sweep walks each
        // output through its own range directly (a per-servo self-test).
        let targets: [i32; 4] = match new_mode {
            Mode::Follow => [
                last_targets[ROUTES[0].source as usize],
                last_targets[ROUTES[1].source as usize],
                last_targets[ROUTES[2].source as usize],
                last_targets[ROUTES[3].source as usize],
            ],
            Mode::Center => [INPUT_CENTER; 4],
            Mode::Sweep  => sweep_targets(uptime_ms),
        };

        if targets != last_applied {
            // Disabled channels get duty 0 (no pulses) in every mode.
            let duty = |o: usize| -> u32 {
                if ROUTES[o].enabled {
                    duty_for_pulse_us(pulse_us_from_input(targets[o], &ROUTES[o]))
                } else {
                    0
                }
            };
            let _ = ch0.set_duty_hw(duty(0));
            let _ = ch1.set_duty_hw(duty(1));
            let _ = ch2.set_duty_hw(duty(2));
            let _ = ch3.set_duty_hw(duty(3));
            last_applied = targets;
        }

        // Sweep-mode heartbeat: emit once per second so we can verify
        // from listen-only that the loop is iterating and PWM targets
        // are advancing. (Follow has its own per-line OK echoes.)
        if new_mode == Mode::Sweep {
            let s = uptime_ms / 1000;
            if s != last_sweep_log_s {
                last_sweep_log_s = s;
                let mut msg: String<128> = String::new();
                let _ = writeln!(
                    &mut msg,
                    "SWEEP t={}s t={},{},{},{}",
                    s, targets[0], targets[1], targets[2], targets[3]
                );
                emit(&mut tx, msg.as_bytes()).await;
            }
        }
    }
}

#[derive(Copy, Clone, PartialEq, Eq)]
enum Mode {
    Follow,
    Center,
    Sweep,
}
