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
const INPUT_MIN:    i32 = 256;
const INPUT_MAX:    i32 = 768;
const INPUT_CENTER: i32 = 512;

const EAR_DEFLECTION_DEG: i32 = 30;
const SERVO_NEUTRAL_DEG:  i32 = 90;

const SERVO_MIN_US: u32 = 500;
const SERVO_MAX_US: u32 = 2400;
const PERIOD_US:    u32 = 20_000;
const LEDC_BITS:    u8  = 14;

// States, layered from "no contact" to "rich contact":
//
//   Sweep   — never received a single USB byte since boot AND boot was
//             more than SWEEP_AFTER_MS ago. Standalone self-test: slowly
//             walks each servo across its full active range so you can
//             eyeball that all four are alive when running off a charger
//             with no PC attached.
//   Follow  — got a valid gaze line within FAILSAFE_TIMEOUT_MS. Normal
//             operating mode.
//   Center  — was Following but the data went stale (forwarder up but
//             no gaze, or forwarder just died). Holds all four servos
//             at SERVO_NEUTRAL_DEG (input == INPUT_CENTER) indefinitely;
//             we don't auto-disable PWM. To stop the servos drawing
//             current, physically power them off.
const FAILSAFE_TIMEOUT_MS: u64 = 500;
const SWEEP_AFTER_MS:      u64 = 3_000;

// Sweep tunables — slow, asymmetric triangle waves on each axis. Periods
// are coprime-ish so the 4D trajectory doesn't repeat in ~minutes.
const SWEEP_PERIODS_MS: [u64; 4] = [3_700, 5_100, 4_300, 6_100];

const INVERT_LEFT_X:  bool = false;
const INVERT_LEFT_Y:  bool = false;
const INVERT_RIGHT_X: bool = false;
const INVERT_RIGHT_Y: bool = false;
const SWAP_EYES:      bool = false;
// ────────────────────────────────────────

const MAX_DUTY: u32 = 1 << LEDC_BITS;

fn pulse_us_from_input(v: i32, invert: bool) -> u32 {
    let v = v.clamp(INPUT_MIN, INPUT_MAX);
    let mut off = v - INPUT_CENTER;
    if invert {
        off = -off;
    }
    let half = ((INPUT_MAX - INPUT_MIN) / 2).max(1);
    let deflection = (off * EAR_DEFLECTION_DEG) / half;
    let angle = (SERVO_NEUTRAL_DEG + deflection).clamp(0, 180) as u32;
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

    let _ = IoWrite::write_all(&mut tx, b"BOOT solcatears-fw v0.1\n").await;
    // ESP32-S3 USB-Serial-JTAG only ships TX bytes to the host when its
    // 64-byte FIFO fills OR we explicitly set WR_DONE via flush(). In
    // Follow mode the firehose of OK echoes keeps the FIFO churning, but
    // standalone (Sweep/Center) writes are tens of bytes — they'd sit in
    // the FIFO indefinitely without an explicit flush.
    let _ = IoWrite::flush(&mut tx).await;

    let mut buf = [0u8; 32];
    let mut idx = 0usize;
    let mut rbuf = [0u8; 64];

    let boot = Instant::now();
    let mut ever_received_byte = false;
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
            if n > 0 {
                ever_received_byte = true;
            }
            for i in 0..n {
                let c = rbuf[i];
                if c == b'\n' {
                    if let Some((mut lx, mut ly, mut rxv, mut ry)) = parse_line(&buf[..idx]) {
                        if SWAP_EYES {
                            core::mem::swap(&mut lx, &mut rxv);
                            core::mem::swap(&mut ly, &mut ry);
                        }
                        last_targets = [lx, ly, rxv, ry];
                        last_valid_line = Some(Instant::now());

                        // Echo for host smoke test (firmware/scripts/echo_test.py).
                        let mut msg: String<128> = String::new();
                        let _ = writeln!(&mut msg, "OK {},{},{},{}", lx, ly, rxv, ry);
                        let _ = IoWrite::write_all(&mut tx, msg.as_bytes()).await;
                        let _ = IoWrite::flush(&mut tx).await;
                    } else {
                        let _ = IoWrite::write_all(&mut tx, b"BAD\n").await;
                        let _ = IoWrite::flush(&mut tx).await;
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
        let stale_ms = last_valid_line
            .map(|t| (now - t).as_millis())
            .unwrap_or(u64::MAX);

        let new_mode = if !ever_received_byte {
            if uptime_ms < SWEEP_AFTER_MS { Mode::Center } else { Mode::Sweep }
        } else if stale_ms < FAILSAFE_TIMEOUT_MS {
            Mode::Follow
        } else {
            Mode::Center
        };

        // Announce mode transitions for debugging via the host smoke test.
        if new_mode != current_mode {
            let s: &[u8] = match new_mode {
                Mode::Follow => b"MODE follow\n",
                Mode::Center => b"MODE center\n",
                Mode::Sweep  => b"MODE sweep\n",
            };
            let _ = IoWrite::write_all(&mut tx, s).await;
            let _ = IoWrite::flush(&mut tx).await;
            current_mode = new_mode;
        }

        // ── apply ────────────────────────────────────────────────────
        let targets = match new_mode {
            Mode::Follow => last_targets,
            Mode::Center => [INPUT_CENTER; 4],
            Mode::Sweep  => sweep_targets(uptime_ms),
        };

        if targets != last_applied {
            let us = [
                pulse_us_from_input(targets[0], INVERT_LEFT_X),
                pulse_us_from_input(targets[1], INVERT_LEFT_Y),
                pulse_us_from_input(targets[2], INVERT_RIGHT_X),
                pulse_us_from_input(targets[3], INVERT_RIGHT_Y),
            ];
            let _ = ch0.set_duty_hw(duty_for_pulse_us(us[0]));
            let _ = ch1.set_duty_hw(duty_for_pulse_us(us[1]));
            let _ = ch2.set_duty_hw(duty_for_pulse_us(us[2]));
            let _ = ch3.set_duty_hw(duty_for_pulse_us(us[3]));
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
                let _ = IoWrite::write_all(&mut tx, msg.as_bytes()).await;
                let _ = IoWrite::flush(&mut tx).await;
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
