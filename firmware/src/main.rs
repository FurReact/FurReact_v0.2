#![no_std]
#![no_main]

use core::fmt::Write as _;
use embassy_time::{Duration, with_timeout};
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

const FAILSAFE_TIMEOUT_MS: u64 = 500;

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

    let mut buf = [0u8; 32];
    let mut idx = 0usize;
    let mut limp = true;
    let mut rbuf = [0u8; 64];

    loop {
        let res = with_timeout(
            Duration::from_millis(FAILSAFE_TIMEOUT_MS),
            rx.read(&mut rbuf),
        )
        .await;

        match res {
            Ok(Ok(n)) => {
                for i in 0..n {
                    let c = rbuf[i];
                    if c == b'\n' {
                        if let Some((mut lx, mut ly, mut rxv, mut ry)) = parse_line(&buf[..idx]) {
                            if SWAP_EYES {
                                core::mem::swap(&mut lx, &mut rxv);
                                core::mem::swap(&mut ly, &mut ry);
                            }
                            let us_lx = pulse_us_from_input(lx,  INVERT_LEFT_X);
                            let us_ly = pulse_us_from_input(ly,  INVERT_LEFT_Y);
                            let us_rx = pulse_us_from_input(rxv, INVERT_RIGHT_X);
                            let us_ry = pulse_us_from_input(ry,  INVERT_RIGHT_Y);
                            let _ = ch0.set_duty_hw(duty_for_pulse_us(us_lx));
                            let _ = ch1.set_duty_hw(duty_for_pulse_us(us_ly));
                            let _ = ch2.set_duty_hw(duty_for_pulse_us(us_rx));
                            let _ = ch3.set_duty_hw(duty_for_pulse_us(us_ry));
                            limp = false;

                            let mut msg: String<128> = String::new();
                            let _ = writeln!(
                                &mut msg,
                                "OK {},{},{},{} us={},{},{},{}",
                                lx, ly, rxv, ry, us_lx, us_ly, us_rx, us_ry
                            );
                            let _ = IoWrite::write_all(&mut tx, msg.as_bytes()).await;
                        } else {
                            let _ = IoWrite::write_all(&mut tx, b"BAD\n").await;
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
            Ok(Err(_)) | Err(_) => {
                if !limp {
                    let _ = ch0.set_duty_hw(0);
                    let _ = ch1.set_duty_hw(0);
                    let _ = ch2.set_duty_hw(0);
                    let _ = ch3.set_duty_hw(0);
                    limp = true;
                    let _ = IoWrite::write_all(&mut tx, b"LIMP\n").await;
                }
                idx = 0;
            }
        }
    }
}
