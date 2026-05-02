# SolCatEars

Steam Frame eye tracking → cat ear servos via USB serial.

## Hardware
- **MCU:** Seeed Xiao ESP32-S3 (Xtensa LX7, native USB). Enumerates as `/dev/cu.usbmodem101` on this Mac.
- **Servos:** 4× SG90 (standard hobby, 50 Hz PWM, 1.0–2.0 ms pulse, 0–180°).
- **Pinout (Xiao silk → GPIO):**
  - `D0` / GPIO1 — left ear pan (X)
  - `D1` / GPIO2 — left ear tilt (Y)
  - `D2` / GPIO3 — right ear pan (X)
  - `D3` / GPIO4 — right ear tilt (Y)

## Serial protocol
- Transport: native USB Serial/JTAG (CDC). No baud rate matters.
- PC sends ASCII lines: `"%04d,%04d,%04d,%04d\n"` → `left_x,left_y,right_x,right_y`.
- Input range: nominal 0–1024; physical 256–768. Center ≈ 512.

## Mapping (all configurable at top of `firmware/src/main.rs`)
- `INPUT_MIN=256, INPUT_MAX=768, INPUT_CENTER=512`
- `EAR_DEFLECTION_DEG=30` — max deflection from servo neutral (90°).
- `SERVO_NEUTRAL_DEG=90`, pulse range 500–2400 µs.
- Default: each eye drives same-side ear, no deadzone, no smoothing. Swap/invert per-axis via `INVERT_*` and `SWAP_EYES` consts.

## Failsafe
- If no valid line for `FAILSAFE_TIMEOUT_MS` (default 500 ms), **go limp**: stop LEDC output so servos draw no current.

## Toolchain
- `espup` + `rust-toolchain.toml` pins the `esp` channel. Build env: `source ~/export-esp.sh`.
- Build + flash + monitor: `cd firmware && cargo run --release` (runner = `espflash flash --monitor`).
- Flash-only (to keep port free for the host tester): `espflash flash --port /dev/cu.usbmodem101 target/xtensa-esp32s3-none-elf/release/solcatears-fw`.
- **Upstream gotcha:** `esp-hal-embassy` is deprecated in the 1.0 era — its replacement is `esp-rtos` with the `embassy` feature (drives an embassy executor + time queue via esp-rtos). Source of truth for version matrix: the official example at `esp-rs/esp-hal:examples/async/embassy_usb_serial_jtag` on the `main` branch. Current pins: `esp-hal = "=1.1.0-rc.0"`, `esp-rtos = "0.3"`, `embassy-executor = "0.10"`, `embassy-sync = "0.8"`, `embedded-io-async = "0.7"`, `esp-bootloader-esp-idf = "0.5"`. Edition: 2024.
- **Linker flag on xtensa must be `-Wl,-Tlinkall.x`** (GNU LD syntax), not plain `-Tlinkall.x`. Wrong form silently misplaces `.flash.appdesc` so the 2nd-stage ESP-IDF bootloader reads garbage at offset 0x20 and refuses to boot (symptom: `boot_comm: Image requires efuse blk rev >= vXX.YY`).

## Host-side smoke test
- `python3 firmware/scripts/echo_test.py` — opens `/dev/cu.usbmodem101` raw (stdlib-only, no pyserial) and asserts OK/BAD/LIMP responses.
- Firmware echoes one line per input: `OK lx,ly,rx,ry us=p0,p1,p2,p3`, `BAD`, or `LIMP` on failsafe.





    Into host mode (for plugging devices like the ESP into the Frame's USB-C):
    sudo sh -c 'echo host > /sys/kernel/debug/usb/a600000.usb/mode'

    Back to device mode (default — Frame appears as a USB device to a host PC):
    sudo sh -c 'echo device > /sys/kernel/debug/usb/a600000.usb/mode'

    Check current mode:
    cat /sys/kernel/debug/usb/a600000.usb/mode

    Notes worth including:
    - After switching to host mode, unplug and replug any attached USB-C peripheral — devices attached during a prior role get into a stuck state and won't enumerate until re-cabled.
    - Verify host mode worked with lsusb (should show the xHCI root hubs 1d6b:0002 and 1d6b:0003) and ls /sys/bus/usb/devices/.
    - Role reverts to the DT default (dr_mode=otg, effectively device) on reboot — the debug-file write is not persistent.
    - Caveat from this session: on this kernel (6.18.0-g868fb94b2951) cdc_acm is not present as a module or built-in, so the ESP32-S3's native USB CDC does not get a /dev/ttyACM* node even
    in host mode. The ESP appears in lsusb as 303a:1001 Espressif USB JTAG/serial debug unit, but the tty is missing until cdc_acm support is added to the kernel. That's the outstanding
    blocker for catears on the Frame, not the role switch itself.